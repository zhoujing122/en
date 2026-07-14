#pragma once

#include "robot_slamd/runtime/active_multi_tof_observation_bundle.hpp"
#include "robot_slamd/runtime/motion_settle_gate.hpp"

#include <cstdint>
#include <string>

namespace robot_slamd {

enum class ActiveObservationControl {
    None,
    BeginCollection,
    EndMotionAndWaitForSettle,
    DiscardFrozenBundle
};

inline std::string to_string(ActiveObservationControl control) {
    switch (control) {
    case ActiveObservationControl::None:
        return "none";
    case ActiveObservationControl::BeginCollection:
        return "begin_collection";
    case ActiveObservationControl::EndMotionAndWaitForSettle:
        return "end_motion_and_wait_for_settle";
    case ActiveObservationControl::DiscardFrozenBundle:
        return "discard_frozen_bundle";
    }
    return "unknown";
}

struct SparseMapRevisionGuard {
    std::uint64_t reference_map_revision = 0;
    std::size_t reference_map_cell_count = 0;
    bool active = false;

    bool unchanged(std::uint64_t current_revision,
                   std::size_t current_cell_count) const {
        return !active ||
               (current_revision == reference_map_revision &&
                current_cell_count == reference_map_cell_count);
    }
};

struct PhaseAwareObservationControllerConfig {
    ActiveMultiTofObservationBundleConfig bundle;
    MotionSettleGateConfig settle;
};

struct PhaseAwareObservationControllerReport {
    ActiveObservationBundleState state = ActiveObservationBundleState::Idle;
    std::uint64_t bundle_id = 0;
    std::size_t begin_count = 0;
    std::size_t end_motion_count = 0;
    std::size_t freeze_attempt_count = 0;
    std::size_t freeze_success_count = 0;
    std::size_t abort_count = 0;
    std::size_t discard_count = 0;
    std::size_t invalid_transition_count = 0;
    std::size_t snapshot_attempt_count = 0;
    std::size_t snapshot_accept_count = 0;
    std::size_t snapshot_reject_count = 0;
    std::size_t settle_update_count = 0;
    std::size_t settle_reset_count = 0;
    std::size_t settle_consecutive_sample_count = 0;
    double settle_stable_duration_s = 0.0;
    std::size_t settle_success_count = 0;
    std::uint64_t reference_map_revision = 0;
    std::size_t reference_map_cell_count = 0;
    bool frozen_bundle_available = false;
    bool frozen_bundle_immutable = true;
    std::string last_fault = "none";
    std::string last_message;
};

class PhaseAwareObservationController {
public:
    explicit PhaseAwareObservationController(
        PhaseAwareObservationControllerConfig config = {})
        : config_(config),
          builder_(config.bundle),
          settle_gate_(config.settle) {}

    ActiveObservationBundleState state() const { return report_.state; }
    bool map_commit_allowed() const {
        return report_.state == ActiveObservationBundleState::Idle;
    }

    ActiveMultiTofObservationResult begin_collection(
        std::uint64_t current_map_revision,
        std::size_t current_map_cell_count,
        const MapFromOdom2D &map_from_odom) {
        if (report_.state != ActiveObservationBundleState::Idle) {
            return invalid_transition("active_bundle_begin_invalid_state");
        }
        const std::uint64_t id = next_bundle_id_++;
        const auto result = builder_.begin(id, current_map_revision,
                                           current_map_cell_count,
                                           map_from_odom);
        if (!result.ok) {
            report_.invalid_transition_count++;
            set_fault(result);
            return result;
        }
        guard_.active = true;
        guard_.reference_map_revision = current_map_revision;
        guard_.reference_map_cell_count = current_map_cell_count;
        report_.begin_count++;
        sync_report("active_bundle_collection_started");
        return result;
    }

    ActiveMultiTofObservationResult end_motion_and_wait_for_settle() {
        if (report_.state != ActiveObservationBundleState::CollectingDuringMotion) {
            return invalid_transition("active_bundle_end_motion_invalid_state");
        }
        const auto result = builder_.mark_waiting_for_motion_settle();
        if (result.ok) {
            report_.end_motion_count++;
            settle_gate_.reset();
            report_.settle_reset_count++;
            sync_report(result.message);
        } else {
            set_fault(result);
        }
        return result;
    }

    ActiveMultiTofObservationResult append_observation(
        const ActiveMultiTofObservationAppendInput &input) {
        if (report_.state != ActiveObservationBundleState::CollectingDuringMotion &&
            report_.state != ActiveObservationBundleState::WaitingForMotionSettle) {
            return invalid_transition("active_bundle_append_invalid_state");
        }
        report_.snapshot_attempt_count++;
        const auto result = builder_.append(input);
        if (result.ok) {
            report_.snapshot_accept_count++;
            sync_report(result.message);
        } else {
            report_.snapshot_reject_count++;
            sync_report(result.message);
            if (builder_.summary().state == ActiveObservationBundleState::Aborted) {
                report_.abort_count++;
            }
        }
        return result;
    }

    MotionSettleGateResult update_settle(const MotionSettleSample &sample,
                                         std::uint64_t current_map_revision) {
        MotionSettleGateResult out;
        if (report_.state != ActiveObservationBundleState::WaitingForMotionSettle) {
            out.reason = "motion_settle_requires_waiting_state";
            report_.invalid_transition_count++;
            return out;
        }
        report_.settle_update_count++;
        out = settle_gate_.update(sample);
        if (out.reset) {
            report_.settle_reset_count++;
        }
        report_.settle_consecutive_sample_count = out.consecutive_sample_count;
        report_.settle_stable_duration_s = out.stable_duration_s;
        if (out.stable) {
            report_.settle_success_count++;
            report_.freeze_attempt_count++;
            const auto frozen = builder_.freeze(current_map_revision);
            if (frozen.ok) {
                report_.freeze_success_count++;
            } else {
                report_.abort_count++;
            }
            sync_report(frozen.message);
        }
        return out;
    }

    ActiveMultiTofObservationResult verify_reference_map(
        std::uint64_t current_map_revision,
        std::size_t current_map_cell_count) {
        if (report_.state == ActiveObservationBundleState::Idle) {
            return {true, ActiveObservationBundleFault::None, "reference_map_idle"};
        }
        if (!guard_.unchanged(current_map_revision, current_map_cell_count)) {
            builder_.abort(
                ActiveObservationBundleFault::ReferenceMapChangedDuringActiveBundle,
                "reference_map_changed_during_active_bundle");
            report_.abort_count++;
            sync_report("reference_map_changed_during_active_bundle");
            return {false,
                    ActiveObservationBundleFault::ReferenceMapChangedDuringActiveBundle,
                    "reference_map_changed_during_active_bundle"};
        }
        return {true, ActiveObservationBundleFault::None, "reference_map_unchanged"};
    }

    ActiveMultiTofObservationResult discard(std::uint64_t bundle_id) {
        if (report_.state != ActiveObservationBundleState::FrozenReady &&
            report_.state != ActiveObservationBundleState::Aborted) {
            return invalid_transition("active_bundle_discard_invalid_state");
        }
        const auto result = builder_.discard(bundle_id);
        if (result.ok) {
            guard_ = SparseMapRevisionGuard{};
            settle_gate_.reset();
            report_.discard_count++;
            sync_report(result.message);
        } else {
            set_fault(result);
        }
        return result;
    }

    const PhaseAwareObservationControllerReport &report() const {
        return report_;
    }
    const ActiveMultiTofObservationBundleSummary &bundle_summary() const {
        return builder_.summary();
    }
    const FrozenMultiTofObservationBundle &frozen_bundle() const {
        return builder_.frozen_bundle();
    }

private:
    ActiveMultiTofObservationResult invalid_transition(
        const std::string &message) {
        report_.invalid_transition_count++;
        report_.last_fault = to_string(ActiveObservationBundleFault::InvalidState);
        report_.last_message = message;
        return {false, ActiveObservationBundleFault::InvalidState, message};
    }

    void set_fault(const ActiveMultiTofObservationResult &result) {
        report_.last_fault = to_string(result.fault);
        report_.last_message = result.message;
    }

    void sync_report(const std::string &message) {
        const auto &summary = builder_.summary();
        report_.state = summary.state;
        report_.bundle_id = summary.bundle_id;
        report_.reference_map_revision = summary.reference_map_revision;
        report_.reference_map_cell_count = summary.reference_map_cell_count;
        report_.frozen_bundle_available = frozen_bundle().available();
        report_.frozen_bundle_immutable = true;
        report_.last_fault = to_string(summary.fault);
        report_.last_message = message;
    }

    PhaseAwareObservationControllerConfig config_;
    ActiveMultiTofObservationBundleBuilder builder_;
    MotionSettleGate settle_gate_;
    SparseMapRevisionGuard guard_;
    PhaseAwareObservationControllerReport report_;
    std::uint64_t next_bundle_id_ = 1;
};

} // namespace robot_slamd
