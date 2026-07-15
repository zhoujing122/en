#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_multi_tof_relocalizer.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace robot_slamd {

enum class SparseRelocalizationState {
    Idle,
    CollectingFirstBundle,
    CollectingConfirmationBundle,
    Succeeded,
    Failed
};

inline const char *to_string(SparseRelocalizationState state) {
    switch (state) {
    case SparseRelocalizationState::Idle: return "idle";
    case SparseRelocalizationState::CollectingFirstBundle:
        return "collecting_first_bundle";
    case SparseRelocalizationState::CollectingConfirmationBundle:
        return "collecting_confirmation_bundle";
    case SparseRelocalizationState::Succeeded: return "succeeded";
    case SparseRelocalizationState::Failed: return "failed";
    }
    return "unknown";
}

enum class SparseRelocalizationPurpose {
    Startup,
    LostRecovery
};

inline const char *to_string(SparseRelocalizationPurpose purpose) {
    return purpose == SparseRelocalizationPurpose::Startup
        ? "startup" : "lost_recovery";
}

struct SparseRelocalizationManagerConfig {
    SparseMultiTofRelocalizerConfig search;
    std::size_t required_confirmation_bundles = 2;
    double confirmation_xy_tolerance_m = 0.15;
    double confirmation_yaw_tolerance_rad = 0.15;
};

inline bool sparse_relocalization_manager_config_valid(
    const SparseRelocalizationManagerConfig &config) {
    return sparse_relocalizer_config_valid(config.search) &&
           config.required_confirmation_bundles == 2 &&
           std::isfinite(config.confirmation_xy_tolerance_m) &&
           config.confirmation_xy_tolerance_m > 0.0 &&
           std::isfinite(config.confirmation_yaw_tolerance_rad) &&
           config.confirmation_yaw_tolerance_rad > 0.0;
}

struct SparseRelocalizationManagerResult {
    bool ok = false;
    bool bundle_consumed = false;
    bool needs_confirmation = false;
    bool succeeded = false;
    bool failed = false;
    SparseRelocalizationState state = SparseRelocalizationState::Idle;
    SparseMultiTofRelocalizationResult search;
    std::optional<MapFromOdom2D> accepted_map_from_odom;
    double confirmation_xy_difference_m = 0.0;
    double confirmation_yaw_difference_rad = 0.0;
    std::string reason;
};

class SparseRelocalizationManager {
public:
    explicit SparseRelocalizationManager(
        SparseRelocalizationManagerConfig config = {})
        : config_(config) {}

    bool start_startup(std::uint64_t reference_revision) {
        return start(SparseRelocalizationPurpose::Startup,
                     reference_revision, std::nullopt);
    }

    bool start_recovery(std::uint64_t reference_revision,
                        const MapFromOdom2D &prediction) {
        if (!sparse_slam_frame_pose_valid(prediction)) return false;
        return start(SparseRelocalizationPurpose::LostRecovery,
                     reference_revision, prediction);
    }

    SparseRelocalizationManagerResult process(
        const SparseTofLocalMatchInput &input) {
        SparseRelocalizationManagerResult out;
        out.state = state_;
        if (!sparse_relocalization_manager_config_valid(config_) ||
            (state_ != SparseRelocalizationState::CollectingFirstBundle &&
             state_ !=
                SparseRelocalizationState::CollectingConfirmationBundle) ||
            input.bundle_id == 0 ||
            input.expected_reference_map_revision != reference_revision_ ||
            input.current_runtime_map_revision != reference_revision_) {
            return fail(std::move(out), "relocalization_manager_contract_invalid");
        }
        if (state_ == SparseRelocalizationState::CollectingFirstBundle) {
            return process_first(input);
        }
        return process_confirmation(input);
    }

    void reset() {
        state_ = SparseRelocalizationState::Idle;
        purpose_ = SparseRelocalizationPurpose::Startup;
        reference_revision_ = 0;
        first_bundle_id_ = 0;
        first_bundle_start_s_ = 0.0;
        provisional_.reset();
        accepted_.reset();
        first_result_.reset();
        confirmation_result_.reset();
    }

    SparseRelocalizationState state() const { return state_; }
    SparseRelocalizationPurpose purpose() const { return purpose_; }
    std::uint64_t reference_revision() const { return reference_revision_; }
    std::uint64_t first_bundle_id() const { return first_bundle_id_; }
    const std::optional<MapFromOdom2D> &provisional() const {
        return provisional_;
    }
    const std::optional<MapFromOdom2D> &accepted() const { return accepted_; }
    const std::optional<SparseMultiTofRelocalizationResult> &first_result()
        const { return first_result_; }
    const std::optional<SparseMultiTofRelocalizationResult>
        &confirmation_result() const { return confirmation_result_; }

private:
    bool start(SparseRelocalizationPurpose purpose,
               std::uint64_t revision,
               std::optional<MapFromOdom2D> prediction) {
        if (state_ != SparseRelocalizationState::Idle ||
            !sparse_relocalization_manager_config_valid(config_)) {
            return false;
        }
        purpose_ = purpose;
        reference_revision_ = revision;
        recovery_prediction_ = prediction;
        state_ = SparseRelocalizationState::CollectingFirstBundle;
        return true;
    }

    SparseRelocalizationManagerResult process_first(
        const SparseTofLocalMatchInput &input) {
        auto search = purpose_ == SparseRelocalizationPurpose::LostRecovery &&
                              recovery_prediction_
            ? relocalizer_.search_local(input, *recovery_prediction_,
                                        config_.search)
            : relocalizer_.search_global(input, config_.search);
        if (!search.accepted &&
            purpose_ == SparseRelocalizationPurpose::LostRecovery) {
            search = relocalizer_.search_global(input, config_.search);
        }
        first_result_ = search;
        if (!search.accepted || !search.best) {
            SparseRelocalizationManagerResult out;
            out.search = search;
            return fail(std::move(out), search.reason);
        }
        first_bundle_id_ = input.bundle_id;
        first_bundle_start_s_ =
            input.frozen_bundle->summary().collection_start_timestamp_s;
        provisional_ = search.best->map_from_odom;
        state_ =
            SparseRelocalizationState::CollectingConfirmationBundle;
        SparseRelocalizationManagerResult out;
        out.ok = true;
        out.bundle_consumed = true;
        out.needs_confirmation = true;
        out.state = state_;
        out.search = search;
        out.reason = "relocalization_first_bundle_accepted";
        return out;
    }

    SparseRelocalizationManagerResult process_confirmation(
        const SparseTofLocalMatchInput &input) {
        if (!provisional_ || input.bundle_id == first_bundle_id_ ||
            input.frozen_bundle->summary().collection_start_timestamp_s <=
                first_bundle_start_s_) {
            SparseRelocalizationManagerResult out;
            return fail(std::move(out),
                        "relocalization_confirmation_bundle_not_independent");
        }
        const auto search = relocalizer_.search_local(
            input, *provisional_, config_.search);
        confirmation_result_ = search;
        if (!search.accepted || !search.best) {
            SparseRelocalizationManagerResult out;
            out.search = search;
            return fail(std::move(out), search.reason);
        }
        const double dx = search.best->map_from_odom.map_T_odom.x_m -
                          provisional_->map_T_odom.x_m;
        const double dy = search.best->map_from_odom.map_T_odom.y_m -
                          provisional_->map_T_odom.y_m;
        const double xy = std::hypot(dx, dy);
        const double yaw = std::fabs(sparse_slam_shortest_yaw_delta(
            search.best->map_from_odom.map_T_odom.yaw_rad,
            provisional_->map_T_odom.yaw_rad));
        if (xy > config_.confirmation_xy_tolerance_m ||
            yaw > config_.confirmation_yaw_tolerance_rad) {
            SparseRelocalizationManagerResult out;
            out.search = search;
            out.confirmation_xy_difference_m = xy;
            out.confirmation_yaw_difference_rad = yaw;
            return fail(std::move(out),
                        "relocalization_confirmation_pose_mismatch");
        }
        accepted_ = search.best->map_from_odom;
        state_ = SparseRelocalizationState::Succeeded;
        SparseRelocalizationManagerResult out;
        out.ok = true;
        out.bundle_consumed = true;
        out.succeeded = true;
        out.state = state_;
        out.search = search;
        out.accepted_map_from_odom = accepted_;
        out.confirmation_xy_difference_m = xy;
        out.confirmation_yaw_difference_rad = yaw;
        out.reason = "relocalization_confirmed";
        return out;
    }

    SparseRelocalizationManagerResult fail(
        SparseRelocalizationManagerResult out, const std::string &reason) {
        state_ = SparseRelocalizationState::Failed;
        out.ok = false;
        out.failed = true;
        out.state = state_;
        out.reason = reason;
        return out;
    }

    SparseRelocalizationManagerConfig config_;
    SparseMultiTofRelocalizer relocalizer_;
    SparseRelocalizationState state_ = SparseRelocalizationState::Idle;
    SparseRelocalizationPurpose purpose_ = SparseRelocalizationPurpose::Startup;
    std::uint64_t reference_revision_ = 0;
    std::uint64_t first_bundle_id_ = 0;
    double first_bundle_start_s_ = 0.0;
    std::optional<MapFromOdom2D> recovery_prediction_;
    std::optional<MapFromOdom2D> provisional_;
    std::optional<MapFromOdom2D> accepted_;
    std::optional<SparseMultiTofRelocalizationResult> first_result_;
    std::optional<SparseMultiTofRelocalizationResult> confirmation_result_;
};

} // namespace robot_slamd
