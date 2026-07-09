#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_binding.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_map_quality_tracker.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <utility>

namespace robot_slamd {

class DeterministicSlamBackendBinding final : public SlamBackendBinding {
public:
    explicit DeterministicSlamBackendBinding(
        DeterministicSlamBackendOptions options = {})
        : options_(options),
          tof_evaluator_(options),
          quality_tracker_(options) {}

    bool ready() const override {
        return options_.enabled && options_.ready;
    }

    SlamBackendUpdateResult update(
        const SlamBackendInputFrame &input) override {
        state_.update_call_count++;
        state_.stage = DeterministicSlamBackendStage::ValidatingInput;
        if (!options_.enabled) {
            return reject(DeterministicSlamBackendFault::BackendDisabled,
                          SlamBackendFault::BackendNotReady,
                          "deterministic_slam_backend_disabled");
        }
        if (!ready()) {
            return reject(DeterministicSlamBackendFault::BackendNotReady,
                          SlamBackendFault::BackendNotReady,
                          "deterministic_slam_backend_not_ready");
        }
        if (!std::isfinite(input.timestamp_s)) {
            return reject(DeterministicSlamBackendFault::InvalidTimestamp,
                          SlamBackendFault::InvalidTimestamp,
                          "deterministic_slam_invalid_timestamp");
        }
        if (options_.require_tof && !input.snapshot.has_tof) {
            return reject(DeterministicSlamBackendFault::MissingTof,
                          SlamBackendFault::MissingTof,
                          "deterministic_slam_missing_tof");
        }
        if (options_.require_imu_or_wheel &&
            !input.snapshot.has_imu &&
            !input.snapshot.has_wheel) {
            return reject(DeterministicSlamBackendFault::MissingMotionReference,
                          SlamBackendFault::MissingMotionReference,
                          "deterministic_slam_missing_motion_reference");
        }

        state_.stage = DeterministicSlamBackendStage::EvaluatingTof;
        const auto stats = tof_evaluator_.evaluate(input.snapshot.tof);
        if (!stats.valid) {
            return reject(to_deterministic_fault(stats.reason),
                          to_slam_fault(stats.reason),
                          stats.reason);
        }

        state_.stage = DeterministicSlamBackendStage::UpdatingQuality;
        auto quality = quality_tracker_.update(input, stats);
        const auto tracker_state = quality_tracker_.state();
        const bool keyframe_added = should_add_keyframe(input, tracker_state);
        if (keyframe_added) {
            state_.keyframe_count++;
            state_.last_keyframe_yaw_rad = estimated_yaw_for_keyframe(input);
        }
        state_.accepted_update_count++;
        state_.valid_scan_count = tracker_state.valid_scan_count;
        state_.coverage_ratio = tracker_state.coverage_ratio;
        state_.yaw_coverage_ratio = tracker_state.yaw_coverage_ratio;
        state_.last_update_time_s = input.timestamp_s;
        state_.last_update_kind = keyframe_added
                                      ? DeterministicSlamBackendUpdateKind::KeyframeAdded
                                      : DeterministicSlamBackendUpdateKind::ScanIntegrated;
        state_.fault = DeterministicSlamBackendFault::None;
        state_.stage = DeterministicSlamBackendStage::Ready;
        state_.reason = "deterministic_slam_update_accepted";

        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Accepted;
        result.fault = SlamBackendFault::None;
        result.map_updated = true;
        result.keyframe_added = keyframe_added;
        result.integrated_scan_count = state_.valid_scan_count;
        result.update_latency_s = 0.0;
        result.quality = quality;
        result.message = "deterministic_slam_update_accepted";
        latest_quality_ = quality;
        return result;
    }

    RobotSlamMapQuality latest_quality(double now_s) const override {
        (void)now_s;
        return latest_quality_;
    }

    SlamBackendSaveResult save_map(
        const std::string &output_path) override {
        SlamBackendSaveResult result;
        result.output_path = output_path;
        result.ok = false;
        result.error = options_.allow_save_map
                           ? "deterministic_slam_save_not_implemented"
                           : "deterministic_slam_save_not_enabled";
        state_.fault = DeterministicSlamBackendFault::SaveNotImplemented;
        return result;
    }

    std::string status() const override {
        std::ostringstream o;
        o << "DeterministicSlamBackendBinding: ready=" << (ready() ? 1 : 0)
          << " stage=" << to_string(state_.stage)
          << " fault=" << to_string(state_.fault)
          << " valid_scan_count=" << state_.valid_scan_count
          << " coverage=" << state_.coverage_ratio
          << " yaw_coverage=" << state_.yaw_coverage_ratio;
        return o.str();
    }

    DeterministicSlamBackendState state() const {
        return state_;
    }

private:
    SlamBackendUpdateResult reject(
        DeterministicSlamBackendFault deterministic_fault,
        SlamBackendFault slam_fault,
        const std::string &message) {
        state_.rejected_update_count++;
        state_.last_update_kind = DeterministicSlamBackendUpdateKind::Rejected;
        state_.fault = deterministic_fault;
        state_.stage = DeterministicSlamBackendStage::Fault;
        state_.reason = message;
        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Rejected;
        result.fault = slam_fault;
        result.map_updated = false;
        result.integrated_scan_count = state_.valid_scan_count;
        result.quality = latest_quality_;
        result.message = message;
        return result;
    }

    DeterministicSlamBackendFault to_deterministic_fault(
        const std::string &reason) const {
        if (reason == "empty_tof_ranges") {
            return DeterministicSlamBackendFault::EmptyTofRanges;
        }
        if (reason == "invalid_angle_increment") {
            return DeterministicSlamBackendFault::InvalidAngleIncrement;
        }
        if (reason == "invalid_range_limit") {
            return DeterministicSlamBackendFault::InvalidRangeLimit;
        }
        return DeterministicSlamBackendFault::TooFewValidRanges;
    }

    SlamBackendFault to_slam_fault(const std::string &reason) const {
        (void)reason;
        return SlamBackendFault::InvalidTofScan;
    }

    bool should_add_keyframe(
        const SlamBackendInputFrame &input,
        const DeterministicSlamBackendState &tracker_state) const {
        if (tracker_state.valid_scan_count <= 1 || state_.keyframe_count == 0) {
            return true;
        }
        const double yaw_rad = estimated_yaw_for_keyframe(input);
        return std::abs(yaw_rad - state_.last_keyframe_yaw_rad) >=
               options_.keyframe_yaw_delta_rad;
    }

    double estimated_yaw_for_keyframe(const SlamBackendInputFrame &input) const {
        if (input.has_predicted_pose) {
            return input.predicted_pose.yaw_rad;
        }
        return static_cast<double>(state_.valid_scan_count) *
               std::max(options_.assumed_scan_yaw_span_rad,
                        options_.yaw_bin_size_rad);
    }

    DeterministicSlamBackendOptions options_;
    DeterministicTofScanEvaluator tof_evaluator_;
    DeterministicMapQualityTracker quality_tracker_;
    DeterministicSlamBackendState state_;
    RobotSlamMapQuality latest_quality_;
};

} // namespace robot_slamd
