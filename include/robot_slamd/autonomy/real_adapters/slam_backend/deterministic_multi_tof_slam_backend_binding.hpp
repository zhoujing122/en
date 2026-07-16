#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_binding.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_multi_tof_map_quality_tracker.hpp"

#include <cmath>
#include <sstream>
#include <string>

namespace robot_slamd {

class DeterministicMultiTofSlamBackendBinding final : public SlamBackendBinding {
public:
    explicit DeterministicMultiTofSlamBackendBinding(
        DeterministicMultiTofBackendOptions options = {})
        : options_(options),
          evaluator_(options),
          quality_tracker_(options) {}

    bool ready() const override {
        return options_.enabled && options_.ready;
    }

    SlamBackendUpdateResult update(
        const SlamBackendInputFrame &input) override {
        state_.update_call_count++;
        state_.stage = DeterministicMultiTofBackendStage::ValidatingInput;
        if (!options_.enabled) {
            return reject(DeterministicMultiTofBackendFault::BackendDisabled,
                          SlamBackendFault::BackendNotReady,
                          "deterministic_multi_tof_backend_disabled");
        }
        if (!ready()) {
            return reject(DeterministicMultiTofBackendFault::BackendNotReady,
                          SlamBackendFault::BackendNotReady,
                          "deterministic_multi_tof_backend_not_ready");
        }
        if (!std::isfinite(input.timestamp_s)) {
            return reject(DeterministicMultiTofBackendFault::InvalidTimestamp,
                          SlamBackendFault::InvalidTimestamp,
                          "deterministic_multi_tof_invalid_timestamp");
        }
        if (options_.require_multi_tof && !input.snapshot.has_multi_tof) {
            return reject(DeterministicMultiTofBackendFault::MissingMultiTof,
                          SlamBackendFault::InvalidTofScan,
                          "deterministic_multi_tof_missing_multi_tof");
        }
        if (options_.require_imu_or_wheel &&
            !input.snapshot.has_imu &&
            !input.snapshot.has_wheel) {
            return reject(DeterministicMultiTofBackendFault::MissingMotionReference,
                          SlamBackendFault::MissingMotionReference,
                          "deterministic_multi_tof_missing_motion_reference");
        }

        state_.stage = DeterministicMultiTofBackendStage::EvaluatingMultiTof;
        const auto stats = evaluator_.evaluate(input.snapshot);
        if (!stats.valid) {
            return reject(to_fault(stats.reason),
                          SlamBackendFault::InvalidTofScan,
                          stats.reason);
        }

        state_.stage = DeterministicMultiTofBackendStage::UpdatingQuality;
        auto quality = quality_tracker_.update(input, stats);
        const auto tracker_state = quality_tracker_.state();
        state_.accepted_update_count++;
        state_.valid_tof_count = stats.valid_tof_count;
        state_.valid_scan_count = tracker_state.valid_scan_count;
        state_.coverage_ratio = tracker_state.coverage_ratio;
        state_.yaw_coverage_ratio = tracker_state.yaw_coverage_ratio;
        state_.last_update_time_s = input.timestamp_s;
        state_.fault = DeterministicMultiTofBackendFault::None;
        state_.stage = DeterministicMultiTofBackendStage::Ready;
        state_.reason = "deterministic_multi_tof_update_accepted";
        latest_quality_ = quality;

        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Accepted;
        result.fault = SlamBackendFault::None;
        result.map_updated = true;
        result.keyframe_added = state_.accepted_update_count == 1;
        result.integrated_scan_count = state_.valid_scan_count;
        result.update_latency_s = 0.0;
        result.quality = quality;
        result.message = "deterministic_multi_tof_update_accepted";
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
        result.error = "deterministic_multi_tof_save_not_implemented";
        state_.fault = DeterministicMultiTofBackendFault::SaveNotImplemented;
        return result;
    }

    std::string status() const override {
        std::ostringstream o;
        o << "DeterministicMultiTofSlamBackendBinding: ready="
          << (ready() ? 1 : 0)
          << " stage=" << to_string(state_.stage)
          << " fault=" << to_string(state_.fault)
          << " valid_tof_count=" << state_.valid_tof_count
          << " coverage=" << state_.coverage_ratio
          << " yaw_coverage=" << state_.yaw_coverage_ratio;
        return o.str();
    }

    DeterministicMultiTofBackendState state() const {
        return state_;
    }

private:
    SlamBackendUpdateResult reject(
        DeterministicMultiTofBackendFault fault,
        SlamBackendFault slam_fault,
        const std::string &message) {
        state_.rejected_update_count++;
        state_.fault = fault;
        state_.stage = DeterministicMultiTofBackendStage::Fault;
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

    DeterministicMultiTofBackendFault to_fault(
        const std::string &reason) const {
        if (reason == "missing_multi_tof") {
            return DeterministicMultiTofBackendFault::MissingMultiTof;
        }
        if (reason == "too_few_tof_frames") {
            return DeterministicMultiTofBackendFault::TooFewTofFrames;
        }
        if (reason == "empty_front_ranges") {
            return DeterministicMultiTofBackendFault::EmptyFrontRanges;
        }
        if (reason == "empty_left_ranges") {
            return DeterministicMultiTofBackendFault::EmptyLeftRanges;
        }
        if (reason == "empty_right_ranges") {
            return DeterministicMultiTofBackendFault::EmptyRightRanges;
        }
        if (reason == "too_few_valid_ranges") {
            return DeterministicMultiTofBackendFault::TooFewValidRanges;
        }
        return DeterministicMultiTofBackendFault::InvalidFrame;
    }

    DeterministicMultiTofBackendOptions options_;
    DeterministicMultiTofScanEvaluator evaluator_;
    DeterministicMultiTofMapQualityTracker quality_tracker_;
    DeterministicMultiTofBackendState state_;
    RobotSlamMapQuality latest_quality_;
};

} // namespace robot_slamd
