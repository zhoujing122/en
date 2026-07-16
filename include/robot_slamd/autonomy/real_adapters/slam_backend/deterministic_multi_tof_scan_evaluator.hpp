#pragma once

#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_multi_tof_slam_backend_types.hpp"

#include <cmath>
#include <string>

namespace robot_slamd {

class DeterministicMultiTofScanEvaluator {
public:
    explicit DeterministicMultiTofScanEvaluator(
        DeterministicMultiTofBackendOptions options = {})
        : options_(options) {}

    DeterministicMultiTofScanStats evaluate(
        const RobotSlamSensorSnapshot &snapshot) const {
        DeterministicMultiTofScanStats stats;
        if (!snapshot.has_multi_tof) {
            stats.reason = "missing_multi_tof";
            return stats;
        }
        const auto &multi = snapshot.multi_tof;
        stats.has_front = multi.has_front;
        stats.has_left = multi.has_left;
        stats.has_right = multi.has_right;
        stats.present_tof_count = (multi.has_front ? 1 : 0) +
                                  (multi.has_left ? 1 : 0) +
                                  (multi.has_right ? 1 : 0);
        if (multi.valid_tof_count < options_.min_required_tof_count ||
            stats.present_tof_count < options_.min_required_tof_count) {
            stats.reason = "too_few_tof_frames";
            return stats;
        }
        if (!evaluate_present_frame(multi.front,
                                    multi.has_front,
                                    options_.front_coverage_weight,
                                    "invalid_front_scalar_tof",
                                    stats)) {
            return stats;
        }
        if (!evaluate_present_frame(multi.left,
                                    multi.has_left,
                                    options_.left_coverage_weight,
                                    "invalid_left_scalar_tof",
                                    stats)) {
            return stats;
        }
        if (!evaluate_present_frame(multi.right,
                                    multi.has_right,
                                    options_.right_coverage_weight,
                                    "invalid_right_scalar_tof",
                                    stats)) {
            return stats;
        }
        stats.valid_tof_count = 0;
        if (multi.has_front) {
            stats.valid_tof_count++;
        }
        if (multi.has_left) {
            stats.valid_tof_count++;
            stats.yaw_coverage_contribution += options_.yaw_coverage_per_side_tof;
        }
        if (multi.has_right) {
            stats.valid_tof_count++;
            stats.yaw_coverage_contribution += options_.yaw_coverage_per_side_tof;
        }
        if (multi.has_front && multi.has_left && multi.has_right) {
            stats.yaw_coverage_contribution =
                options_.yaw_coverage_full_three_tof;
        }
        stats.valid = true;
        stats.reason = multi.degraded ? "multi_tof_scan_ok_degraded"
                                      : "multi_tof_scan_ok";
        return stats;
    }

private:
    bool evaluate_present_frame(
        const ScalarTofSnapshotFrame &frame,
        bool present,
        double coverage_weight,
        const std::string &invalid_reason,
        DeterministicMultiTofScanStats &stats) const {
        if (!present) {
            return true;
        }
        stats.total_range_count += 1;
        if (!frame.protocol_valid || !frame.usable_for_mapping ||
            !std::isfinite(frame.distance_m) ||
            frame.distance_m < options_.min_range_m ||
            frame.distance_m > options_.max_range_m) {
            stats.reason = invalid_reason;
            return false;
        }
        stats.valid_range_count += 1;
        stats.coverage_contribution += coverage_weight;
        return true;
    }

    DeterministicMultiTofBackendOptions options_;
};

} // namespace robot_slamd
