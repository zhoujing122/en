#pragma once

#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_multi_tof_scan_evaluator.hpp"

#include <algorithm>

namespace robot_slamd {

class DeterministicMultiTofMapQualityTracker {
public:
    explicit DeterministicMultiTofMapQualityTracker(
        DeterministicMultiTofBackendOptions options = {})
        : options_(options) {
        latest_quality_.reason = "deterministic_multi_tof_quality_too_few_scans";
    }

    RobotSlamMapQuality update(
        const SlamBackendInputFrame &input,
        const DeterministicMultiTofScanStats &stats) {
        (void)input;
        if (!stats.valid) {
            latest_quality_.good_enough = false;
            latest_quality_.reason = stats.reason;
            return latest_quality_;
        }
        state_.accepted_update_count++;
        state_.valid_scan_count++;
        state_.valid_tof_count = stats.valid_tof_count;
        state_.coverage_ratio = std::min(
            1.0,
            state_.coverage_ratio +
                stats.coverage_contribution /
                    static_cast<double>(std::max(1, options_.scans_to_good_quality)));
        state_.yaw_coverage_ratio = std::min(
            1.0,
            state_.yaw_coverage_ratio +
                stats.yaw_coverage_contribution /
                    static_cast<double>(std::max(1, options_.scans_to_good_quality)));
        latest_quality_.coverage_ratio = state_.coverage_ratio;
        latest_quality_.yaw_coverage_ratio = state_.yaw_coverage_ratio;
        latest_quality_.valid_scan_count = state_.valid_scan_count;
        latest_quality_.good_enough =
            state_.valid_scan_count >= options_.scans_to_good_quality &&
            state_.coverage_ratio >= options_.min_good_coverage_ratio &&
            state_.yaw_coverage_ratio >= options_.min_good_yaw_coverage_ratio;
        if (latest_quality_.good_enough) {
            latest_quality_.reason = "deterministic_multi_tof_quality_good";
        } else if (state_.valid_scan_count < options_.scans_to_good_quality) {
            latest_quality_.reason = "deterministic_multi_tof_quality_too_few_scans";
        } else if (state_.coverage_ratio < options_.min_good_coverage_ratio) {
            latest_quality_.reason = "deterministic_multi_tof_quality_low_coverage";
        } else {
            latest_quality_.reason = "deterministic_multi_tof_quality_low_yaw_coverage";
        }
        state_.reason = latest_quality_.reason;
        return latest_quality_;
    }

    RobotSlamMapQuality latest_quality(double now_s) const {
        (void)now_s;
        return latest_quality_;
    }

    DeterministicMultiTofBackendState state() const {
        return state_;
    }

private:
    DeterministicMultiTofBackendOptions options_;
    DeterministicMultiTofBackendState state_;
    RobotSlamMapQuality latest_quality_;
};

} // namespace robot_slamd
