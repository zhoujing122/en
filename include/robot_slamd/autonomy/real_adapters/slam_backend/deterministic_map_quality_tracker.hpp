#pragma once

#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_tof_scan_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace robot_slamd {

class DeterministicMapQualityTracker {
public:
    explicit DeterministicMapQualityTracker(
        DeterministicSlamBackendOptions options = {})
        : options_(options) {
        const int bin_count = std::max(
            1,
            static_cast<int>(std::ceil((2.0 * kPi) / options_.yaw_bin_size_rad)));
        yaw_bins_.assign(static_cast<std::size_t>(bin_count), false);
        latest_quality_.reason = "deterministic_quality_too_few_scans";
    }

    RobotSlamMapQuality update(
        const SlamBackendInputFrame &input,
        const DeterministicTofScanStats &stats) {
        if (!stats.valid) {
            latest_quality_.good_enough = false;
            latest_quality_.reason = stats.reason;
            return latest_quality_;
        }

        state_.valid_scan_count++;
        state_.accepted_update_count++;
        state_.last_update_time_s = input.timestamp_s;
        const double yaw_rad = normalize_yaw_rad(estimate_yaw_delta_rad(input));
        mark_yaw_coverage(yaw_rad);

        state_.coverage_ratio = std::min(
            1.0,
            static_cast<double>(state_.valid_scan_count) /
                static_cast<double>(options_.min_valid_scan_count_for_good));
        const int covered_bins = static_cast<int>(std::count(
            yaw_bins_.begin(), yaw_bins_.end(), true));
        state_.yaw_coverage_ratio =
            static_cast<double>(covered_bins) /
            static_cast<double>(yaw_bins_.size());

        latest_quality_.coverage_ratio = state_.coverage_ratio;
        latest_quality_.yaw_coverage_ratio = state_.yaw_coverage_ratio;
        latest_quality_.valid_scan_count = state_.valid_scan_count;
        latest_quality_.good_enough =
            state_.valid_scan_count >= options_.min_valid_scan_count_for_good &&
            state_.coverage_ratio >= options_.min_coverage_ratio_for_good &&
            state_.yaw_coverage_ratio >= options_.min_yaw_coverage_ratio_for_good;
        if (latest_quality_.good_enough) {
            latest_quality_.reason = "deterministic_quality_good";
        } else if (state_.valid_scan_count < options_.min_valid_scan_count_for_good) {
            latest_quality_.reason = "deterministic_quality_too_few_scans";
        } else if (state_.coverage_ratio < options_.min_coverage_ratio_for_good) {
            latest_quality_.reason = "deterministic_quality_low_coverage";
        } else {
            latest_quality_.reason = "deterministic_quality_low_yaw_coverage";
        }
        state_.reason = latest_quality_.reason;
        return latest_quality_;
    }

    RobotSlamMapQuality latest_quality(double now_s) const {
        (void)now_s;
        return latest_quality_;
    }

    DeterministicSlamBackendState state() const {
        return state_;
    }

private:
    double estimate_yaw_delta_rad(const SlamBackendInputFrame &input) const {
        if (input.has_predicted_pose) {
            return input.predicted_pose.yaw_rad;
        }
        const double sequence_yaw =
            static_cast<double>(state_.valid_scan_count) *
            std::max(options_.assumed_scan_yaw_span_rad,
                     options_.yaw_bin_size_rad);
        double motion_yaw = 0.0;
        if (input.snapshot.has_wheel) {
            motion_yaw += input.snapshot.wheel.angular_rad_s * 0.5;
        }
        if (input.snapshot.has_imu) {
            motion_yaw += input.snapshot.imu.yaw_rate_rad_s * 0.25;
        }
        return sequence_yaw + motion_yaw;
    }

    double normalize_yaw_rad(double yaw_rad) const {
        while (yaw_rad < 0.0) {
            yaw_rad += 2.0 * kPi;
        }
        while (yaw_rad >= 2.0 * kPi) {
            yaw_rad -= 2.0 * kPi;
        }
        return yaw_rad;
    }

    void mark_yaw_coverage(double yaw_rad) {
        if (yaw_bins_.empty()) {
            return;
        }
        const int index = static_cast<int>(yaw_rad / options_.yaw_bin_size_rad) %
                          static_cast<int>(yaw_bins_.size());
        yaw_bins_[static_cast<std::size_t>(index)] = true;
    }

    DeterministicSlamBackendOptions options_;
    DeterministicSlamBackendState state_;
    RobotSlamMapQuality latest_quality_;
    std::vector<bool> yaw_bins_;
};

} // namespace robot_slamd
