#pragma once

#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_slamd {

class DeterministicTofScanEvaluator {
public:
    explicit DeterministicTofScanEvaluator(
        DeterministicSlamBackendOptions options = {})
        : options_(options) {}

    DeterministicTofScanStats evaluate(const TofScanFrame &tof) const {
        DeterministicTofScanStats stats;
        stats.total_range_count = static_cast<int>(tof.ranges_m.size());
        if (tof.ranges_m.empty()) {
            stats.reason = "empty_tof_ranges";
            return stats;
        }
        if (!std::isfinite(tof.angle_increment_rad) ||
            tof.angle_increment_rad <= 0.0) {
            stats.reason = "invalid_angle_increment";
            return stats;
        }
        if (!std::isfinite(tof.max_range_m) || tof.max_range_m <= 0.0) {
            stats.reason = "invalid_range_limit";
            return stats;
        }

        double sum = 0.0;
        stats.min_range_m = std::numeric_limits<double>::infinity();
        stats.max_range_m = 0.0;
        for (const double range_m : tof.ranges_m) {
            if (!std::isfinite(range_m)) {
                continue;
            }
            stats.finite_range_count++;
            if (range_m >= options_.min_range_m &&
                range_m <= options_.max_range_m &&
                range_m <= tof.max_range_m) {
                stats.valid_range_count++;
                if (range_m < 1.0) {
                    stats.near_obstacle_count++;
                }
                stats.min_range_m = std::min(stats.min_range_m, range_m);
                stats.max_range_m = std::max(stats.max_range_m, range_m);
                sum += range_m;
            }
        }
        if (stats.valid_range_count > 0) {
            stats.mean_range_m = sum / stats.valid_range_count;
        } else {
            stats.min_range_m = 0.0;
        }
        stats.valid_range_ratio =
            static_cast<double>(stats.valid_range_count) /
            static_cast<double>(stats.total_range_count);
        stats.horizontal_fov_rad =
            std::abs(tof.angle_increment_rad) *
            static_cast<double>(std::max(0, stats.total_range_count - 1));

        if (stats.valid_range_count < options_.min_valid_ranges) {
            stats.reason = "too_few_valid_ranges";
            return stats;
        }
        if (stats.valid_range_ratio < options_.min_valid_range_ratio) {
            stats.reason = "low_valid_range_ratio";
            return stats;
        }
        stats.valid = true;
        stats.reason = "tof_scan_ok";
        return stats;
    }

private:
    DeterministicSlamBackendOptions options_;
};

} // namespace robot_slamd
