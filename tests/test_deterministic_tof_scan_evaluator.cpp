#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_tof_scan_evaluator.hpp"

#include <cmath>
#include <limits>
#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
} // namespace

namespace {
robot_slamd::TofScanFrame valid_tof() {
    robot_slamd::TofScanFrame tof;
    tof.timestamp_s = 100.0;
    tof.ranges_m = {0.5, 1.0, std::numeric_limits<double>::quiet_NaN(), 2.0, 3.0};
    tof.angle_increment_rad = 0.1;
    tof.max_range_m = 4.0;
    tof.frame_id = "tof_frame";
    return tof;
}
}

int main() {
    using namespace robot_slamd;
    DeterministicTofScanEvaluator evaluator;
    auto stats = evaluator.evaluate(valid_tof());
    expect(stats.valid, "valid tof pass");
    expect(stats.total_range_count == 5, "total count");
    expect(stats.finite_range_count == 4, "finite count");
    expect(stats.valid_range_count == 4, "valid count");
    expect(stats.near_obstacle_count == 1, "near obstacle count");
    expect(std::abs(stats.mean_range_m - 1.625) < 1e-9, "mean range");
    expect(stats.min_range_m == 0.5, "min range");
    expect(stats.max_range_m == 3.0, "max range");
    auto empty = valid_tof();
    empty.ranges_m.clear();
    expect(evaluator.evaluate(empty).reason == "empty_tof_ranges", "empty fail");
    auto bad_angle = valid_tof();
    bad_angle.angle_increment_rad = 0.0;
    expect(evaluator.evaluate(bad_angle).reason == "invalid_angle_increment", "angle fail");
    auto too_few = valid_tof();
    too_few.ranges_m = {std::numeric_limits<double>::quiet_NaN(), 0.5};
    expect(evaluator.evaluate(too_few).reason == "too_few_valid_ranges", "few fail");
    DeterministicSlamBackendOptions ratio_options;
    ratio_options.min_valid_range_ratio = 0.50;
    DeterministicTofScanEvaluator ratio_evaluator(ratio_options);
    auto low_ratio = valid_tof();
    low_ratio.ranges_m = {0.5, 1.0, 2.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0};
    expect(ratio_evaluator.evaluate(low_ratio).reason == "low_valid_range_ratio", "ratio fail");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
