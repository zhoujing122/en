#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_map_quality_tracker.hpp"

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
robot_slamd::SlamBackendInputFrame input(double time_s) {
    robot_slamd::SlamBackendInputFrame frame;
    frame.timestamp_s = time_s;
    frame.snapshot.has_wheel = true;
    frame.snapshot.wheel.valid = true;
    frame.snapshot.wheel.angular_rad_s = 0.2;
    return frame;
}
robot_slamd::DeterministicTofScanStats stats() {
    robot_slamd::DeterministicTofScanStats s;
    s.valid = true;
    s.valid_range_count = 4;
    s.total_range_count = 4;
    s.valid_range_ratio = 1.0;
    s.reason = "tof_scan_ok";
    return s;
}
}
int main() {
    using namespace robot_slamd;
    DeterministicSlamBackendOptions options;
    options.min_yaw_coverage_ratio_for_good = 0.10;
    DeterministicMapQualityTracker tracker(options);
    auto q1 = tracker.update(input(100.0), stats());
    expect(q1.valid_scan_count == 1, "scan count one");
    expect(q1.coverage_ratio > 0.0, "coverage grows");
    auto q2 = tracker.update(input(100.1), stats());
    expect(q2.yaw_coverage_ratio >= q1.yaw_coverage_ratio, "yaw coverage grows");
    auto q3 = tracker.update(input(100.2), stats());
    expect(q3.good_enough, "quality good after threshold");
    expect(q3.reason == "deterministic_quality_good", "good reason");
    expect(tracker.latest_quality(100.2).valid_scan_count == 3, "latest quality");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
