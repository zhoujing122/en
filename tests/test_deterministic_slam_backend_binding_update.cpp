#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_binding.hpp"

#include <cstdio>
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
robot_slamd::RobotSlamSensorSnapshot valid_snapshot() {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_tof = true;
    s.has_imu = true;
    s.has_wheel = true;
    s.tof.timestamp_s = 100.0;
    s.tof.ranges_m = {0.5, 1.0, 2.0, 3.0};
    s.tof.angle_increment_rad = 0.1;
    s.tof.max_range_m = 4.0;
    s.imu.timestamp_s = 100.0;
    s.imu.yaw_rate_rad_s = 0.1;
    s.wheel.timestamp_s = 100.0;
    s.wheel.valid = true;
    s.wheel.angular_rad_s = 0.2;
    return s;
}
}
int main() {
    using namespace robot_slamd;
    DeterministicSlamBackendBinding default_backend;
    expect(!default_backend.ready(), "default backend not ready");
    DeterministicSlamBackendOptions options;
    options.enabled = true;
    options.ready = true;
    options.min_yaw_coverage_ratio_for_good = 0.10;
    DeterministicSlamBackendBinding backend(options);
    expect(backend.ready(), "enabled ready backend ready");
    SlamBackendInputFrame input;
    input.timestamp_s = 100.0;
    input.snapshot = valid_snapshot();
    const auto update = backend.update(input);
    expect(update.status == SlamBackendUpdateStatus::Accepted, "update accepted");
    expect(update.map_updated, "map updated");
    expect(update.integrated_scan_count == 1, "scan count increases");
    expect(update.keyframe_added, "first keyframe added");
    expect(backend.latest_quality(100.0).valid_scan_count == 1, "latest quality updated");
    const auto save = backend.save_map("/tmp/deterministic_slam_should_not_exist.map");
    expect(!save.ok, "save map disabled");
    expect(std::remove("/tmp/deterministic_slam_should_not_exist.map") != 0, "save did not write file");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
