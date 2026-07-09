#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_binding.hpp"

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
robot_slamd::DeterministicSlamBackendBinding backend() {
    robot_slamd::DeterministicSlamBackendOptions options;
    options.enabled = true;
    options.ready = true;
    return robot_slamd::DeterministicSlamBackendBinding(options);
}
robot_slamd::RobotSlamSensorSnapshot valid_snapshot() {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_tof = true;
    s.has_wheel = true;
    s.tof.ranges_m = {0.5, 1.0, 2.0, 3.0};
    s.tof.angle_increment_rad = 0.1;
    s.tof.max_range_m = 4.0;
    s.wheel.valid = true;
    return s;
}
robot_slamd::SlamBackendInputFrame input(robot_slamd::RobotSlamSensorSnapshot s) {
    robot_slamd::SlamBackendInputFrame frame;
    frame.timestamp_s = 100.0;
    frame.snapshot = s;
    return frame;
}
}
int main() {
    using namespace robot_slamd;
    auto missing_tof = valid_snapshot();
    missing_tof.has_tof = false;
    auto b1 = backend();
    expect(b1.update(input(missing_tof)).fault == SlamBackendFault::MissingTof, "missing tof");
    auto missing_motion = valid_snapshot();
    missing_motion.has_wheel = false;
    missing_motion.has_imu = false;
    auto b2 = backend();
    expect(b2.update(input(missing_motion)).fault == SlamBackendFault::MissingMotionReference, "missing motion");
    auto invalid_time = input(valid_snapshot());
    invalid_time.timestamp_s = std::numeric_limits<double>::quiet_NaN();
    auto b3 = backend();
    expect(b3.update(invalid_time).fault == SlamBackendFault::InvalidTimestamp, "invalid time");
    auto invalid_tof = valid_snapshot();
    invalid_tof.tof.ranges_m.clear();
    auto b4 = backend();
    expect(b4.update(input(invalid_tof)).fault == SlamBackendFault::InvalidTofScan, "invalid tof");
    DeterministicSlamBackendBinding disabled;
    expect(disabled.update(input(valid_snapshot())).fault == SlamBackendFault::BackendNotReady, "disabled rejected");
    expect(!disabled.save_map("/tmp/no_map").ok, "save disabled rejected");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
