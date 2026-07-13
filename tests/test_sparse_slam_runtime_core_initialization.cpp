
#include "robot_slamd/runtime/sparse_slam_runtime_core.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; }
}
robot_slamd::ScalarTofSnapshotFrame frame(std::uint16_t mm, double yaw, const char *id, double t) {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_mm = mm;
    out.distance_m = static_cast<double>(mm) / 1000.0;
    out.confidence = 100;
    out.echo_tag_u48 = 0x330000 + mm;
    out.effective_timestamp_s = t;
    out.protocol_valid = true;
    out.usable_for_mapping = true;
    out.full_fov_rad = 1.6 * robot_slamd::kPi / 180.0;
    out.mount_yaw_rad = yaw;
    out.frame_id = id;
    out.source = "m3d2a0_core_test";
    return out;
}
robot_slamd::RobotSlamSensorSnapshot snapshot(double t, double linear, bool multi = true) {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_wheel = true;
    s.wheel.timestamp_s = t;
    s.wheel.linear_mps = linear;
    s.wheel.angular_rad_s = 0.0;
    s.wheel.valid = true;
    s.has_imu = true;
    s.imu.timestamp_s = t;
    s.imu.yaw_rate_rad_s = 0.0;
    s.imu.az_mps2 = 9.8;
    s.has_tof = false;
    s.has_multi_tof = multi;
    if (multi) {
        s.multi_tof.has_front = true;
        s.multi_tof.has_left = true;
        s.multi_tof.has_right = true;
        s.multi_tof.valid_tof_count = 3;
        s.multi_tof.front = frame(1000, 0.0, "front", t);
        s.multi_tof.left = frame(1200, robot_slamd::kPi / 2.0, "left", t);
        s.multi_tof.right = frame(1200, -robot_slamd::kPi / 2.0, "right", t);
        s.multi_tof.synchronized_time_s = t;
    }
    return s;
}
}

int main() {
    using namespace robot_slamd;
    Config cfg;
    SparseSlamRuntimeCore core(cfg);
    auto init = core.initialize(sparse_slam_initialization_request_from_config(cfg));
    expect(!init.ok, "initialization waits for stationary samples");
    auto before = core.report();
    expect(before.backend_update_attempt_count == 0, "no backend before samples");
    auto first = core.step(snapshot(0.1, 0.0), 0.1);
    expect(first.initialized, "first stationary sample initializes core");
    const auto &r = core.report();
    expect(r.initialization_complete, "initialization complete");
    expect(r.startup_zero_used, "startup zero used");
    expect(r.initial_yaw_defined_by_startup_frame, "yaw defined by startup frame");
    expect(!r.initial_yaw_measured_by_imu, "yaw not measured by imu");
    expect(r.stationary_check_passed, "stationary gate passed");
    expect(r.wheel_baseline_established, "wheel baseline established");
    expect(r.map_from_odom_initialized, "map from odom initialized");
    expect(r.backend_update_attempt_count == 0, "no map update during initialization");
    return failures ? 1 : 0;
}
