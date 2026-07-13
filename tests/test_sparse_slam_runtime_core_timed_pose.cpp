
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
    cfg.sparse_slam_pose_interpolation_max_gap_s = 1.0;
    SparseSlamRuntimeCore core(cfg);
    core.initialize(sparse_slam_initialization_request_from_config(cfg));
    core.step(snapshot(0.1, 0.0), 0.1);
    core.step(snapshot(0.2, 0.05, false), 0.2);
    auto s = snapshot(0.3, 0.05, true);
    s.multi_tof.front.effective_timestamp_s = 0.2;
    s.multi_tof.left.effective_timestamp_s = 0.25;
    s.multi_tof.right.effective_timestamp_s = 0.3;
    auto out = core.step(s, 0.3);
    const auto &r = core.report();
    expect(out.backend_called, "backend called after timed lookup");
    expect(r.front_pose_lookup_success_count == 1, "front lookup used front time");
    expect(r.left_pose_lookup_success_count == 1, "left lookup used left time");
    expect(r.right_pose_lookup_success_count == 1, "right lookup used right time");
    expect(r.measurement_pose_set_success_count == 1, "measurement pose set ready");
    expect(r.measurement_time_pose_consumed, "measurement-time pose consumed");
    expect(!r.single_pose_fallback_consumed, "single pose fallback disabled");
    expect(r.backend_accept_count == 1, "backend accepted strict update");
    return failures ? 1 : 0;
}
