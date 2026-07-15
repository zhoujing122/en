
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
    Config moving_cfg;
    SparseSlamRuntimeCore moving(moving_cfg);
    moving.initialize(sparse_slam_initialization_request_from_config(moving_cfg));
    auto moved = moving.step(snapshot(0.1, 0.2), 0.1);
    expect(!moved.ok, "moving startup rejected");
    expect(moving.report().backend_update_attempt_count == 0, "moving startup does not map");

    Config load_cfg;
    load_cfg.sparse_slam_map_startup_mode = "load_existing";
    load_cfg.sparse_slam_initial_pose_mode = "configured_pose";
    load_cfg.sparse_slam_has_configured_pose = true;
    SparseSlamRuntimeCore load(load_cfg);
    auto init = load.initialize(sparse_slam_initialization_request_from_config(load_cfg));
    expect(!init.ok, "load existing fail closed");
    expect(load.report().initialization_status == "initial_pose_missing",
           "missing map path and extrinsics fail closed");

    Config relocal_cfg;
    relocal_cfg.sparse_slam_map_startup_mode = "load_existing";
    relocal_cfg.sparse_slam_initial_pose_mode = "relocalization";
    SparseSlamRuntimeCore relocal(relocal_cfg);
    auto relocal_init = relocal.initialize(sparse_slam_initialization_request_from_config(relocal_cfg));
    expect(!relocal_init.ok, "relocalization fail closed");
    expect(relocal.report().initialization_status == "initial_pose_missing",
           "missing map path and extrinsics reject before relocalization");
    return failures ? 1 : 0;
}
