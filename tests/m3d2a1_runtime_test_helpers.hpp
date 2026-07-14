
#pragma once

#include "robot_slamd/runtime/sparse_slam_runtime_core.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace m3d2a1_test {

inline int failures = 0;

inline void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

inline robot_slamd::ScalarTofSnapshotFrame frame(std::uint16_t mm,
                                                  double yaw,
                                                  const char *id,
                                                  double t) {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_mm = mm;
    out.distance_m = static_cast<double>(mm) / 1000.0;
    out.confidence = 100;
    out.echo_tag_u48 = 0x410000 + mm;
    out.effective_timestamp_s = t;
    out.protocol_valid = true;
    out.usable_for_mapping = true;
    out.full_fov_rad = 1.6 * robot_slamd::kPi / 180.0;
    out.mount_yaw_rad = yaw;
    out.frame_id = id;
    out.source = "m3d2a1_runtime_test";
    return out;
}

inline robot_slamd::RobotSlamSensorSnapshot snapshot(double t,
                                                      double linear,
                                                      double wheel_yaw = 0.0,
                                                      double imu_yaw = 0.0,
                                                      bool multi = true) {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_wheel = true;
    s.wheel.timestamp_s = t;
    s.wheel.linear_mps = linear;
    s.wheel.angular_rad_s = wheel_yaw;
    s.wheel.valid = true;
    s.has_imu = true;
    s.imu.timestamp_s = t;
    s.imu.yaw_rate_rad_s = imu_yaw;
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
        s.multi_tof.source = "m3d2a1_runtime_test";
    }
    return s;
}

inline robot_slamd::Config config() {
    robot_slamd::Config cfg;
    cfg.sparse_slam_pose_interpolation_max_gap_s = 1.0;
    cfg.sparse_slam_active_bundle_min_snapshots = 2;
    cfg.sparse_slam_active_bundle_min_matchable_rays = 3;
    cfg.sparse_slam_active_bundle_min_yaw_span_rad = 0.0;
    cfg.sparse_slam_active_bundle_max_snapshots = 8;
    cfg.sparse_slam_active_bundle_max_rays = 24;
    cfg.sparse_slam_active_bundle_max_duration_s = 5.0;
    cfg.sparse_slam_active_bundle_max_snapshot_gap_s = 0.30;
    cfg.sparse_slam_settle_min_consecutive_samples = 3;
    cfg.sparse_slam_settle_min_stable_duration_s = 0.20;
    cfg.sparse_slam_settle_max_sample_gap_s = 0.20;
    cfg.sparse_slam_settle_max_abs_linear_speed_mps = 0.02;
    cfg.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s = 0.02;
    cfg.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s = 0.02;
    return cfg;
}

inline robot_slamd::SparseSlamRuntimeCore ready_core(robot_slamd::Config cfg = config()) {
    robot_slamd::SparseSlamRuntimeCore core(cfg);
    core.initialize(robot_slamd::sparse_slam_initialization_request_from_config(cfg));
    auto init = core.step(snapshot(0.1, 0.0, 0.0, 0.0, true), 0.1);
    expect(init.initialized, "core initialized by stationary sample");
    return core;
}

inline robot_slamd::SparseSlamStepRequest request(
    const robot_slamd::RobotSlamSensorSnapshot &s,
    robot_slamd::ActiveObservationControl control = robot_slamd::ActiveObservationControl::None,
    std::uint64_t bundle_id = 0) {
    robot_slamd::SparseSlamStepRequest req;
    req.snapshot = s;
    req.now_s = s.has_wheel ? s.wheel.timestamp_s : 0.0;
    req.observation_control = control;
    req.bundle_id = bundle_id;
    return req;
}

} // namespace m3d2a1_test
