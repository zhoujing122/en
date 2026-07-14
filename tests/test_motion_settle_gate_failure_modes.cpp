#include "robot_slamd/runtime/phase_aware_observation_controller.hpp"
#include "robot_slamd/config/config.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void expect(bool cond, const std::string &msg) {
    if (!cond) throw std::runtime_error(msg);
}

constexpr double kTestPi = 3.14159265358979323846;

robot_slamd::MotionSettleSample settle_sample(double t,
                                              double linear = 0.0,
                                              double wheel_yaw = 0.0,
                                              double imu_yaw = 0.0) {
    robot_slamd::MotionSettleSample s;
    s.timestamp_s = t;
    s.wheel_fresh = true;
    s.imu_fresh = true;
    s.linear_mps = linear;
    s.wheel_angular_rad_s = wheel_yaw;
    s.imu_yaw_rate_rad_s = imu_yaw;
    return s;
}

robot_slamd::ScalarTofSnapshotFrame tof(std::uint16_t mm, double yaw, double t, const std::string &id) {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_mm = mm;
    out.distance_m = static_cast<double>(mm) / 1000.0;
    out.confidence = 100;
    out.echo_tag_u48 = 0x550000 + mm;
    out.effective_timestamp_s = t;
    out.protocol_valid = true;
    out.usable_for_mapping = true;
    out.full_fov_rad = 1.6 * kTestPi / 180.0;
    out.mount_yaw_rad = yaw;
    out.frame_id = id;
    out.source = "phase_controller_test";
    return out;
}

robot_slamd::ActiveMultiTofObservationAppendInput append_input(double t, double yaw = 0.0) {
    robot_slamd::ActiveMultiTofObservationAppendInput in;
    in.snapshot.has_multi_tof = true;
    in.snapshot.multi_tof.has_front = true;
    in.snapshot.multi_tof.has_left = true;
    in.snapshot.multi_tof.has_right = true;
    in.snapshot.multi_tof.front = tof(1000, 0.0, t, "front");
    in.snapshot.multi_tof.left = tof(1100, kTestPi / 2.0, t, "left");
    in.snapshot.multi_tof.right = tof(1200, -kTestPi / 2.0, t, "right");
    in.snapshot.multi_tof.synchronized_time_s = t;
    in.snapshot.multi_tof.valid_tof_count = 3;
    in.snapshot.multi_tof.source = "phase_controller_test";
    in.measurement_poses.front.valid = true;
    in.measurement_poses.left.valid = true;
    in.measurement_poses.right.valid = true;
    in.measurement_poses.front.measurement_timestamp_s = t;
    in.measurement_poses.left.measurement_timestamp_s = t;
    in.measurement_poses.right.measurement_timestamp_s = t;
    in.measurement_poses.front.base_pose_in_map = robot_slamd::make_map_pose({0.0, 0.0, yaw});
    in.measurement_poses.left.base_pose_in_map = robot_slamd::make_map_pose({0.0, 0.0, yaw});
    in.measurement_poses.right.base_pose_in_map = robot_slamd::make_map_pose({0.0, 0.0, yaw});
    in.measurement_poses.complete = true;
    in.measurement_poses.valid_pose_count = 3;
    in.front_odom_pose_at_measurement = robot_slamd::make_odom_pose({0.0, 0.0, yaw});
    in.left_odom_pose_at_measurement = robot_slamd::make_odom_pose({0.0, 0.0, yaw});
    in.right_odom_pose_at_measurement = robot_slamd::make_odom_pose({0.0, 0.0, yaw});
    return in;
}

robot_slamd::PhaseAwareObservationController controller() {
    robot_slamd::PhaseAwareObservationControllerConfig cfg;
    cfg.bundle.min_snapshots = 2;
    cfg.bundle.min_matchable_rays = 3;
    cfg.bundle.min_yaw_span_rad = 0.0;
    cfg.settle.min_consecutive_samples = 3;
    cfg.settle.min_stable_duration_s = 0.2;
    cfg.settle.max_sample_gap_s = 0.2;
    return robot_slamd::PhaseAwareObservationController(cfg);
}

} // namespace

int main() {
    using namespace robot_slamd;
    MotionSettleGate gate;
    auto stale = settle_sample(1.0);
    stale.wheel_fresh = false;
    expect(!gate.update(stale).ok, "wheel stale rejected");
    auto imu_stale = settle_sample(1.1);
    imu_stale.imu_fresh = false;
    expect(!gate.update(imu_stale).ok, "imu stale rejected");
    expect(gate.update(settle_sample(1.2, 0.1)).reset, "linear speed resets");
    expect(gate.update(settle_sample(1.3, 0.0, 0.1)).reset, "wheel angular resets");
    expect(gate.update(settle_sample(1.4, 0.0, 0.0, 0.1)).reset, "imu yaw resets");
    expect(gate.update(settle_sample(1.5)).ok, "stable restarts");
    expect(!gate.update(settle_sample(1.4)).ok, "timestamp rollback rejected");
    MotionSettleGateConfig cfg;
    cfg.max_sample_gap_s = 0.05;
    MotionSettleGate gap_gate(cfg);
    expect(gap_gate.update(settle_sample(2.0)).ok, "gap first");
    auto gap = gap_gate.update(settle_sample(2.2));
    expect(gap.ok && gap.reset && gap.consecutive_sample_count == 1, "gap resets but consumes sample");
    return 0;
}
