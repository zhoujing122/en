#include "robot_slamd/runtime/active_multi_tof_observation_bundle.hpp"
#include "robot_slamd/config/config.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void expect(bool cond, const std::string &msg) {
    if (!cond) {
        throw std::runtime_error(msg);
    }
}

constexpr double kTestPi = 3.14159265358979323846;

robot_slamd::ScalarTofSnapshotFrame tof(std::uint16_t mm,
                                        double yaw,
                                        double t,
                                        const std::string &id,
                                        const std::string &source = "test") {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_mm = mm;
    out.distance_m = static_cast<double>(mm) / 1000.0;
    out.confidence = 100;
    out.echo_tag_u48 = 0x440000 + mm;
    out.effective_timestamp_s = t;
    out.protocol_valid = true;
    out.usable_for_mapping = true;
    out.full_fov_rad = 1.6 * kTestPi / 180.0;
    out.mount_yaw_rad = yaw;
    out.frame_id = id;
    out.source = source;
    return out;
}

robot_slamd::RobotSlamSensorSnapshot snapshot(double t, const std::string &source = "test") {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_multi_tof = true;
    s.multi_tof.has_front = true;
    s.multi_tof.has_left = true;
    s.multi_tof.has_right = true;
    s.multi_tof.front = tof(1000, 0.0, t, "front", source);
    s.multi_tof.left = tof(1100, kTestPi / 2.0, t, "left", source);
    s.multi_tof.right = tof(1200, -kTestPi / 2.0, t, "right", source);
    s.multi_tof.synchronized_time_s = t;
    s.multi_tof.valid_tof_count = 3;
    s.multi_tof.source = source;
    return s;
}

robot_slamd::TimedMapBasePose2D map_pose(double t, double yaw) {
    robot_slamd::TimedMapBasePose2D out;
    out.valid = true;
    out.measurement_timestamp_s = t;
    out.base_pose_in_map = robot_slamd::make_map_pose({0.1 * t, 0.0, yaw});
    return out;
}

robot_slamd::OdomPose2D odom(double yaw) {
    return robot_slamd::make_odom_pose({0.0, 0.0, yaw});
}

robot_slamd::ActiveMultiTofObservationAppendInput append_input(double t,
                                                               double yaw = 0.0,
                                                               const std::string &source = "test") {
    robot_slamd::ActiveMultiTofObservationAppendInput in;
    in.snapshot = snapshot(t, source);
    in.measurement_poses.front = map_pose(t, yaw);
    in.measurement_poses.left = map_pose(t, yaw);
    in.measurement_poses.right = map_pose(t, yaw);
    in.measurement_poses.complete = true;
    in.measurement_poses.valid_pose_count = 3;
    in.front_odom_pose_at_measurement = odom(yaw);
    in.left_odom_pose_at_measurement = odom(yaw);
    in.right_odom_pose_at_measurement = odom(yaw);
    in.sequence_index = static_cast<std::size_t>(t * 100.0);
    return in;
}

robot_slamd::ActiveMultiTofObservationBundleBuilder builder(robot_slamd::ActiveMultiTofObservationBundleConfig cfg = {}) {
    robot_slamd::ActiveMultiTofObservationBundleBuilder b(cfg);
    const auto begin = b.begin(7, 3, 42, robot_slamd::identity_map_from_odom());
    expect(begin.ok, begin.message);
    return b;
}

} // namespace

int main() {
    using namespace robot_slamd;
    expect(to_string(ActiveObservationBundleState::Idle) == "idle", "idle string");
    expect(to_string(ActiveObservationBundleState::CollectingDuringMotion) == "collecting_during_motion", "collecting string");
    expect(to_string(ActiveObservationBundleState::WaitingForMotionSettle) == "waiting_for_motion_settle", "waiting string");
    expect(to_string(ActiveObservationBundleState::FrozenReady) == "frozen_ready", "frozen string");
    expect(to_string(ActiveObservationBundleState::Aborted) == "aborted", "aborted string");
    expect(to_string(ActiveObservationBundleFault::NoMatchableRay) == "no_matchable_ray", "fault string");
    ActiveMultiTofObservationBundleConfig cfg;
    expect(active_bundle_config_valid(cfg), "default config valid");
    Config c;
    validate_config(c);
    c.sparse_slam_active_bundle_max_snapshots = 0;
    bool threw = false;
    try { validate_config(c); } catch (...) { threw = true; }
    expect(threw, "invalid max snapshots rejected");
    Config resolved;
    write_resolved_config(resolved, "/tmp/en_m3d2a1_active_bundle_resolved.yaml");
    return 0;
}
