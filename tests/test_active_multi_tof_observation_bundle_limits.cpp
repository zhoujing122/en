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
    ActiveMultiTofObservationBundleConfig cfg;
    cfg.max_snapshots = 2;
    cfg.max_rays = 6;
    cfg.max_duration_s = 0.25;
    cfg.max_snapshot_gap_s = 0.20;
    cfg.min_snapshots = 2;
    cfg.min_matchable_rays = 3;
    auto b = builder(cfg);
    expect(b.append(append_input(1.0)).ok, "first append");
    const auto dup = b.append(append_input(1.0));
    expect(!dup.ok && dup.fault == ActiveObservationBundleFault::NonMonotonicTimestamp, "duplicate rejected");
    expect(b.summary().state == ActiveObservationBundleState::Aborted, "duplicate aborts");

    b = builder(cfg);
    expect(b.append(append_input(1.0)).ok, "first append gap");
    const auto gap = b.append(append_input(1.25));
    expect(!gap.ok && gap.fault == ActiveObservationBundleFault::SnapshotGapExceeded, "gap rejected");
    expect(b.summary().state == ActiveObservationBundleState::Aborted, "gap aborts");

    b = builder(cfg);
    expect(b.append(append_input(1.0)).ok, "first duration");
    expect(b.append(append_input(1.1)).ok, "second duration");
    const auto cap = b.append(append_input(1.2));
    expect(!cap.ok && cap.fault == ActiveObservationBundleFault::CapacityExceeded, "capacity rejected");

    b = builder(cfg);
    expect(b.append(append_input(1.0)).ok, "first freeze");
    const auto early = b.freeze(3);
    expect(!early.ok && early.fault == ActiveObservationBundleFault::InvalidState, "freeze requires waiting");
    expect(b.mark_waiting_for_motion_settle().ok, "mark waiting");
    expect(b.append(append_input(1.1)).ok, "second freeze");
    const auto fr = b.freeze(3);
    expect(fr.ok, fr.message);
    expect(b.summary().state == ActiveObservationBundleState::FrozenReady, "frozen state");
    expect(b.frozen_bundle().available(), "frozen available");
    expect(b.frozen_bundle().frames().size() == 2, "frozen frame count fixed");
    const auto after = b.append(append_input(1.2));
    expect(!after.ok && after.fault == ActiveObservationBundleFault::FrozenImmutable, "frozen append rejected");
    expect(b.discard(99).fault == ActiveObservationBundleFault::BundleIdMismatch, "wrong id rejected");
    expect(b.discard(7).ok, "discard ok");
    expect(b.summary().state == ActiveObservationBundleState::Idle, "discard idle");
    return 0;
}
