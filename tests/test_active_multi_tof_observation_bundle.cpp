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
    auto b = builder();
    expect(b.summary().state == ActiveObservationBundleState::CollectingDuringMotion, "begin collecting");
    expect(b.summary().bundle_id == 7, "bundle id");
    auto in = append_input(1.0);
    const auto a = b.append(in);
    expect(a.ok, a.message);
    expect(b.summary().accepted_snapshot_count == 1, "accepted count");
    expect(b.summary().front_ray_count == 1, "front ray");
    expect(b.summary().left_ray_count == 1, "left ray");
    expect(b.summary().right_ray_count == 1, "right ray");
    expect(b.summary().hit_ray_count == 3, "hit count");
    expect(b.summary().matchable_ray_count == 3, "matchable count");
    expect(b.frames().size() == 1, "frame retained");
    expect(b.frames()[0].front.map_pose.valid, "front pose retained");
    expect(b.frames()[0].left.map_pose.valid, "left pose retained");
    expect(b.frames()[0].right.map_pose.valid, "right pose retained");
    expect(b.frames()[0].front.frame.echo_tag_u48 == in.snapshot.multi_tof.front.echo_tag_u48, "echo tag preserved");
    expect(b.frames()[0].front.effective_timestamp_s == in.snapshot.multi_tof.front.effective_timestamp_s, "timestamp preserved");

    auto no_return = append_input(1.1, 0.0, "explicit_no_return_test");
    const auto nr = b.append(no_return);
    expect(nr.ok, nr.message);
    expect(b.summary().no_return_ray_count == 3, "no return count");
    expect(b.summary().matchable_ray_count == 6, "no return matchable");

    auto invalid = append_input(1.2);
    invalid.snapshot.multi_tof.front.protocol_valid = false;
    invalid.snapshot.multi_tof.left.protocol_valid = false;
    invalid.snapshot.multi_tof.right.protocol_valid = false;
    const auto before = b.summary().accepted_snapshot_count;
    const auto rej = b.append(invalid);
    expect(!rej.ok, "invalid-only rejected");
    expect(rej.fault == ActiveObservationBundleFault::NoMatchableRay, "zero matchable fault");
    expect(b.summary().accepted_snapshot_count == before, "append failure transactional");
    expect(b.summary().rejected_snapshot_count == 1, "reject count");

    auto missing_pose = append_input(1.3);
    missing_pose.measurement_poses.complete = false;
    const auto mp = b.append(missing_pose);
    expect(!mp.ok && mp.fault == ActiveObservationBundleFault::MissingMeasurementPose, "missing pose rejected");

    auto nonfinite_pose = append_input(1.4);
    nonfinite_pose.measurement_poses.front.base_pose_in_map.map_T_base.x_m = NAN;
    const auto nf = b.append(nonfinite_pose);
    expect(!nf.ok && nf.fault == ActiveObservationBundleFault::NonFinitePose, "nonfinite pose rejected");
    return 0;
}
