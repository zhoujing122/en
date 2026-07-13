#include "robot_slamd/runtime/multi_tof_measurement_pose_resolver.hpp"

#include <cmath>
#include <iostream>

namespace {
void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}
void near(double a, double b, const char *message) {
    expect(std::fabs(a - b) < 1e-9, message);
}
robot_slamd::OdomPose2D odom(double x) {
    robot_slamd::RobotPose2D raw;
    raw.x_m = x;
    return robot_slamd::make_odom_pose(raw);
}
} // namespace

int main() {
    using namespace robot_slamd;
    TimedOdomPoseBufferConfig buffer_cfg;
    buffer_cfg.max_interpolation_gap_s = 2.0;
    TimedOdomPoseBuffer buffer(buffer_cfg);
    expect(buffer.append(1.0, odom(0.0)).ok, "append 1");
    expect(buffer.append(2.0, odom(2.0)).ok, "append 2");
    MapOdomFrameState frames;
    RobotPose2D configured;
    configured.x_m = 10.0;
    expect(frames.initialize_configured_pose(make_map_pose(configured)).ok,
           "configured frame");

    MultiTofSnapshot snapshot;
    snapshot.has_front = true;
    snapshot.has_left = true;
    snapshot.has_right = true;
    snapshot.front.effective_timestamp_s = 1.0;
    snapshot.left.effective_timestamp_s = 1.5;
    snapshot.right.effective_timestamp_s = 2.0;
    MultiTofMeasurementPoseResolver resolver;
    const auto result = resolver.resolve(snapshot, buffer, frames);
    expect(result.ok && result.poses.complete, "pose set complete");
    near(result.poses.front.base_pose_in_map.map_T_base.x_m, 10.0,
         "front exact map pose");
    near(result.poses.left.base_pose_in_map.map_T_base.x_m, 11.0,
         "left interpolated map pose");
    near(result.poses.right.base_pose_in_map.map_T_base.x_m, 12.0,
         "right exact map pose");
    expect(result.poses.front.measurement_timestamp_s == 1.0,
           "front uses front timestamp");
    expect(result.poses.left.measurement_timestamp_s == 1.5,
           "left uses left timestamp");
    expect(result.poses.right.measurement_timestamp_s == 2.0,
           "right uses right timestamp");

    snapshot.right.effective_timestamp_s = 3.0;
    const auto rejected = resolver.resolve(snapshot, buffer, frames);
    expect(!rejected.ok, "require all rejects missing right pose");

    MultiTofMeasurementPoseResolverConfig cfg;
    cfg.require_all_three_measurement_poses = false;
    MultiTofMeasurementPoseResolver degraded(cfg);
    const auto partial = degraded.resolve(snapshot, buffer, frames);
    expect(partial.ok && partial.poses.valid_pose_count == 2,
           "degraded policy explicit");
    return 0;
}
