#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"

#include <cmath>
#include <iostream>

namespace {
void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

robot_slamd::ScalarTofSnapshotFrame frame(double t, double yaw) {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_m = 1.0;
    out.distance_mm = 1000;
    out.confidence = 90;
    out.echo_tag_u48 = 7;
    out.effective_timestamp_s = t;
    out.protocol_valid = true;
    out.usable_for_mapping = true;
    out.full_fov_rad = 0.01;
    out.mount_yaw_rad = yaw;
    return out;
}

robot_slamd::TimedMapBasePose2D timed(double t, double x) {
    robot_slamd::TimedMapBasePose2D out;
    out.valid = true;
    out.measurement_timestamp_s = t;
    robot_slamd::RobotPose2D pose;
    pose.x_m = x;
    out.base_pose_in_map = robot_slamd::make_map_pose(pose);
    out.lookup_status = robot_slamd::TimedPoseLookupStatus::Exact;
    return out;
}

robot_slamd::SlamBackendInputFrame input(bool measurement_poses) {
    robot_slamd::SlamBackendInputFrame in;
    in.timestamp_s = 10.1;
    in.has_predicted_pose = true;
    in.snapshot.has_multi_tof = true;
    in.snapshot.has_wheel = true;
    in.snapshot.has_imu = true;
    in.snapshot.multi_tof.has_front = true;
    in.snapshot.multi_tof.has_left = true;
    in.snapshot.multi_tof.has_right = true;
    in.snapshot.multi_tof.front = frame(9.9, 0.0);
    in.snapshot.multi_tof.left = frame(10.0, 0.0);
    in.snapshot.multi_tof.right = frame(10.1, 0.0);
    if (measurement_poses) {
        in.has_multi_tof_measurement_poses = true;
        in.multi_tof_measurement_poses.front = timed(9.9, 0.0);
        in.multi_tof_measurement_poses.left = timed(10.0, 5.0);
        in.multi_tof_measurement_poses.right = timed(10.1, 10.0);
        in.multi_tof_measurement_poses.complete = true;
        in.multi_tof_measurement_poses.valid_pose_count = 3;
    }
    return in;
}
} // namespace

int main() {
    robot_slamd::SparseMultiTofOccupancyBackendOptions strict;
    strict.minimum_accepted_snapshots_for_good_quality = 1;
    strict.minimum_valid_rays_for_good_quality = 1;
    strict.minimum_observed_cells_for_good_quality = 1;
    strict.minimum_angular_bins_for_good_quality = 1;
    strict.require_multi_tof_measurement_poses = true;
    strict.allow_single_pose_fallback = false;
    robot_slamd::SparseMultiTofOccupancyBackendBinding backend(strict);
    auto missing = backend.update(input(false));
    expect(missing.status == robot_slamd::SlamBackendUpdateStatus::Rejected,
           "strict missing measurement poses rejected");

    auto accepted = backend.update(input(true));
    expect(accepted.status == robot_slamd::SlamBackendUpdateStatus::Accepted,
           "measurement poses accepted");
    const auto report = backend.report();
    expect(report.measurement_time_pose_consumed,
           "measurement-time pose consumed");
    expect(!report.single_pose_fallback_consumed,
           "single pose fallback not consumed");
    expect(report.front_ray_count == 1 && report.left_ray_count == 1 &&
               report.right_ray_count == 1,
           "all three rays built");
    const auto snapshot = backend.grid_snapshot();
    bool saw_far_x = false;
    for (const auto &cell : snapshot.cells()) {
        if (cell.key.x >= 90) {
            saw_far_x = true;
        }
    }
    expect(saw_far_x, "right measurement pose affected map cells");

    auto mismatch = input(true);
    mismatch.multi_tof_measurement_poses.left.measurement_timestamp_s = 9.0;
    robot_slamd::SparseMultiTofOccupancyBackendBinding mismatch_backend(strict);
    auto rejected = mismatch_backend.update(mismatch);
    expect(rejected.status == robot_slamd::SlamBackendUpdateStatus::Rejected,
           "timestamp mismatch rejected");

    robot_slamd::SparseMultiTofOccupancyBackendOptions fallback;
    fallback.minimum_accepted_snapshots_for_good_quality = 1;
    fallback.minimum_valid_rays_for_good_quality = 1;
    fallback.minimum_observed_cells_for_good_quality = 1;
    fallback.minimum_angular_bins_for_good_quality = 1;
    fallback.require_multi_tof_measurement_poses = false;
    fallback.allow_single_pose_fallback = true;
    robot_slamd::SparseMultiTofOccupancyBackendBinding compat(fallback);
    auto fallback_result = compat.update(input(false));
    expect(fallback_result.status == robot_slamd::SlamBackendUpdateStatus::Accepted,
           "compat single pose fallback accepted");
    expect(compat.report().single_pose_fallback_consumed,
           "compat fallback reported");
    return 0;
}
