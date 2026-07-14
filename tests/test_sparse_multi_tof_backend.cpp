#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::ScalarTofSnapshotFrame frame(std::uint16_t mm,
                                          double yaw,
                                          const char *id = "tof") {
    robot_slamd::ScalarTofSnapshotFrame f;
    f.distance_mm = mm;
    f.distance_m = static_cast<double>(mm) / 1000.0;
    f.confidence = 100;
    f.echo_tag_u48 = 0x1200 + mm;
    f.effective_timestamp_s = 10.0;
    f.protocol_valid = true;
    f.usable_for_mapping = true;
    f.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
    f.mount_yaw_rad = yaw;
    f.frame_id = id;
    f.source = "unit_test";
    return f;
}

robot_slamd::RobotSlamSensorSnapshot snapshot(bool front,
                                              bool left,
                                              bool right) {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_multi_tof = true;
    s.has_tof = false;
    s.multi_tof.has_front = front;
    s.multi_tof.has_left = left;
    s.multi_tof.has_right = right;
    s.multi_tof.valid_tof_count = (front ? 1 : 0) + (left ? 1 : 0) +
                                  (right ? 1 : 0);
    s.multi_tof.front = frame(1000, 0.0, "front");
    s.multi_tof.left = frame(1000, 3.14159265358979323846 / 2.0, "left");
    s.multi_tof.right = frame(1000, -3.14159265358979323846 / 2.0, "right");
    s.multi_tof.synchronized_time_s = 10.0;
    return s;
}

robot_slamd::SlamBackendInputFrame input(
    const robot_slamd::RobotSlamSensorSnapshot &s) {
    robot_slamd::SlamBackendInputFrame in;
    in.timestamp_s = 10.0;
    in.snapshot = s;
    in.has_predicted_pose = true;
    in.predicted_pose = robot_slamd::RobotPose2D{};
    in.source = "unit_test_predicted_pose";
    return in;
}
}

int main() {
    using namespace robot_slamd;
    SparseMultiTofOccupancyBackendOptions options;
    options.grid.resolution_m = 0.5;
    options.minimum_accepted_snapshots_for_good_quality = 1;
    options.minimum_valid_rays_for_good_quality = 1;
    options.minimum_observed_cells_for_good_quality = 1;
    options.minimum_angular_bins_for_good_quality = 1;
    options.use_planar_tof_extrinsics = true;
    options.planar_tof_extrinsics.front = {0.5, 0.5, 0.0};
    options.planar_tof_extrinsics.left = {-0.5, 0.5, 1.5707963267948966};
    options.planar_tof_extrinsics.right = {-0.5, -0.5, -1.5707963267948966};

    SparseMultiTofOccupancyBackendBinding backend(options);
    auto result = backend.update(input(snapshot(true, false, false)));
    expect(result.status == SlamBackendUpdateStatus::Accepted,
           "single front hit accepted");
    expect(result.map_updated, "single front hit updates map");
    expect(backend.report().native_multi_tof_backend_consumption,
           "native multi-ToF consumed");
    expect(!backend.report().legacy_projection_consumed,
           "legacy projection not consumed");
    expect(backend.report().front_ray_count == 1, "front ray counted");
    expect(backend.report().hit_ray_count == 1, "hit ray counted");
    auto map = backend.grid_snapshot();
    expect(map.occupied_cell_count() > 0, "hit endpoint occupied");
    expect(map.query_cell({3, 1}).evidence > 0,
           "backend applies translated front sensor origin once");
    expect(backend.report().free_cell_update_count > 0, "hit path free updates");

    SparseMultiTofOccupancyBackendBinding three(options);
    auto three_result = three.update(input(snapshot(true, true, true)));
    expect(three_result.status == SlamBackendUpdateStatus::Accepted,
           "three direction hit accepted");
    expect(three.report().front_ray_count == 1 &&
               three.report().left_ray_count == 1 &&
               three.report().right_ray_count == 1,
           "front left right rays counted");
    expect(three.report().hit_ray_count == 3, "three hit rays");

    auto no_return_snapshot = snapshot(true, false, false);
    no_return_snapshot.multi_tof.front.distance_mm = 12000;
    no_return_snapshot.multi_tof.front.distance_m = 12.0;
    no_return_snapshot.multi_tof.front.source =
        "sim_center_raycast_explicit_no_return";
    SparseMultiTofOccupancyBackendBinding nr_backend(options);
    auto nr = nr_backend.update(input(no_return_snapshot));
    expect(nr.status == SlamBackendUpdateStatus::Accepted,
           "explicit no return accepted");
    expect(nr_backend.report().no_return_ray_count == 1,
           "no return counted");
    expect(nr_backend.grid_snapshot().occupied_cell_count() == 0,
           "no return has no occupied endpoint");

    auto missing_right = snapshot(true, true, false);
    missing_right.multi_tof.degraded = true;
    missing_right.multi_tof.degraded_reason = "right_missing_allowed";
    SparseMultiTofOccupancyBackendBinding degraded(options);
    auto degraded_result = degraded.update(input(missing_right));
    expect(degraded_result.status == SlamBackendUpdateStatus::Accepted,
           "allow missing right accepted natively");
    expect(degraded.report().front_ray_count == 1 &&
               degraded.report().left_ray_count == 1 &&
               degraded.report().right_ray_count == 0,
           "missing right does not require legacy projection");

    expect(backend.latest_quality(10.0).good_enough,
           "map quality formula can become ready");
    return failures ? 1 : 0;
}
