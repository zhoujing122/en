#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_scan_matcher.hpp"

#include <cmath>

namespace d2test {

inline robot_slamd::SparseTofRayObservation ray(
    robot_slamd::ScalarTofReturnKind kind,
    const robot_slamd::RobotPose2D &pose,
    double mount_yaw,
    double range) {
    robot_slamd::SparseTofRayObservation r;
    r.return_kind = kind;
    r.sensor_origin_x_m = pose.x_m;
    r.sensor_origin_y_m = pose.y_m;
    r.ray_yaw_rad = robot_slamd::se2_normalize_yaw(pose.yaw_rad + mount_yaw);
    r.measured_range_m = range;
    r.free_space_range_m = range;
    r.confidence = 100;
    r.echo_tag_u48 = 42;
    r.effective_timestamp_s = 10.0;
    r.protocol_valid = true;
    r.synchronized = true;
    r.usable_for_mapping = kind != robot_slamd::ScalarTofReturnKind::Invalid;
    r.frame_id = "d2_test_tof";
    return r;
}

inline robot_slamd::SparseTofMatchObservation observation(
    double t,
    const robot_slamd::RobotPose2D &odom_pose,
    double front = 1.0,
    double left = 0.75,
    double right = 0.55) {
    robot_slamd::SparseTofMatchObservation obs;
    obs.timestamp_s = t;
    obs.odom_pose = odom_pose;
    obs.rays[0] = ray(robot_slamd::ScalarTofReturnKind::Hit, odom_pose, 0.0, front);
    obs.rays[1] = ray(robot_slamd::ScalarTofReturnKind::Hit, odom_pose, 1.5707963267948966, left);
    obs.rays[2] = ray(robot_slamd::ScalarTofReturnKind::Hit, odom_pose, -1.5707963267948966, right);
    obs.ray_count = 3;
    return obs;
}

inline robot_slamd::SparseTofObservationBundle bundle_at(
    const robot_slamd::RobotPose2D &odom_pose) {
    robot_slamd::SparseTofObservationBundle bundle;
    bundle.add(observation(10.0, odom_pose));
    robot_slamd::RobotPose2D moved = odom_pose;
    moved.x_m += 0.05;
    bundle.add(observation(10.1, moved, 0.95, 0.80, 0.50));
    return bundle;
}

inline robot_slamd::SparseOccupancyGridConfig grid_config() {
    robot_slamd::SparseOccupancyGridConfig cfg;
    cfg.resolution_m = 0.10;
    cfg.maximum_ray_cells = 256;
    cfg.maximum_active_cells = 10000;
    return cfg;
}

inline robot_slamd::SparseOccupancyGridSnapshot reference_map() {
    auto cfg = grid_config();
    robot_slamd::SparseOccupancyGrid grid(cfg);
    robot_slamd::RobotPose2D origin;
    auto bundle = bundle_at(origin);
    for (const auto &obs : bundle.observations) {
        std::vector<robot_slamd::SparseTofRayObservation> rays;
        for (std::size_t i = 0; i < obs.ray_count; ++i) {
            rays.push_back(obs.rays[i]);
        }
        grid.apply_observations(rays);
    }
    return grid.snapshot();
}

inline robot_slamd::SparseTofLocalScanMatcherConfig matcher_config() {
    robot_slamd::SparseTofLocalScanMatcherConfig cfg;
    cfg.grid = grid_config();
    cfg.search.coarse_translation_window_m = 0.12;
    cfg.search.coarse_translation_step_m = 0.04;
    cfg.search.coarse_yaw_window_rad = 0.16;
    cfg.search.coarse_yaw_step_rad = 0.04;
    cfg.search.fine_translation_window_m = 0.04;
    cfg.search.fine_translation_step_m = 0.02;
    cfg.search.fine_yaw_window_rad = 0.04;
    cfg.search.fine_yaw_step_rad = 0.01;
    cfg.search.maximum_candidate_count = 20000;
    cfg.score.minimum_valid_ray_count = 3;
    cfg.score.minimum_hit_ray_count = 1;
    cfg.score.minimum_known_cell_count = 2;
    cfg.score.minimum_known_cell_ratio = 0.05;
    cfg.score.minimum_normalized_score = 0.0;
    cfg.score.minimum_score_margin = 0.0;
    cfg.score.minimum_observability_contrast = 0.0;
    return cfg;
}

} // namespace d2test
