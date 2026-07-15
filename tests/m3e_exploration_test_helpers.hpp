#pragma once

#include "robot_slamd/simulation/exploration/m3e_exploration_runner.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace m3e_test {

inline void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << "\n";
        std::exit(1);
    }
}

inline robot_slamd::Config config() {
    robot_slamd::Config out;
    out.slam_runtime_mode = "sparse_sim_exploration";
    out.tof_protocol_max_range_m = 2.0;
    out.tof_mapping_max_range_m = 2.0;
    out.sparse_slam_planar_tof_extrinsics_configured = true;
    out.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    out.sparse_slam_front_tof_x_m = 0.04;
    out.sparse_slam_left_tof_y_m = 0.04;
    out.sparse_slam_left_tof_yaw_rad = 1.5707963267948966;
    out.sparse_slam_right_tof_y_m = -0.04;
    out.sparse_slam_right_tof_yaw_rad = -1.5707963267948966;
    out.sparse_slam_map_id = "m3e_test_room";
    out.sparse_slam_map_run_id = "deterministic_test";
    out.sparse_slam_active_bundle_max_snapshots = 512;
    out.sparse_slam_active_bundle_max_rays = 1536;
    out.sparse_slam_active_bundle_max_duration_s = 30.0;
    out.sparse_slam_active_bundle_max_snapshot_gap_s = 0.20;
    out.sparse_slam_active_bundle_min_snapshots = 3;
    out.sparse_slam_active_bundle_min_matchable_rays = 6;
    out.sparse_slam_active_bundle_min_yaw_span_rad = 0.05;
    out.sparse_slam_settle_max_abs_linear_speed_mps = 0.03;
    out.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s = 0.03;
    out.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s = 0.03;
    out.sparse_slam_settle_min_stable_duration_s = 0.10;
    out.sparse_slam_local_match_min_bundle_frames = 3;
    out.sparse_slam_local_match_min_matchable_rays = 6;
    out.sparse_slam_local_match_min_used_rays = 6;
    out.sparse_slam_local_match_min_known_cells = 4;
    out.sparse_slam_local_match_max_unknown_ratio = 0.95;
    out.sparse_slam_local_match_max_contradiction_ratio = 0.90;
    out.sparse_slam_local_match_min_normalized_score = -0.50;
    out.sparse_slam_local_match_min_score_margin = 0.0;
    out.sparse_slam_local_match_min_score_range = 0.0;
    out.sparse_slam_local_match_multimodal_max_score_drop = 0.0;
    out.exploration_bootstrap_minimum_known_cells = 60;
    out.exploration_bootstrap_minimum_yaw_rad = 5.5;
    out.exploration_bootstrap_max_duration_s = 22.0;
    out.exploration_max_duration_s = 240.0;
    out.exploration_maximum_planning_failures = 30;
    out.exploration_minimum_goal_distance_m = 0.30;
    out.exploration_failure_cooldown_cycles = 3;
    out.exploration_failure_retry_revision_delta = 30;
    out.exploration_obstacle_stop_distance_m = 0.20;
    out.exploration_emergency_stop_distance_m = 0.09;
    out.exploration_completion_minimum_known_ratio = 0.70;
    out.exploration_simulation_tof_max_range_m = 2.0;
    return out;
}

inline robot_slamd::M3EExplorationRunReport run(const std::string &path) {
    expect(robot_slamd::mkdir_p(path), "create deterministic run directory");
    return robot_slamd::M3EExplorationRunner{}.run(config(), path);
}

} // namespace m3e_test
