#include "m3e_exploration_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    const auto formal_config = load_config(
        std::string(ROBOT_SLAMD_SOURCE_DIR) +
        "/config/sparse_sim_lost_recovery.yaml", "");
    m3e_test::expect(formal_config.sparse_slam_map_startup_mode ==
                         "load_existing" &&
                         formal_config.sparse_slam_initial_pose_mode ==
                         "configured_pose" &&
                         formal_config.sparse_slam_has_configured_pose,
                     "formal lost recovery config is parseable");
    const std::string reference_dir =
        "/tmp/en_m3d3b_recovery_reference";
    const std::string recovery_dir =
        "/tmp/en_m3d3b_recovery_run";
    const std::string recovery_repeat_dir =
        "/tmp/en_m3d3b_recovery_run_repeat";
    m3e_test::expect(mkdir_p(reference_dir), "create recovery reference dir");
    m3e_test::expect(mkdir_p(recovery_dir), "create recovery run dir");
    m3e_test::expect(mkdir_p(recovery_repeat_dir),
                     "create repeat recovery run dir");

    const auto reference =
        M3EExplorationRunner{}.run(m3e_test::config(), reference_dir);
    m3e_test::expect(reference.ok && reference.final_map_save_success,
                     "recovery reference map generated");

    auto config = m3e_test::config();
    config.sparse_slam_map_startup_mode = "load_existing";
    config.sparse_slam_initial_pose_mode = "configured_pose";
    config.sparse_slam_has_configured_pose = true;
    config.sparse_slam_map_path = reference.final_map_path;
    config.exploration_simulation_initial_x_m = -0.45;
    config.exploration_simulation_initial_y_m = -0.35;
    config.exploration_simulation_initial_yaw_rad = 0.55;
    config.sparse_slam_configured_pose_x_m = 0.40;
    config.sparse_slam_configured_pose_y_m = 0.45;
    config.sparse_slam_configured_pose_yaw_rad = 0.55;
    config.sparse_slam_local_match_min_normalized_score = 0.80;
    config.sparse_slam_localization_health_max_consistency_failures = 2;
    config.sparse_slam_localization_health_min_lost_duration_s = 0.0;
    config.sparse_slam_localization_health_min_lost_odom_distance_m = 0.0;
    config.sparse_slam_active_bundle_max_snapshots = 1024;
    config.sparse_slam_active_bundle_max_rays = 3072;
    config.sparse_slam_active_bundle_max_duration_s = 60.0;
    config.sparse_slam_relocalization_coarse_max_scored_rays = 96;
    config.sparse_slam_relocalization_refine_max_scored_rays = 288;
    config.sparse_slam_relocalization_max_cells_per_ray = 512;
    config.sparse_slam_relocalization_coarse_free_cell_stride = 8;
    config.sparse_slam_relocalization_coarse_yaw_step_rad = kPi / 6.0;
    config.sparse_slam_relocalization_refine_xy_step_m = 0.05;
    config.sparse_slam_relocalization_refine_yaw_step_rad = kPi / 36.0;
    config.sparse_slam_relocalization_final_xy_step_m = 0.025;
    config.sparse_slam_relocalization_final_yaw_step_rad = kPi / 180.0;
    config.sparse_slam_relocalization_top_k = 4;
    config.sparse_slam_relocalization_max_total_candidates = 15000;
    config.sparse_slam_relocalization_min_normalized_score = -1.0;
    config.sparse_slam_relocalization_min_score_margin = 0.0;
    config.sparse_slam_relocalization_min_score_range = 0.0;
    config.sparse_slam_relocalization_multimodal_max_score_drop = 0.0;
    config.sparse_slam_relocalization_exclusion_xy_m = 0.20;
    config.sparse_slam_relocalization_exclusion_yaw_rad = 0.20;
    config.sparse_slam_relocalization_confirmation_xy_tolerance_m = 0.35;
    config.sparse_slam_relocalization_confirmation_yaw_tolerance_rad = 0.35;
    config.exploration_max_duration_s = 150.0;
    validate_config(config);

    const auto run = M3EExplorationRunner{}.run(config, recovery_dir);
    if (run.recovery_success_count == 0) {
        std::cerr << "recovery attempts=" << run.recovery_attempt_count
                  << " successes=" << run.recovery_success_count
                  << " relocalization_state=" << run.relocalization_state
                  << " reason=" << run.relocalization_reason
                  << " termination=" << run.termination_reason
                  << " stops=" << run.localization_stop_count << "\n";
    }
    m3e_test::expect(run.recovery_attempt_count >= 1,
                     "persistent map inconsistency triggers Lost");
    m3e_test::expect(run.recovery_success_count >= 1 &&
                         run.relocalization_success_count >= 1,
                     "local then global recovery is independently confirmed");
    m3e_test::expect(run.localization_stop_count >= 1 &&
                         run.stop_command_count >= 1,
                     "Lost immediately sends motion Stop");
    m3e_test::expect(run.exploration_resumed &&
                         run.exploration.replan_count >= 1,
                     "successful recovery clears path and replans");
    m3e_test::expect(!run.collision,
                     "recovery run remains collision free");
    m3e_test::expect(run.relocalization_search_count >= 3 &&
                         run.relocalization_first_bundle_id != 0,
                     "recovery rejects the wrong local mode and confirms a global mode");
    const auto repeat =
        M3EExplorationRunner{}.run(config, recovery_repeat_dir);
    m3e_test::expect(repeat.recovery_success_count ==
                         run.recovery_success_count &&
                         repeat.relocalization_search_count ==
                             run.relocalization_search_count &&
                         repeat.relocalization_best_x_m ==
                             run.relocalization_best_x_m &&
                         repeat.relocalization_best_y_m ==
                             run.relocalization_best_y_m &&
                         repeat.relocalization_best_yaw_rad ==
                             run.relocalization_best_yaw_rad &&
                         repeat.termination_reason == run.termination_reason,
                     "lost recovery is deterministic");
    return 0;
}
