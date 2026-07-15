#include "m3e_exploration_test_helpers.hpp"

#include <cmath>

int main() {
    using namespace robot_slamd;
    const auto formal_config = load_config(
        std::string(ROBOT_SLAMD_SOURCE_DIR) +
        "/config/sparse_sim_startup_relocalization.yaml", "");
    m3e_test::expect(formal_config.sparse_slam_map_startup_mode ==
                         "load_existing" &&
                         formal_config.sparse_slam_initial_pose_mode ==
                         "relocalization" &&
                         !formal_config.sparse_slam_has_configured_pose,
                     "formal startup relocalization config is parseable");
    const std::string reference_dir =
        "/tmp/en_m3d3b_startup_reference";
    const std::string relocalized_dir =
        "/tmp/en_m3d3b_startup_relocalized";
    const std::string relocalized_repeat_dir =
        "/tmp/en_m3d3b_startup_relocalized_repeat";
    m3e_test::expect(mkdir_p(reference_dir), "create reference run directory");
    m3e_test::expect(mkdir_p(relocalized_dir),
                     "create relocalization run directory");
    m3e_test::expect(mkdir_p(relocalized_repeat_dir),
                     "create repeat relocalization run directory");

    const auto reference =
        M3EExplorationRunner{}.run(m3e_test::config(), reference_dir);
    m3e_test::expect(reference.ok && reference.final_map_save_success,
                     "deterministic reference map generated");

    auto config = m3e_test::config();
    config.sparse_slam_map_startup_mode = "load_existing";
    config.sparse_slam_initial_pose_mode = "relocalization";
    config.sparse_slam_has_configured_pose = false;
    config.sparse_slam_map_path = reference.final_map_path;
    config.exploration_simulation_initial_x_m = -0.45;
    config.exploration_simulation_initial_y_m = -0.35;
    config.exploration_simulation_initial_yaw_rad = 0.55;
    config.sparse_slam_active_bundle_max_snapshots = 1024;
    config.sparse_slam_active_bundle_max_rays = 3072;
    config.sparse_slam_active_bundle_max_duration_s = 60.0;
    config.sparse_slam_relocalization_coarse_max_scored_rays = 96;
    config.sparse_slam_relocalization_refine_max_scored_rays = 288;
    config.exploration_max_duration_s = 90.0;
    config.sparse_slam_relocalization_coarse_free_cell_stride = 8;
    config.sparse_slam_relocalization_coarse_yaw_step_rad = kPi / 6.0;
    config.sparse_slam_relocalization_refine_xy_step_m = 0.05;
    config.sparse_slam_relocalization_refine_yaw_step_rad = kPi / 36.0;
    config.sparse_slam_relocalization_final_xy_step_m = 0.025;
    config.sparse_slam_relocalization_final_yaw_step_rad = kPi / 180.0;
    config.sparse_slam_relocalization_top_k = 4;
    config.sparse_slam_relocalization_max_total_candidates = 15000;
    config.sparse_slam_relocalization_max_cells_per_ray = 512;
    config.sparse_slam_relocalization_min_normalized_score = -1.0;
    config.sparse_slam_relocalization_min_score_margin = 0.0;
    config.sparse_slam_relocalization_min_score_range = 0.0;
    config.sparse_slam_relocalization_multimodal_max_score_drop = 0.0;
    config.sparse_slam_relocalization_exclusion_xy_m = 0.20;
    config.sparse_slam_relocalization_exclusion_yaw_rad = 0.20;
    config.sparse_slam_relocalization_confirmation_xy_tolerance_m = 0.30;
    config.sparse_slam_relocalization_confirmation_yaw_tolerance_rad = 0.30;
    validate_config(config);

    const auto run =
        M3EExplorationRunner{}.run(config, relocalized_dir);
    if (run.relocalization_success_count != 1) {
        std::cerr << "state=" << run.relocalization_state
                  << " reason=" << run.relocalization_reason
                  << " searches=" << run.relocalization_search_count
                  << " termination=" << run.termination_reason
                  << " position_error=" << run.final_position_error_m
                  << " yaw_error=" << run.final_yaw_error_rad << "\n";
    }
    m3e_test::expect(
        run.startup_mode == "load_existing" &&
            run.initial_pose_mode == "relocalization",
        "formal runner uses load-existing relocalization startup");
    m3e_test::expect(run.relocalization_success_count == 1 &&
                         run.relocalization_search_count == 2 &&
                         run.localization_stop_count >= 1,
                     "two independent rotating bundles confirm startup pose");
    m3e_test::expect(run.exploration_resumed &&
                         run.exploration.plan_count > 0,
                     "exploration replans after automatic startup localization");
    const auto map_from_world = compose_robot_poses(
        reference.exploration.final_estimated_map_pose,
        inverse_robot_pose(reference.final_ground_truth_pose));
    const auto expected_map_pose = compose_robot_poses(
        map_from_world, run.final_ground_truth_pose);
    const auto estimated_map_pose =
        run.exploration.final_estimated_map_pose;
    const double map_position_error = std::hypot(
        estimated_map_pose.x_m - expected_map_pose.x_m,
        estimated_map_pose.y_m - expected_map_pose.y_m);
    const double map_yaw_error = std::fabs(sparse_slam_shortest_yaw_delta(
        estimated_map_pose.yaw_rad, expected_map_pose.yaw_rad));
    m3e_test::expect(map_position_error <= 0.35 &&
                         map_yaw_error <= 0.35,
                     "estimated pose agrees with truth expressed in map frame");
    m3e_test::expect(!run.collision,
                     "startup relocalization and resumed exploration are safe");
    m3e_test::expect(
        run.exploration.map_revision_start >=
            reference.exploration.map_revision_end,
        "reference map remains available with restored revision");
    m3e_test::expect(run.final_map_save_success,
                     "relocalized exploration saves a final map artifact");
    const auto repeat =
        M3EExplorationRunner{}.run(config, relocalized_repeat_dir);
    m3e_test::expect(repeat.relocalization_success_count == 1 &&
                         repeat.relocalization_search_count ==
                             run.relocalization_search_count &&
                         repeat.relocalization_best_x_m ==
                             run.relocalization_best_x_m &&
                         repeat.relocalization_best_y_m ==
                             run.relocalization_best_y_m &&
                         repeat.relocalization_best_yaw_rad ==
                             run.relocalization_best_yaw_rad &&
                         repeat.termination_reason == run.termination_reason,
                     "startup relocalization is deterministic");
    return 0;
}
