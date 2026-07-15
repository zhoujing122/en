#pragma once

#include "robot_slamd/config/config.hpp"
#include "robot_slamd/exploration/autonomous_exploration_controller.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_map_artifact.hpp"
#include "robot_slamd/simulation/core/sim_clock.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"
#include "robot_slamd/simulation/ports/sim_sensor_port.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

namespace robot_slamd {

struct M3EExplorationRunReport {
    bool ok = false;
    AutonomousExplorationReport exploration;
    std::size_t simulation_steps = 0;
    double simulated_duration_s = 0.0;
    double ground_truth_travel_distance_m = 0.0;
    bool collision = false;
    std::size_t sensor_snapshot_count = 0;
    std::size_t motion_command_count = 0;
    std::size_t stop_command_count = 0;
    bool final_map_save_success = false;
    std::string final_map_path;
    std::uint64_t final_map_checksum = 0;
    std::string report_path;
    std::string trajectory_path;
    std::string termination_reason = "not_started";
};

inline AutonomousExplorationConfig m3e_exploration_config_from(
    const Config &config) {
    AutonomousExplorationConfig out;
    out.navigation.min_x_m = config.exploration_min_x_m;
    out.navigation.max_x_m = config.exploration_max_x_m;
    out.navigation.min_y_m = config.exploration_min_y_m;
    out.navigation.max_y_m = config.exploration_max_y_m;
    out.navigation.robot_radius_m = config.exploration_robot_radius_m;
    out.navigation.safety_margin_m = config.exploration_safety_margin_m;
    out.navigation.maximum_cells = static_cast<std::size_t>(
        config.exploration_maximum_navigation_cells);
    out.frontier.minimum_cluster_cells = static_cast<std::size_t>(
        config.exploration_minimum_frontier_cluster_cells);
    out.frontier.information_gain_weight =
        config.exploration_information_gain_weight;
    out.frontier.minimum_goal_clearance_m =
        config.exploration_minimum_goal_clearance_m;
    out.frontier.minimum_goal_distance_m =
        config.exploration_minimum_goal_distance_m;
    out.frontier.astar.maximum_expanded_nodes = static_cast<std::size_t>(
        config.exploration_maximum_astar_nodes);
    out.no_frontier_confirmation_cycles = static_cast<std::size_t>(
        config.exploration_no_frontier_confirmation_cycles);
    out.completion_minimum_known_ratio =
        config.exploration_completion_minimum_known_ratio;
    out.bootstrap_minimum_known_cells = static_cast<std::size_t>(
        config.exploration_bootstrap_minimum_known_cells);
    out.bootstrap_minimum_yaw_rad = config.exploration_bootstrap_minimum_yaw_rad;
    out.bootstrap_max_duration_s = config.exploration_bootstrap_max_duration_s;
    out.maximum_duration_s = config.exploration_max_duration_s;
    out.maximum_planning_failures = static_cast<std::size_t>(
        config.exploration_maximum_planning_failures);
    out.failure_memory.maximum_entries = static_cast<std::size_t>(
        config.exploration_failure_memory_capacity);
    out.failure_memory.cooldown_planning_cycles = static_cast<std::size_t>(
        config.exploration_failure_cooldown_cycles);
    out.failure_memory.maximum_consecutive_failures = static_cast<std::size_t>(
        config.exploration_maximum_goal_failures);
    out.failure_memory.revision_change_for_retry = static_cast<std::uint64_t>(
        config.exploration_failure_retry_revision_delta);
    out.bootstrap_turn_speed_normalized = 0.38;
    out.bootstrap_turn_segment_duration_s = 0.50;
    out.tracker.turn_speed_normalized = 0.35;
    out.tracker.forward_speed_normalized = 0.40;
    out.tracker.turn_segment_duration_s = 0.40;
    out.tracker.forward_segment_duration_s = 0.50;
    out.tracker.waypoint_tolerance_m = config.exploration_waypoint_tolerance_m;
    out.tracker.yaw_tolerance_rad = config.exploration_yaw_tolerance_rad;
    out.tracker.minimum_progress_m = config.exploration_minimum_progress_m;
    out.tracker.maximum_no_progress_duration_s =
        config.exploration_no_progress_timeout_s;
    out.tracker.maximum_segments_without_progress = static_cast<std::size_t>(
        config.exploration_maximum_segments_without_progress);
    out.tracker.obstacle_stop_distance_m =
        config.exploration_obstacle_stop_distance_m;
    out.tracker.emergency_stop_distance_m =
        config.exploration_emergency_stop_distance_m;
    out.tracker.obstacle_confirmation_frames = static_cast<std::size_t>(
        config.exploration_obstacle_confirmation_frames);
    return out;
}

inline void write_m3e_exploration_report(
    const M3EExplorationRunReport &report,
    const std::string &path) {
    std::ofstream out(path);
    out << "ok=" << (report.ok ? 1 : 0) << "\n"
        << "state=" << to_string(report.exploration.state) << "\n"
        << "termination_reason=" << report.termination_reason << "\n"
        << "known_cells_start=" << report.exploration.known_cells_start << "\n"
        << "known_cells_end=" << report.exploration.known_cells_end << "\n"
        << "occupied_cells_start=" << report.exploration.occupied_cells_start << "\n"
        << "occupied_cells_end=" << report.exploration.occupied_cells_end << "\n"
        << "free_cells_start=" << report.exploration.free_cells_start << "\n"
        << "free_cells_end=" << report.exploration.free_cells_end << "\n"
        << "map_revision_start=" << report.exploration.map_revision_start << "\n"
        << "map_revision_end=" << report.exploration.map_revision_end << "\n"
        << "frontiers_detected=" << report.exploration.frontiers_detected << "\n"
        << "reachable_frontiers=" << report.exploration.reachable_frontiers << "\n"
        << "selected_goals=" << report.exploration.selected_goals << "\n"
        << "goal_attempt_count=" << report.exploration.goal_attempt_count << "\n"
        << "completed_goals=" << report.exploration.completed_goals << "\n"
        << "failed_goals=" << report.exploration.failed_goals << "\n"
        << "plan_count=" << report.exploration.plan_count << "\n"
        << "replan_count=" << report.exploration.replan_count << "\n"
        << "last_planning_reason="
        << report.exploration.last_planning_reason << "\n"
        << "expanded_astar_nodes=" << report.exploration.expanded_astar_nodes << "\n"
        << "commanded_turns=" << report.exploration.commanded_turns << "\n"
        << "commanded_forward_segments="
        << report.exploration.commanded_forward_segments << "\n"
        << "obstacle_stops=" << report.exploration.obstacle_stops << "\n"
        << "matcher_rejected_goals=" << report.exploration.matcher_rejected_goals << "\n"
        << "path_blocked_goals=" << report.exploration.path_blocked_goals << "\n"
        << "no_progress_goals=" << report.exploration.no_progress_goals << "\n"
        << "obstacle_blocked_goals=" << report.exploration.obstacle_blocked_goals << "\n"
        << "exploration_bounds_total_cells=" << report.exploration.exploration_bounds_total_cells << "\n"
        << "known_ratio=" << report.exploration.known_ratio << "\n"
        << "free_ratio=" << report.exploration.free_ratio << "\n"
        << "unknown_ratio=" << report.exploration.unknown_ratio << "\n"
        << "remaining_frontier_clusters=" << report.exploration.remaining_frontier_clusters << "\n"
        << "reachable_frontier_clusters=" << report.exploration.reachable_frontier_clusters << "\n"
        << "cooled_frontier_clusters=" << report.exploration.cooled_frontier_clusters << "\n"
        << "permanently_failed_frontier_clusters=" << report.exploration.permanently_failed_frontier_clusters << "\n"
        << "completion_confirmation_cycles=" << report.exploration.completion_confirmation_cycles << "\n"
        << "estimated_travel_distance_m="
        << report.exploration.estimated_travel_distance_m << "\n"
        << "matcher_attempts=" << report.exploration.matcher_attempts << "\n"
        << "atomic_commit_successes="
        << report.exploration.atomic_commit_successes << "\n"
        << "keyframe_count=" << report.exploration.keyframe_count << "\n"
        << "final_pose_x_m=" << report.exploration.final_estimated_map_pose.x_m << "\n"
        << "final_pose_y_m=" << report.exploration.final_estimated_map_pose.y_m << "\n"
        << "final_pose_yaw_rad="
        << report.exploration.final_estimated_map_pose.yaw_rad << "\n"
        << "simulation_steps=" << report.simulation_steps << "\n"
        << "simulated_duration_s=" << report.simulated_duration_s << "\n"
        << "ground_truth_travel_distance_m="
        << report.ground_truth_travel_distance_m << "\n"
        << "collision=" << (report.collision ? 1 : 0) << "\n"
        << "final_map_save_success="
        << (report.final_map_save_success ? 1 : 0) << "\n"
        << "final_map_path=" << report.final_map_path << "\n"
        << "final_map_checksum=" << report.final_map_checksum << "\n";
}

class M3EExplorationRunner {
public:
    M3EExplorationRunReport run(const Config &input_config,
                                const std::string &run_dir,
                                double duration_override_s = -1.0) const {
        M3EExplorationRunReport report;
        report.report_path = run_dir + "/exploration_report.txt";
        report.trajectory_path = run_dir + "/exploration_trajectory.csv";
        report.final_map_path = run_dir + "/final_sparse_map.smap";
        Config config = input_config;
        if (duration_override_s > 0.0) {
            config.exploration_max_duration_s = duration_override_s;
        }
        const double dt = config.exploration_simulation_fixed_dt_s;
        auto clock = std::make_shared<SimClock>(0.0);
        auto world = std::make_shared<FakeWorld2D>();
        world->add_axis_aligned_room(
            config.exploration_min_x_m, config.exploration_min_y_m,
            config.exploration_max_x_m, config.exploration_max_y_m);
        world->add_axis_aligned_obstacle(0.25, -0.15, 0.55, 0.35);
        SimRobotPlantConfig plant_config;
        plant_config.wheel_radius_m = config.wheel_radius_left_m;
        plant_config.wheel_base_m = config.wheel_base_m;
        plant_config.robot_radius_m = config.exploration_robot_radius_m;
        plant_config.collision_check_enabled = true;
        auto plant = std::make_shared<SimRobotPlant>(plant_config);
        if (!plant->reset({}, 0.0)) {
            report.termination_reason = "plant_reset_failed";
            write_m3e_exploration_report(report, report.report_path);
            return report;
        }
        SimMotionPortConfig motion_config;
        motion_config.allow_translation = true;
        motion_config.maximum_algorithm_speed_normalized = 0.40;
        motion_config.maximum_algorithm_duration_s = 0.50;
        auto motion = std::make_shared<SimMotionPort>(plant, motion_config);
        SimThreeScalarTofConfig tof_config;
        tof_config.max_range_m = config.exploration_simulation_tof_max_range_m;
        tof_config.no_hit_as_explicit_no_return = true;
        tof_config.front_latency_s = 0.0;
        tof_config.left_latency_s = 0.0;
        tof_config.right_latency_s = 0.0;
        tof_config.left_response_offset_s = 0.0;
        tof_config.right_response_offset_s = 0.0;
        tof_config.extrinsics = {
            {config.sparse_slam_front_tof_x_m,
             config.sparse_slam_front_tof_y_m,
             config.sparse_slam_front_tof_yaw_rad},
            {config.sparse_slam_left_tof_x_m,
             config.sparse_slam_left_tof_y_m,
             config.sparse_slam_left_tof_yaw_rad},
            {config.sparse_slam_right_tof_x_m,
             config.sparse_slam_right_tof_y_m,
             config.sparse_slam_right_tof_yaw_rad}};
        SimSensorPortConfig sensor_config;
        SimWheelEncoderConfig wheel_config;
        wheel_config.left_request_latency_s = 0.0;
        wheel_config.right_request_latency_s = 0.0;
        auto sensor = std::make_shared<SimSensorPort>(
            clock, world, plant, sensor_config, SimWheelEncoder(wheel_config), SimImu{},
            SimThreeScalarTof(tof_config));
        SparseSlamRuntimeCore core(config);
        auto init_request = sparse_slam_initialization_request_from_config(config);
        const auto initialized = core.initialize(init_request);
        if (initialized.status == SparseSlamInitializationStatus::Fault) {
            report.termination_reason = initialized.message;
            write_m3e_exploration_report(report, report.report_path);
            return report;
        }
        AutonomousExplorationController controller(
            m3e_exploration_config_from(config));
        std::ofstream trajectory(report.trajectory_path);
        trajectory << "timestamp_s,estimated_x_m,estimated_y_m,estimated_yaw_rad,"
                      "state,map_revision,map_cells\n";
        const std::size_t maximum_steps = static_cast<std::size_t>(
            std::ceil(config.exploration_max_duration_s / dt)) + 1U;
        bool terminal = false;
        for (std::size_t step = 0; step < maximum_steps && !terminal; ++step) {
            const double now = clock->now_s();
            motion->update(now);
            const auto snapshot = sensor->latest_snapshot(now);
            const auto feedback = motion->latest_feedback(now);
            const auto before = plant->state().pose;
            const auto result = controller.step(
                snapshot, now, feedback, core, *motion);
            const auto pose = core.current_map_pose().map_T_base;
            trajectory << now << ',' << pose.x_m << ',' << pose.y_m << ','
                       << pose.yaw_rad << ',' << to_string(controller.state()) << ','
                       << core.report().current_map_revision << ','
                       << core.report().sparse_map_cell_count << '\n';
            if (!plant->step(dt, now + dt, world.get())) {
                report.termination_reason = "plant_step_failed";
                break;
            }
            const auto after = plant->state().pose;
            report.ground_truth_travel_distance_m +=
                std::hypot(after.x_m - before.x_m, after.y_m - before.y_m);
            report.collision = report.collision || plant->state().collision;
            report.simulation_steps++;
            terminal = result.terminal;
            report.termination_reason = result.reason;
            if (!clock->advance(dt)) {
                report.termination_reason = "clock_advance_failed";
                break;
            }
        }
        report.simulated_duration_s = clock->now_s();
        report.exploration = controller.report();
        report.sensor_snapshot_count = static_cast<std::size_t>(sensor->success_count());
        report.motion_command_count = static_cast<std::size_t>(motion->accepted_count());
        report.stop_command_count = static_cast<std::size_t>(motion->stop_count());
        if (controller.state() == AutonomousExplorationState::Completed) {
            const auto saved = core.save_sparse_map(report.final_map_path);
            report.final_map_save_success = saved.ok;
            if (saved.ok) {
                const auto loaded = load_sparse_map_artifact(
                    report.final_map_path,
                    {static_cast<std::size_t>(config.sparse_slam_map_artifact_max_cells),
                     static_cast<std::size_t>(config.sparse_slam_map_artifact_max_file_bytes)});
                if (loaded.ok) {
                    report.final_map_checksum = sparse_map_fnv1a64(
                        sparse_map_artifact_payload(loaded.artifact));
                }
            }
        }
        report.ok = controller.state() == AutonomousExplorationState::Completed &&
            report.exploration.selected_goals >= 3 &&
            report.exploration.completed_goals >= 3 &&
            report.exploration.completed_goals * 2 >=
                report.exploration.selected_goals &&
            report.exploration.expanded_astar_nodes > 0 &&
            report.exploration.commanded_forward_segments > 0 &&
            report.exploration.known_cells_end > report.exploration.known_cells_start &&
            report.exploration.map_revision_end > report.exploration.map_revision_start &&
            report.exploration.plan_count > 0 &&
            report.exploration.replan_count > 0 &&
            report.final_map_save_success && !report.collision;
        if (!report.ok && report.termination_reason.empty()) {
            report.termination_reason = "m3e_acceptance_not_met";
        }
        write_m3e_exploration_report(report, report.report_path);
        return report;
    }
};

} // namespace robot_slamd
