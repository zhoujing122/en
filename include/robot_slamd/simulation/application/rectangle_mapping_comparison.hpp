#pragma once

#include "robot_slamd/simulation/application/simulation_robot_slam_runner.hpp"
#include "robot_slamd/simulation/exploration/m3e_exploration_runner.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"

#include <filesystem>
#include <array>
#include <cmath>

namespace robot_slamd {

struct RectangleMappingComparisonReport {
    bool stop_go_completed = false;
    std::string stop_go_termination;
    std::string frontier_termination;
    std::size_t stop_go_collision_count = 0;
    std::size_t frontier_collision_count = 0;
    std::size_t stop_go_simulation_steps = 0;
    std::size_t frontier_simulation_steps = 0;
    double stop_go_odom_travel_distance_m = 0.0;
    double frontier_odom_travel_distance_m = 0.0;
    std::uint64_t stop_go_map_revision = 0;
    std::uint64_t frontier_map_revision = 0;
    std::size_t stop_go_map_commits = 0;
    std::size_t frontier_map_commits = 0;
    std::size_t stop_go_map_cells = 0;
    std::size_t frontier_map_cells = 0;
    double stop_go_wall_coverage_ratio = 0.0;
    double frontier_known_ratio = 0.0;
    double frontier_wall_coverage_ratio = 0.0;
    double stop_go_p95_wall_thickness_cells = 0.0;
    double frontier_p95_wall_thickness_cells = 0.0;
    double frontier_maximum_wall_thickness_cells = 0.0;
    double stop_go_ghost_occupied_ratio = 0.0;
    double frontier_ghost_occupied_ratio = 0.0;
    std::size_t stop_go_duplicate_wall_bands = 0;
    std::size_t frontier_duplicate_wall_bands = 0;
    double stop_go_estimated_closure_error_m = 0.0;
    double frontier_estimated_closure_error_m = 0.0;
    double stop_go_estimated_closure_yaw_error_rad = 0.0;
    double frontier_estimated_closure_yaw_error_rad = 0.0;
    double stop_go_ground_truth_closure_error_m = 0.0;
    double frontier_ground_truth_closure_error_m = 0.0;
    double stop_go_ground_truth_closure_yaw_error_rad = 0.0;
    double frontier_ground_truth_closure_yaw_error_rad = 0.0;
    std::uint64_t stop_go_map_checksum = 0;
    std::uint64_t frontier_map_checksum = 0;
    std::size_t stop_go_runtime_cycles = 0;
    std::size_t frontier_runtime_cycles = 0;
};

inline RectangleMappingComparisonReport run_rectangle_mapping_comparison(
    const Config &config, const StopGoMappingRunReport &stop_go,
    const std::string &run_dir) {
    RectangleMappingComparisonReport result;
    result.stop_go_completed = stop_go.ok &&
        stop_go.termination_reason == "rectangle_closure_confirmed";
    result.stop_go_termination = stop_go.termination_reason;
    result.stop_go_collision_count = stop_go.collision_count;
    result.stop_go_simulation_steps = stop_go.simulation_tick_count;
    result.stop_go_odom_travel_distance_m = stop_go.total_odom_travel_distance_m;
    result.stop_go_map_revision = stop_go.map_revision;
    result.stop_go_map_commits = stop_go.map_commits;
    result.stop_go_map_cells = stop_go.map_cells;
    result.stop_go_wall_coverage_ratio = stop_go.observable_wall_coverage_ratio;
    result.stop_go_p95_wall_thickness_cells = stop_go.p95_wall_thickness_cells;
    result.stop_go_ghost_occupied_ratio = stop_go.ghost_occupied_cell_ratio;
    result.stop_go_duplicate_wall_bands = stop_go.duplicate_wall_band_count;
    result.stop_go_estimated_closure_error_m = stop_go.estimated_closure_position_error_m;
    result.stop_go_estimated_closure_yaw_error_rad =
        stop_go.estimated_closure_yaw_error_rad;
    result.stop_go_ground_truth_closure_error_m = stop_go.ground_truth_final_position_error_m;
    result.stop_go_ground_truth_closure_yaw_error_rad =
        stop_go.ground_truth_final_yaw_error_rad;
    result.stop_go_map_checksum = stop_go.final_map_checksum;

    Config frontier_config = config;
    std::filesystem::create_directories(run_dir);
    const auto frontier = M3EExplorationRunner{}.run(
        frontier_config, run_dir + "/frontier_a_star", 60.0);
    result.frontier_termination = frontier.termination_reason;
    result.frontier_collision_count = frontier.collision ? 1U : 0U;
    result.frontier_simulation_steps = frontier.simulation_steps;
    result.frontier_odom_travel_distance_m = frontier.exploration.estimated_travel_distance_m;
    result.frontier_map_revision = frontier.core.current_map_revision;
    result.frontier_map_commits = frontier.core.current_map_revision;
    result.frontier_map_cells = frontier.core.sparse_map_cell_count;
    result.frontier_known_ratio = frontier.exploration.known_ratio;
    result.frontier_estimated_closure_error_m = std::hypot(
        frontier.exploration.final_estimated_map_pose.x_m -
            frontier.initial_estimated_map_pose.x_m,
        frontier.exploration.final_estimated_map_pose.y_m -
            frontier.initial_estimated_map_pose.y_m);
    result.frontier_estimated_closure_yaw_error_rad =
        sparse_slam_shortest_yaw_delta(
            frontier.initial_estimated_map_pose.yaw_rad,
            frontier.exploration.final_estimated_map_pose.yaw_rad);
    result.frontier_ground_truth_closure_error_m = std::hypot(
        frontier.final_ground_truth_pose.x_m - frontier.initial_ground_truth_pose.x_m,
        frontier.final_ground_truth_pose.y_m - frontier.initial_ground_truth_pose.y_m);
    result.frontier_ground_truth_closure_yaw_error_rad =
        sparse_slam_shortest_yaw_delta(
            frontier.initial_ground_truth_pose.yaw_rad,
            frontier.final_ground_truth_pose.yaw_rad);
    result.frontier_map_checksum = frontier.final_map_checksum;
    FakeWorld2D rectangle_world;
    const bool comparison_world_ok =
        rectangle_world.add_axis_aligned_room(-2.0, -2.0, 2.0, 2.0) &&
        rectangle_world.add_axis_aligned_obstacle(-1.06, 0.595, 0.80, 0.605) &&
        rectangle_world.add_axis_aligned_obstacle(0.795, -0.90, 0.805, 0.60) &&
        rectangle_world.add_axis_aligned_obstacle(-1.06, -0.905, 0.80, -0.895) &&
        rectangle_world.add_axis_aligned_obstacle(-1.065, -0.90, -1.055, 0.60);
    const std::array<SimLineSegment2D, 4> inner_walls{{
        {-1.06, 0.60, 0.80, 0.60, 101},
        {0.80, -0.90, 0.80, 0.60, 102},
        {0.80, -0.90, -1.06, -0.90, 103},
        {-1.06, 0.60, -1.06, -0.90, 104}}};
    if (comparison_world_ok && frontier.final_map_snapshot.valid()) {
        const auto quality = evaluate_rectangle_map_quality(
            frontier.final_map_snapshot, rectangle_world.segments(), inner_walls,
            frontier.initial_ground_truth_pose, frontier.final_ground_truth_pose,
            frontier.initial_estimated_map_pose);
        result.frontier_wall_coverage_ratio = quality.observable_wall_coverage_ratio;
        result.frontier_p95_wall_thickness_cells = quality.p95_wall_thickness_cells;
        result.frontier_maximum_wall_thickness_cells = quality.maximum_wall_thickness_cells;
        result.frontier_ghost_occupied_ratio = quality.ghost_occupied_cell_ratio;
        result.frontier_duplicate_wall_bands = quality.duplicate_wall_band_count;
    }
    result.frontier_runtime_cycles = frontier.simulation_steps;
    result.stop_go_runtime_cycles = stop_go.state_trace.size();
    return result;
}

} // namespace robot_slamd
