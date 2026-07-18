#include "robot_slamd/config/config.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"

#include <filesystem>
#include <iostream>

static robot_slamd::Config config() {
    robot_slamd::Config c;
    c.runtime_sensor_source = "simulation";
    c.runtime_operation = "stop_go_wall_mapping";
    c.stop_go_mapping_enabled = true;
    c.sparse_slam_planar_tof_extrinsics_configured = true;
    c.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    c.sparse_slam_front_tof_x_m = 0.10;
    c.sparse_slam_left_tof_y_m = 0.08;
    c.sparse_slam_left_tof_yaw_rad = 1.5707963267948966;
    c.sparse_slam_right_tof_y_m = -0.08;
    c.sparse_slam_right_tof_yaw_rad = -1.5707963267948966;
    c.exploration_simulation_initial_x_m = -1.2;
    c.exploration_simulation_initial_y_m = 0.0;
    c.exploration_simulation_initial_yaw_rad = 0.0;
    c.sparse_slam_map_run_id = "formal_stop_go_test";
    return c;
}

int main() {
    using namespace robot_slamd;
    const std::string path = "/tmp/p2_stop_go_formal_test";
    std::filesystem::create_directories(path);
    const auto report = StopGoStraightWallSimulationRunner{}.run(config(), path);
    std::cout << "completed_steps=" << report.completed_steps
              << " commands_submitted=" << report.commands_submitted
              << " commands_completed=" << report.commands_completed
              << " core_wait_ticks=" << report.core_wait_ticks
              << " core_ready_observed=" << report.core_ready_observed
              << " stable_samples=" << report.stable_samples
              << " map_commits=" << report.map_commits
              << " map_revision=" << report.map_revision
              << " map_cells=" << report.map_cells
              << " left_wall_observed_cells=" << report.left_wall_observed_cells
              << " collision_count=" << report.collision_count
              << " map_writes_while_moving=" << report.map_writes_while_moving
              << " command_speed_used_as_odometry=" << report.command_speed_used_as_odometry
              << " ground_truth_used_by_algorithm=" << report.ground_truth_used_by_algorithm
              << " final_map_checksum=" << report.final_map_checksum
              << " termination_reason=" << report.termination_reason << "\n";
    return report.ok && report.completed_steps == 20 && report.commands_completed == 20 &&
                   report.stable_samples == 21 && report.map_commits == 21 &&
                   report.map_revision == 21 && report.map_cells > 100 &&
                   report.left_wall_observed_cells > 0 &&
                   report.collision_count == 0 && report.map_writes_while_moving == 0 &&
                   !report.command_speed_used_as_odometry &&
                   !report.ground_truth_used_by_algorithm
               ? 0 : 1;
}
