#include "robot_slamd/config/config.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"

#include <filesystem>
#include <algorithm>
#include <iostream>

static robot_slamd::Config test_config() {
    robot_slamd::Config c;
    c.runtime_sensor_source = "simulation";
    c.runtime_operation = "stop_go_wall_mapping";
    c.stop_go_mapping_enabled = true;
    c.stop_go_max_steps = 3;
    c.stop_go_max_total_distance_mm = 60.0;
    c.stop_go_forward_step_mm = 20.0;
    c.stop_go_motion_max_rpm = 12.0;
    c.stop_go_motion_timeout_ms = 3000;
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
    return c;
}

int main() {
    using namespace robot_slamd;
    const auto path = std::string("/tmp/p2_stop_go_controller_test");
    std::filesystem::create_directories(path);
    auto report = StopGoStraightWallSimulationRunner{}.run(test_config(), path);
    if (!report.ok || report.completed_steps != 3 || report.commands_completed != 3 ||
        report.stable_samples != 4 || report.map_commits != 4 ||
        report.map_writes_while_moving != 0 || report.collision_count != 0 ||
        report.command_speed_used_as_odometry || report.ground_truth_used_by_algorithm) return 1;
    const auto has = [&report](const char *state) {
        return std::find(report.state_trace.begin(), report.state_trace.end(), state) !=
               report.state_trace.end();
    };
    if (!has("INITIAL_SETTLE") || !has("INITIAL_SAMPLE") ||
        !has("WAIT_MOTION_DONE") || !has("WAIT_MAPPING_SETTLE") ||
        !has("ACQUIRE_THREE_TOF") || !has("COMMIT_MAPPING_SAMPLE") ||
        !has("COMPLETED")) return 1;
    if (report.command_ids.size() != 3 || report.command_ids[0] == report.command_ids[1] ||
        report.command_ids[1] == report.command_ids[2]) return 1;
    return 0;
}
