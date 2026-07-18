#include "robot_slamd/config/config.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"

#include <filesystem>
#include <algorithm>
#include <cmath>
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
    const auto has = [](const StopGoMappingRunReport &report, const char *state) {
        return std::find(report.state_trace.begin(), report.state_trace.end(), state) !=
               report.state_trace.end();
    };
    const auto first_index = [](const StopGoMappingRunReport &report,
                                const char *state) {
        const auto found = std::find(report.state_trace.begin(),
                                     report.state_trace.end(), state);
        return found == report.state_trace.end()
            ? report.state_trace.size()
            : static_cast<std::size_t>(found - report.state_trace.begin());
    };

    const auto path = std::string("/tmp/p2_stop_go_controller_test");
    std::filesystem::create_directories(path);
    auto report = StopGoStraightWallSimulationRunner{}.run(test_config(), path);
    if (!report.ok || report.completed_steps != 3 ||
        report.commands_submitted != 3 || report.commands_completed != 3 ||
        report.stable_samples != 4 || report.map_commits != 4 ||
        report.map_writes_while_moving != 0 || report.collision_count != 0 ||
        report.command_speed_used_as_odometry || report.ground_truth_used_by_algorithm ||
        !report.core_ready_observed || report.core_wait_ticks == 0) return 1;
    if (!has(report, "WAITING_FOR_CORE_READY") || !has(report, "INITIAL_SETTLE") ||
        !has(report, "INITIAL_SAMPLE") ||
        !has(report, "WAIT_MOTION_DONE") || !has(report, "WAIT_MAPPING_SETTLE") ||
        !has(report, "ACQUIRE_THREE_TOF") || !has(report, "COMMIT_MAPPING_SAMPLE") ||
        !has(report, "COMPLETED")) return 1;
    if (first_index(report, "WAITING_FOR_CORE_READY") >=
            first_index(report, "INITIAL_SETTLE") ||
        first_index(report, "INITIAL_SETTLE") >=
            first_index(report, "ISSUE_FORWARD_STEP")) return 1;
    if (report.command_ids.size() != 3 || report.command_ids[0] == report.command_ids[1] ||
        report.command_ids[1] == report.command_ids[2] || report.command_ids[0] != 1) return 1;

    // The command baseline is local to the motion port.  Canonical wheel
    // ticks and Core odometry remain continuous across stop-go cycles.
    if (report.replay_records.size() != 4) return 1;
    for (std::size_t index = 1; index < report.replay_records.size(); ++index) {
        const auto &previous = report.replay_records[index - 1];
        const auto &current = report.replay_records[index];
        if (current.snapshot.wheel.left_ticks + 1e-9 < previous.snapshot.wheel.left_ticks ||
            current.snapshot.wheel.right_ticks + 1e-9 < previous.snapshot.wheel.right_ticks ||
            current.odom_pose.x_m + 1e-12 < previous.odom_pose.x_m ||
            std::fabs(current.odom_pose.y_m - previous.odom_pose.y_m) > 1e-9 ||
            std::fabs(current.map_from_odom.x_m - previous.map_from_odom.x_m) > 1e-12 ||
            std::fabs(current.map_from_odom.y_m - previous.map_from_odom.y_m) > 1e-12 ||
            std::fabs(current.map_from_odom.yaw_rad - previous.map_from_odom.yaw_rad) > 1e-12) {
            return 1;
        }
    }

    // An incomplete stationary calibration is a non-fault waiting state.  A
    // finite simulation guard terminates the test without ever submitting a
    // motion command or entering the formal stop-go loop.
    auto waiting_config = test_config();
    waiting_config.sparse_slam_startup_min_imu_samples = 1000000;
    waiting_config.stop_go_max_steps = 1;
    const auto waiting_path = std::string("/tmp/p2_stop_go_controller_waiting_test");
    std::filesystem::create_directories(waiting_path);
    const auto waiting = StopGoStraightWallSimulationRunner{}.run(
        waiting_config, waiting_path);
    if (waiting.ok || waiting.commands_submitted != 0 ||
        !has(waiting, "WAITING_FOR_CORE_READY") || has(waiting, "FAULT") ||
        has(waiting, "ISSUE_FORWARD_STEP") || waiting.core_ready_observed ||
        waiting.state_trace.empty() || waiting.state_trace.back() != "WAITING_FOR_CORE_READY") return 1;

    // With a ready Core and a one-step limit, exactly one command is issued;
    // readiness is not allowed to duplicate the first submission.
    auto one_step_config = test_config();
    one_step_config.stop_go_max_steps = 1;
    one_step_config.stop_go_max_total_distance_mm = 20.0;
    const auto one_step_path = std::string("/tmp/p2_stop_go_controller_one_step_test");
    std::filesystem::create_directories(one_step_path);
    const auto one_step = StopGoStraightWallSimulationRunner{}.run(
        one_step_config, one_step_path);
    if (!one_step.ok || one_step.commands_submitted != 1 ||
        one_step.command_ids.size() != 1 || one_step.command_ids.front() != 1 ||
        !one_step.core_ready_observed) return 1;
    return 0;
}
