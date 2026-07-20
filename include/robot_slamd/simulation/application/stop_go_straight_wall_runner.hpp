#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_map_artifact.hpp"
#include "robot_slamd/mapping/stop_go_mapping_controller.hpp"
#include "robot_slamd/simulation/core/sim_clock.hpp"
#include "robot_slamd/simulation/ports/sim_relative_motion_step_port.hpp"
#include "robot_slamd/simulation/ports/sim_sensor_port.hpp"
#include "robot_slamd/simulation/ports/sim_stop_go_tof_reader.hpp"
#include "robot_slamd/replay/stop_go_replay_codec.hpp"

#include <cmath>
#include <fstream>
#include <memory>
#include <string>

namespace robot_slamd {

class StopGoStraightWallSimulationRunner final {
public:
    StopGoMappingRunReport run(const Config &input_config,
                               const std::string &run_dir) const {
        return run_scenario(input_config, run_dir, "straight");
    }

    StopGoMappingRunReport run_scenario(const Config &input_config,
                                        const std::string &run_dir,
                                        const std::string &scenario) const {
        Config config = input_config;
        StopGoMappingRunReport failed;
        SensorSource source;
        OperationMode operation;
        if (!parse_sensor_source(config.runtime_sensor_source, source) ||
            !parse_operation_mode(config.runtime_operation, operation) ||
            source != SensorSource::Simulation ||
            operation != OperationMode::StopGoWallMapping) {
            failed.termination_reason = "stop_go_simulation_source_or_operation_rejected";
            return failed;
        }

        auto clock = std::make_shared<SimClock>(0.0);
        auto world = std::make_shared<FakeWorld2D>();
        if (!world->add_axis_aligned_room(-2.0, -2.0, 2.0, 2.0) ||
            !world->add_axis_aligned_obstacle(-1.8, 0.60, 1.8, 0.605)) {
            failed.termination_reason = "straight_wall_world_setup_failed";
            return failed;
        }
        if (scenario == "front_blocked" &&
            !world->add_axis_aligned_obstacle(-1.02, 0.35, -0.99, 0.55)) {
            failed.termination_reason = "front_blocked_world_setup_failed";
            return failed;
        }
        if (scenario == "heading_toward_wall") {
            config.exploration_simulation_initial_yaw_rad = 5.0 * kPi / 180.0;
        } else if (scenario == "heading_away_from_wall") {
            config.exploration_simulation_initial_yaw_rad = -5.0 * kPi / 180.0;
        } else if (scenario == "too_far_from_wall") {
            config.exploration_simulation_initial_y_m = 0.43;
            // The formal too-far fixture gives the bounded bias enough
            // longitudinal distance to demonstrate convergence while the
            // default P4 gain remains the configured provisional value.
            config.stop_go_wall_distance_to_heading_gain_deg_per_m = 60.0;
        } else if (scenario == "too_close_to_wall") {
            config.exploration_simulation_initial_y_m = 0.47;
            config.sparse_slam_left_tof_y_m = 0.03;
            config.stop_go_wall_distance_to_heading_gain_deg_per_m = 60.0;
        }
        SimRobotPlantConfig plant_config;
        plant_config.wheel_radius_m = config.wheel_radius_left_m;
        plant_config.wheel_base_m = config.wheel_base_m;
        plant_config.collision_check_enabled = scenario != "straight";
        auto plant = std::make_shared<SimRobotPlant>(plant_config);
        if (!plant->reset({config.exploration_simulation_initial_x_m,
                           config.exploration_simulation_initial_y_m,
                           config.exploration_simulation_initial_yaw_rad}, 0.0)) {
            failed.termination_reason = "straight_wall_plant_reset_failed";
            return failed;
        }

        SimSensorPortConfig sensor_config;
        sensor_config.contract_options.protocol_min_distance_mm = 20;
        sensor_config.contract_options.protocol_max_distance_mm = 4500;
        sensor_config.contract_options.mapping_min_distance_mm = 30;
        sensor_config.contract_options.mapping_max_distance_mm = 4500;
        sensor_config.contract_options.mapping_min_confidence =
            config.stop_go_tof_mapping_min_confidence;
        sensor_config.contract_options.left_mount_yaw_rad = 1.5707963267948966;
        sensor_config.contract_options.right_mount_yaw_rad = -1.5707963267948966;
        SimThreeScalarTofConfig tof_config;
        tof_config.max_range_m = 4.0;
        tof_config.no_hit_as_explicit_no_return = false;
        tof_config.extrinsics = {
            {config.sparse_slam_front_tof_x_m, config.sparse_slam_front_tof_y_m,
             config.sparse_slam_front_tof_yaw_rad},
            {config.sparse_slam_left_tof_x_m, config.sparse_slam_left_tof_y_m,
             config.sparse_slam_left_tof_yaw_rad},
            {config.sparse_slam_right_tof_x_m, config.sparse_slam_right_tof_y_m,
             config.sparse_slam_right_tof_yaw_rad}};
        auto sensor = std::make_shared<SimSensorPort>(
            clock, world, plant, sensor_config, SimWheelEncoder{}, SimImu{},
            SimThreeScalarTof(tof_config));
        if (!sensor->ready()) {
            failed.termination_reason = "straight_wall_sensor_not_ready";
            return failed;
        }
        RobotSlamApplication application(config, source, operation, sensor, nullptr);
        const auto initialized = application.initialize();
        if (!initialized.ok) {
            failed.termination_reason = initialized.reason;
            return failed;
        }
        SimRelativeMotionStepPort motion(plant);
        const auto stop_go_config = stop_go_mapping_config_from(config);
        auto front_reader = SimStopGoTofReader(
            clock, world, plant, SimThreeScalarTof(tof_config), StableTofDirection::Front);
        auto left_reader = SimStopGoTofReader(
            clock, world, plant, SimThreeScalarTof(tof_config), StableTofDirection::Left);
        auto right_reader = SimStopGoTofReader(
            clock, world, plant, SimThreeScalarTof(tof_config), StableTofDirection::Right);
        StableThreeTofSampler sampler({
            [&front_reader]() { return front_reader.read(); },
            [&left_reader, &plant, &scenario]() {
                if (scenario == "left_wall_loss" &&
                    plant->state().pose.x_m > -0.95) {
                    return TimedTofFrame{};
                }
                return left_reader.read();
            },
            [&right_reader]() { return right_reader.read(); }}, stop_go_config.tof);
        std::string log_path = stop_go_config.log_path;
        if (log_path.empty() && !run_dir.empty()) log_path = run_dir + "/stop_go_mapping.jsonl";
        StopGoMappingController controller(
            application, motion,
            [&sensor](double now_s) { return sensor->latest_snapshot(now_s); },
            std::move(sampler), stop_go_config, log_path);

        const double dt = config.exploration_simulation_fixed_dt_s > 0.0
            ? config.exploration_simulation_fixed_dt_s : 0.02;
        bool tick_ok = controller.tick(clock->now_s());
        std::size_t guard = 0;
        while (tick_ok && !controller.terminal() && guard++ < 20000U) {
            if (!clock->advance(dt) || !motion.advance(dt, clock->now_s(), world.get())) {
                tick_ok = false;
                break;
            }
            tick_ok = controller.tick(clock->now_s());
        }
        auto report = controller.report();
        if (!tick_ok && !controller.terminal()) {
            motion.stop(clock->now_s());
            report.ok = false;
            report.termination_reason = "straight_wall_simulation_tick_failed";
        }
        if (!controller.terminal()) {
            motion.stop(clock->now_s());
            report.ok = false;
            report.termination_reason = "straight_wall_simulation_guard_exhausted";
        }
        report.collision_count = plant->state().collision ? 1U : 0U;
        report.map_revision = application.core().report().current_map_revision;
        report.map_cells = application.core().report().sparse_map_cell_count;
        report.final_estimated_pose = application.core().current_map_pose().map_T_base;
        const auto map_snapshot = application.core().sparse_map_snapshot();
        for (const auto &cell : map_snapshot.cells()) {
            if (cell.evidence >= map_snapshot.occupied_threshold() &&
                cell.key.y >= 10 && cell.key.y <= 13 &&
                cell.key.x >= -1 && cell.key.x <= 10) {
                report.left_wall_observed_cells++;
            }
        }
        const std::string map_path = run_dir.empty()
            ? "/tmp/robot_slamd_stop_go_straight_wall.smap"
            : run_dir + "/stop_go_straight_wall.smap";
        const auto saved = application.core().save_sparse_map(map_path);
        if (saved.ok) {
            std::ifstream input(map_path, std::ios::binary);
            std::ostringstream text;
            text << input.rdbuf();
            const std::string payload = text.str();
            const auto checksum_pos = payload.rfind("checksum ");
            report.final_map_checksum = checksum_pos == std::string::npos
                ? 0 : sparse_map_fnv1a64(payload.substr(0, checksum_pos));
        } else {
            report.ok = false;
            report.termination_reason = "straight_wall_map_save_failed:" + saved.reason;
        }
        if (!run_dir.empty()) {
            std::string replay_reason;
            if (!StopGoReplayLogCodec::write(
                    run_dir + "/stop_go_mapping.replay",
                    report.replay_records, replay_reason)) {
                report.log_error = replay_reason;
            }
        }
        if (config.stop_go_mapping_mode == "left_wall_follow") {
            report.ok = report.ok && report.completed_steps > 0 &&
                        !report.replay_records.empty() &&
                        report.map_commits == report.replay_records.size() &&
                        report.collision_count == 0 &&
                        report.map_writes_while_moving == 0 &&
                        report.map_writes_during_turn_or_verify == 0;
        } else {
            report.ok = report.ok && report.completed_steps > 0 &&
                        report.stable_samples == report.completed_steps + 1 &&
                        report.map_commits == report.stable_samples &&
                        report.collision_count == 0 &&
                        report.map_writes_while_moving == 0;
        }
        report.ground_truth_used_by_algorithm = false;
        report.command_speed_used_as_odometry = false;
        report.command_target_used_as_odometry = false;
        return report;
    }
};

} // namespace robot_slamd
