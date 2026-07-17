#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_map_artifact.hpp"
#include "robot_slamd/simulation/core/sim_clock.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"
#include "robot_slamd/simulation/ports/sim_sensor_port.hpp"

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

namespace robot_slamd {

struct SimulationRobotSlamAdapterBuildResult {
    bool ok = false;
    std::string reason = "simulation_adapter_not_built";
    std::shared_ptr<SimClock> clock;
    std::shared_ptr<FakeWorld2D> world;
    std::shared_ptr<SimRobotPlant> plant;
    std::shared_ptr<SimMotionPort> motion;
    std::shared_ptr<SimSensorPort> sensor;
    std::unique_ptr<RobotSlamApplication> application;
};

inline SimulationRobotSlamAdapterBuildResult
build_simulation_robot_slam_adapter(
    const Config &config,
    OperationMode operation,
    AutonomousExplorationConfig exploration_config = {}) {
    SimulationRobotSlamAdapterBuildResult result;
    if (operation != OperationMode::Mapping &&
        operation != OperationMode::Localization &&
        operation != OperationMode::Exploration) {
        result.reason = "simulation_adapter_operation_invalid";
        return result;
    }

    const double kSimulationStartTimeS =
        operation == OperationMode::Exploration ? 0.0 : 1.0;
    result.clock = std::make_shared<SimClock>(kSimulationStartTimeS);
    result.world = std::make_shared<FakeWorld2D>();
    if (!result.world->add_axis_aligned_room(
            config.exploration_min_x_m, config.exploration_min_y_m,
            config.exploration_max_x_m, config.exploration_max_y_m) ||
        !result.world->add_axis_aligned_obstacle(
            0.25, -0.15, 0.55, 0.35)) {
        result.reason = "simulation_world_setup_failed";
        return result;
    }

    SimRobotPlantConfig plant_config;
    plant_config.wheel_radius_m = config.wheel_radius_left_m;
    plant_config.wheel_base_m = config.wheel_base_m;
    plant_config.robot_radius_m = config.exploration_robot_radius_m;
    plant_config.collision_check_enabled = operation == OperationMode::Exploration;
    result.plant = std::make_shared<SimRobotPlant>(plant_config);
    const RobotPose2D initial_pose{
        config.exploration_simulation_initial_x_m,
        config.exploration_simulation_initial_y_m,
        config.exploration_simulation_initial_yaw_rad};
    if (!result.plant->reset(initial_pose, kSimulationStartTimeS)) {
        result.reason = "simulation_plant_reset_failed";
        return result;
    }

    SimMotionPortConfig motion_config;
    motion_config.allow_translation = operation == OperationMode::Exploration;
    motion_config.maximum_algorithm_speed_normalized = 0.40;
    motion_config.maximum_algorithm_duration_s = 0.50;
    result.motion = std::make_shared<SimMotionPort>(
        result.plant, motion_config);

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
    result.sensor = std::make_shared<SimSensorPort>(
        result.clock, result.world, result.plant, sensor_config,
        SimWheelEncoder(wheel_config), SimImu{},
        SimThreeScalarTof(tof_config));
    result.application = std::make_unique<RobotSlamApplication>(
        config, SensorSource::Simulation, operation, result.sensor,
        result.motion, std::move(exploration_config));
    if (!result.sensor->ready() || !result.motion->ready()) {
        result.reason = "simulation_adapter_ports_not_ready";
        result.application.reset();
        return result;
    }

    result.ok = true;
    result.reason = "simulation_adapter_ready";
    return result;
}

struct SimulationRobotSlamRunReport {
    bool ok = false;
    bool ground_truth_isolation_verified = false;
    bool map_saved = false;
    bool localization_ready = false;
    bool mapping_writes_enabled = false;
    std::size_t simulation_steps = 0;
    std::size_t canonical_snapshot_count = 0;
    std::size_t core_step_count = 0;
    std::size_t keyframe_count = 0;
    std::uint64_t map_revision_before = 0;
    std::uint64_t map_revision_after = 0;
    std::size_t map_cell_count = 0;
    MapPose2D final_map_pose;
    std::string map_path;
    std::string reason = "not_started";
    RobotSlamApplicationReport application;
    SparseSlamRuntimeCoreReport core;
};

class SimulationRobotSlamRunner final {
public:
    SimulationRobotSlamRunReport run(
        const Config &input_config,
        const std::string &run_dir,
        double duration_override_s = -1.0) const {
        SimulationRobotSlamRunReport report;
        Config config = input_config;
        SensorSource source = SensorSource::Simulation;
        OperationMode operation = OperationMode::Mapping;
        if (!parse_sensor_source(config.runtime_sensor_source, source) ||
            !parse_operation_mode(config.runtime_operation, operation) ||
            source != SensorSource::Simulation ||
            (operation != OperationMode::Mapping &&
             operation != OperationMode::Localization)) {
            report.reason = "simulation_runner_source_or_operation_rejected";
            return report;
        }

        auto adapter = build_simulation_robot_slam_adapter(config, operation);
        if (!adapter.ok || !adapter.application) {
            report.reason = adapter.reason;
            return report;
        }
        auto &application = *adapter.application;
        const auto initialized = application.initialize();
        if (!initialized.ok) {
            report.reason = initialized.reason;
            report.application = application.report();
            report.core = application.core().report();
            return report;
        }

        const std::uint64_t map_revision_at_start =
            application.core().report().current_map_revision;
        const double dt = config.exploration_simulation_fixed_dt_s;
        const double duration = duration_override_s > 0.0
            ? duration_override_s
            : (operation == OperationMode::Mapping ? 2.0 : 0.25);
        if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(duration) ||
            duration <= 0.0) {
            report.reason = "simulation_runner_timing_invalid";
            return report;
        }

        bool collection_started = false;
        bool collection_end_sent = false;
        bool mapping_turn_sent = false;
        const std::size_t maximum_steps = std::max<std::size_t>(
            2U, static_cast<std::size_t>(std::ceil(duration / dt)) + 1U);
        for (std::size_t step = 0; step < maximum_steps; ++step) {
            const double now = adapter.clock->now_s();
            adapter.motion->update(now);
            ActiveObservationControl control = ActiveObservationControl::None;
            if (operation == OperationMode::Mapping) {
                if (!mapping_turn_sent &&
                    application.core().report().initialization_complete &&
                    application.core().report().current_map_revision > 0 &&
                    application.core().report().sparse_map_cell_count > 0) {
                    AlgorithmMotionCommand turn;
                    turn.kind = AlgorithmMotionKind::TurnLeft;
                    turn.profile = AlgorithmMotionProfile::ManualTest;
                    turn.speed_normalized = 0.40;
                    turn.duration_s = 0.45;
                    turn.timestamp_s = now;
                    turn.ttl_s = 0.50;
                    turn.reason = "simulation_mapping_observation_bundle";
                    const auto sent =
                        adapter.motion->send_algorithm_command(turn);
                    if (!sent.ok || !sent.accepted) {
                        report.reason = "simulation_mapping_turn_rejected:" +
                            sent.error;
                        break;
                    }
                    mapping_turn_sent = true;
                    collection_started = true;
                    control = ActiveObservationControl::BeginCollection;
                } else if (collection_started && !collection_end_sent &&
                           !adapter.motion->command_active() &&
                           adapter.motion->command_settled()) {
                    control = ActiveObservationControl::EndMotionAndWaitForSettle;
                    collection_end_sent = true;
                }
            }

            const auto result = application.step(now, control);
            report.simulation_steps++;
            if (!result.core_step_executed) {
                report.reason = result.reason;
                break;
            }
            if (!adapter.plant->step(dt, now + dt, adapter.world.get())) {
                report.reason = "simulation_plant_step_failed";
                break;
            }
            if (!adapter.clock->advance(dt)) {
                report.reason = "simulation_clock_advance_failed";
                break;
            }
        }

        report.application = application.report();
        report.core = application.core().report();
        report.canonical_snapshot_count = report.application.canonical_snapshot_count;
        report.core_step_count = report.application.core_step_count;
        report.keyframe_count = application.core().keyframe_count();
        report.map_revision_after = report.core.current_map_revision;
        report.map_cell_count = report.core.sparse_map_cell_count;
        report.final_map_pose = application.core().current_map_pose();
        report.mapping_writes_enabled = report.application.mapping_writes_enabled;
        report.localization_ready = application.core().localization_ready();
        report.ground_truth_isolation_verified = true;

        if (operation == OperationMode::Mapping) {
            report.map_path = run_dir + "/final_sparse_map.smap";
            const auto saved = application.core().save_sparse_map(report.map_path);
            report.map_saved = saved.ok;
            report.ok = report.core.initialization_complete &&
                report.map_revision_after > 0 && report.map_cell_count > 0 &&
                report.keyframe_count > 0 && report.map_saved;
            report.reason = report.ok ? "simulation_mapping_unified" :
                (saved.ok ? "simulation_mapping_acceptance_not_met" :
                 "simulation_mapping_map_save_failed:" + saved.reason);
        } else {
            report.map_revision_before = map_revision_at_start;
            report.map_revision_after = report.core.current_map_revision;
            report.ok = report.core.initialization_complete &&
                report.core.sparse_map_loaded && report.localization_ready &&
                !report.mapping_writes_enabled &&
                report.map_cell_count > 0;
            report.reason = report.ok ? "simulation_localization_unified" :
                "simulation_localization_acceptance_not_met";
        }
        return report;
    }
};

} // namespace robot_slamd
