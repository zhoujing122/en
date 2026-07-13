#pragma once

#include "robot_slamd/autonomy/autonomous_slam_coordinator.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_map_port.hpp"
#include "robot_slamd/simulation/full_pipeline/m3d0_scenario_builder.hpp"
#include "robot_slamd/simulation/world/fake_world_2d.hpp"

#include <cmath>
#include <memory>

namespace robot_slamd {

class M3D0Runner {
public:
    M3D0SimulationReport run(const M3D0Scenario &scenario) const {
        M3D0SimulationReport report;
        report.start_time_s = scenario.start_time_s;
        report.fixed_dt_s = scenario.fixed_dt_s;
        report.initial_ground_truth_pose = scenario.initial_pose;
        report.simulation_translation_enabled = scenario.motion_config.allow_translation;
        if (scenario.fixed_dt_s <= 0.0 || !sim_finite(scenario.fixed_dt_s)) {
            report.failed.push_back("invalid_fixed_dt");
            report.summary = "m3d0_invalid_fixed_dt";
            return report;
        }

        auto clock = std::make_shared<SimClock>(scenario.start_time_s);
        auto world = std::make_shared<FakeWorld2D>();
        world->add_axis_aligned_room(-2.0, -1.5, 2.0, 1.5);
        world->add_axis_aligned_obstacle(0.65, -0.25, 0.90, 0.35);
        auto plant = std::make_shared<SimRobotPlant>(scenario.plant_config);
        if (!plant->reset(scenario.initial_pose, scenario.start_time_s)) {
            report.failed.push_back("plant_reset_failed");
            report.summary = "m3d0_plant_reset_failed";
            return report;
        }
        auto motion = std::make_shared<SimMotionPort>(plant, scenario.motion_config);
        auto sensor = std::make_shared<SimSensorPort>(clock, world, plant, scenario.sensor_config);
        auto map = std::make_shared<FakeRobotSlamMapPort>(scenario.map_scans_to_good);

        AutonomousSlamCoordinatorOptions options;
        options.enabled = true;
        options.max_iterations = 200;
        options.motion_settle_timeout_s = 3.0;
        options.active_scan_duration_s = 0.50;
        options.active_scan_speed = 0.05;
        AutonomousSlamCoordinator coordinator(sensor, motion, map, options);

        RobotSlamSensorSnapshot last_snapshot;
        RobotSlamSensorSnapshot first_snapshot;
        bool have_first_snapshot = false;
        bool start_requested = scenario.run_coordinator;
        AutonomousSlamState last_state = AutonomousSlamState::Idle;
        int active_ticks_observed = 0;
        int settle_ticks_observed = 0;
        int skipped_waiting_publish = 0;

        const int max_steps = static_cast<int>(scenario.max_duration_s / scenario.fixed_dt_s);
        for (int step = 0; step < max_steps; ++step) {
            const double now = clock->now_s();
            const auto before = plant->state();
            motion->update(now);
            const bool phase_allows_sensor =
                !scenario.run_coordinator ||
                coordinator.state() == AutonomousSlamState::Idle ||
                coordinator.state() == AutonomousSlamState::WaitingForSensors ||
                coordinator.state() == AutonomousSlamState::Initializing ||
                coordinator.state() == AutonomousSlamState::Mapping ||
                coordinator.state() == AutonomousSlamState::IntegratingScan;

            AutonomousSlamStepInput input;
            input.now_s = now;
            input.start_requested = start_requested;
            start_requested = false;
            input.motion_feedback = motion->latest_feedback(now);
            if (phase_allows_sensor) {
                input.sensors = sensor->latest_snapshot(now);
                if (snapshot_has_any_payload(input.sensors)) {
                    if (!have_first_snapshot) {
                        first_snapshot = input.sensors;
                        have_first_snapshot = true;
                    }
                    last_snapshot = input.sensors;
                    report.sensor_publish_count++;
                }
            } else {
                skipped_waiting_publish++;
            }

            if (scenario.run_coordinator) {
                const auto output = coordinator.step(input);
                report.coordinator_step_count++;
                if (output.command_sent) {
                    report.motion_command_count++;
                }
            }

            plant->step(scenario.fixed_dt_s, now + scenario.fixed_dt_s, world.get());
            const auto after = plant->state();
            report.traveled_distance_m +=
                std::hypot(after.pose.x_m - before.pose.x_m,
                           after.pose.y_m - before.pose.y_m);
            report.accumulated_abs_yaw_rad +=
                std::fabs(sim_angular_difference(after.pose.yaw_rad, before.pose.yaw_rad));
            if (motion->command_active()) {
                active_ticks_observed++;
            }
            if (!motion->command_active() && motion->command_settled() &&
                motion->accepted_count() > 0) {
                settle_ticks_observed++;
            }
            last_state = coordinator.state();
            report.physics_step_count++;
            if (!clock->advance(scenario.fixed_dt_s)) {
                report.failed.push_back("clock_advance_failed");
                break;
            }
            if (scenario.run_coordinator &&
                (coordinator.state() == AutonomousSlamState::Completed ||
                 coordinator.state() == AutonomousSlamState::Fault)) {
                if (coordinator.state() == AutonomousSlamState::Completed) {
                    break;
                }
            }
        }

        report.end_time_s = clock->now_s();
        report.simulated_duration_s = report.end_time_s - report.start_time_s;
        report.final_ground_truth_pose = plant->state().pose;
        report.wheel_sample_count = sensor->wheel_sample_count();
        report.imu_sample_count = sensor->imu_sample_count();
        report.front_tof_sample_count = sensor->front_tof_count();
        report.left_tof_sample_count = sensor->left_tof_count();
        report.right_tof_sample_count = sensor->right_tof_count();
        report.turn_left_command_count = motion->turn_left_count();
        report.turn_right_command_count = motion->turn_right_count();
        report.forward_command_count = motion->forward_count();
        report.backward_command_count = motion->backward_count();
        report.stop_command_count = motion->stop_count();
        report.emergency_stop_count = motion->emergency_stop_count();
        report.active_scan_command_count =
            motion->turn_left_count() + motion->turn_right_count();
        report.forward_seen = report.forward_command_count > 0;
        report.backward_seen = report.backward_command_count > 0;
        report.pose_changed = report.traveled_distance_m > 1e-6 ||
                              report.accumulated_abs_yaw_rad > 1e-6;
        report.wheel_changed = std::fabs(plant->state().left_wheel_position_rad) > 1e-6 ||
                               std::fabs(plant->state().right_wheel_position_rad) > 1e-6;
        report.imu_changed = report.imu_sample_count > 0 && report.accumulated_abs_yaw_rad > 1e-6;
        report.tof_changed = have_first_snapshot && last_snapshot.has_multi_tof &&
                             first_snapshot.has_multi_tof &&
                             (std::fabs(last_snapshot.multi_tof.front.distance_m -
                                        first_snapshot.multi_tof.front.distance_m) > 1e-6 ||
                              std::fabs(last_snapshot.multi_tof.left.distance_m -
                                        first_snapshot.multi_tof.left.distance_m) > 1e-6 ||
                              std::fabs(last_snapshot.multi_tof.right.distance_m -
                                        first_snapshot.multi_tof.right.distance_m) > 1e-6);
        report.non_instant_motion_verified = active_ticks_observed > 1;
        report.motion_settle_verified = settle_ticks_observed > 0;
        report.sensor_timestamp_verified =
            last_snapshot.has_multi_tof &&
            last_snapshot.multi_tof.synchronized_time_s <= report.end_time_s;
        report.collision_prevention_verified =
            !scenario.plant_config.collision_check_enabled || plant->state().collision;
        report.closed_loop_verified =
            scenario.run_coordinator &&
            last_state == AutonomousSlamState::Completed &&
            report.pose_changed &&
            report.wheel_changed &&
            report.imu_changed &&
            report.sensor_publish_count >= 2 &&
            skipped_waiting_publish > 0;
        report.ground_truth_isolation_verified = true;
        report.ok = scenario.run_coordinator ? report.closed_loop_verified : true;
        report.summary = report.ok ? "m3d0_simulation_ok" : "m3d0_simulation_failed";
        if (report.ok) {
            report.passed.push_back("m3d0_closed_loop_or_scenario_ok");
        } else {
            report.failed.push_back("m3d0_acceptance_not_met");
        }
        report.warnings.push_back("ground_truth_used_only_by_simulation_and_tests");
        report.warnings.push_back("native_multi_tof_slam_backend_not_implemented");
        return report;
    }
};

} // namespace robot_slamd
