#pragma once

#include "robot_slamd/autonomy/autonomous_slam_coordinator.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_fake_motion_port.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_scenario_builder.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_binding.hpp"

#include <memory>
#include <string>

namespace robot_slamd {

class FullAutonomousSlamRunner {
public:
    explicit FullAutonomousSlamRunner(
        FullAutonomousSlamPipelineOptions options = {})
        : options_(options) {}

    FullAutonomousSlamPipelineReport run_once(
        const FullAutonomousSlamScenario &scenario) const {
        FullAutonomousSlamPipelineReport report;
        report.stage = FullAutonomousSlamPipelineStage::BuildingScenario;
        report.scenario = scenario.kind;
        auto options = effective_options(scenario);
        if (!options.enabled) {
            return fail(report,
                        FullAutonomousSlamPipelineFault::ScenarioInvalid,
                        "full_pipeline_disabled");
        }

        RealSensorReplayOptions replay_options;
        replay_options.enabled = true;
        replay_options.loop = false;
        replay_options.fail_on_contract_error = true;
        replay_options.require_non_empty_log = true;
        replay_options.time_mode = RealSensorReplayTimeMode::RecordPacketTime;
        replay_options.reject_invalid_records = true;
        replay_options.require_packet_records = true;
        replay_options.preserve_parse_errors = true;
        replay_options.max_records_per_run = 10000;
        auto replay = std::make_shared<RealSensorReplayPort>(
            scenario.replay_log,
            RealSensorSnapshotBuilder{},
            replay_options);
        if (!replay->ready()) {
            return fail(report,
                        FullAutonomousSlamPipelineFault::SensorReplayNotReady,
                        "full_pipeline_sensor_replay_not_ready");
        }

        auto backend = std::make_shared<DeterministicSlamBackendBinding>(
            scenario.backend_options);
        if (!backend->ready()) {
            return fail(report,
                        FullAutonomousSlamPipelineFault::BackendNotReady,
                        "full_pipeline_backend_not_ready");
        }
        auto map_port = std::make_shared<SlamBackendMapPortAdapter>(backend);
        auto motion_port = std::make_shared<FullAutonomousSlamFakeMotionPort>(
            scenario.reject_motion);

        auto coordinator_options = scenario.coordinator_options;
        coordinator_options.enabled = true;
        coordinator_options.max_iterations = options.max_steps + 5;
        coordinator_options.max_active_scan_commands = options.max_active_scan_commands;
        AutonomousSlamCoordinator coordinator(
            replay,
            motion_port,
            map_port,
            coordinator_options);

        for (int step = 0; step < options.max_steps; ++step) {
            report.step_count++;
            const double now_s = 100.0 + 0.1 * static_cast<double>(step);
            report.stage = FullAutonomousSlamPipelineStage::RunningSensorReplay;
            auto snapshot = replay->latest_snapshot(now_s);
            if (!snapshot.has_tof && !snapshot.has_imu && !snapshot.has_wheel) {
                return fail(report,
                            FullAutonomousSlamPipelineFault::SensorSnapshotMissing,
                            "full_pipeline_sensor_snapshot_missing");
            }
            report.sensor_snapshot_count++;

            AutonomousSlamStepInput input;
            input.now_s = now_s;
            input.start_requested = step == 0;
            input.sensors = snapshot;
            input.motion_feedback = motion_port->latest_feedback(now_s);

            report.stage = FullAutonomousSlamPipelineStage::UpdatingCoordinator;
            const auto output = coordinator.step(input);
            report.coordinator_step_count++;
            report.final_state = output.state;
            report.final_quality = backend->latest_quality(now_s);
            update_backend_counts(report, backend->state());
            update_motion_counts(report, *motion_port);

            if (output.state == AutonomousSlamState::SendingMotionCommand ||
                output.algorithm_command.kind == AlgorithmMotionKind::ActiveScanTurnLeft ||
                output.algorithm_command.kind == AlgorithmMotionKind::ActiveScanTurnRight) {
                report.stage = FullAutonomousSlamPipelineStage::SendingShadowMotion;
            } else if (output.state == AutonomousSlamState::WaitingMotionSettle) {
                report.stage = FullAutonomousSlamPipelineStage::WaitingMotionSettle;
            } else if (output.state == AutonomousSlamState::Mapping) {
                report.stage = FullAutonomousSlamPipelineStage::UpdatingSlamBackend;
            }

            if (motion_port->saw_forward_or_backward() &&
                options.require_no_forward_backward) {
                return fail(report,
                            FullAutonomousSlamPipelineFault::MotionRejected,
                            "full_pipeline_forward_backward_seen");
            }

            if (output.state == AutonomousSlamState::Fault) {
                const auto fault = output.fault == AutonomousSlamFault::MotionRejected
                                       ? FullAutonomousSlamPipelineFault::MotionRejected
                                       : FullAutonomousSlamPipelineFault::CoordinatorFault;
                return fail(report, fault, output.message);
            }

            if (report.backend_rejected_update_count > 0) {
                return fail(report,
                            FullAutonomousSlamPipelineFault::BackendRejected,
                            "full_pipeline_backend_rejected");
            }

            if (report.active_scan_command_count > options.max_active_scan_commands) {
                return fail(report,
                            FullAutonomousSlamPipelineFault::MapQualityStuck,
                            "full_pipeline_active_scan_limit_exceeded");
            }

            if (output.completed || coordinator.state() == AutonomousSlamState::Completed) {
                report.stage = FullAutonomousSlamPipelineStage::Completed;
                report.final_state = AutonomousSlamState::Completed;
                report.final_quality = backend->latest_quality(now_s);
                update_backend_counts(report, backend->state());
                update_motion_counts(report, *motion_port);
                if (options.require_map_quality_good &&
                    !report.final_quality.good_enough) {
                    return fail(report,
                                FullAutonomousSlamPipelineFault::CompletedButQualityNotGood,
                                "full_pipeline_completed_but_quality_not_good");
                }
                if (report.backend_accepted_update_count <
                    options.min_backend_accepted_updates) {
                    return fail(report,
                                FullAutonomousSlamPipelineFault::MapQualityStuck,
                                "full_pipeline_too_few_backend_updates");
                }
                report.ok = true;
                report.fault = FullAutonomousSlamPipelineFault::None;
                report.passed.push_back("full_pipeline_completed");
                report.passed.push_back("full_pipeline_map_quality_good");
                report.passed.push_back("full_pipeline_shadow_motion_only");
                report.summary = "full_autonomous_slam_pipeline_passed";
                return report;
            }
        }

        update_backend_counts(report, backend->state());
        update_motion_counts(report, *motion_port);
        report.final_quality = backend->latest_quality(100.0);
        return fail(report,
                    FullAutonomousSlamPipelineFault::MaxStepsExceeded,
                    "full_pipeline_max_steps_exceeded");
    }

private:
    FullAutonomousSlamPipelineOptions effective_options(
        const FullAutonomousSlamScenario &scenario) const {
        if (options_.enabled) {
            return options_;
        }
        return scenario.pipeline_options;
    }

    static void update_backend_counts(
        FullAutonomousSlamPipelineReport &report,
        const DeterministicSlamBackendState &state) {
        report.backend_update_count = state.update_call_count;
        report.backend_accepted_update_count = state.accepted_update_count;
        report.backend_rejected_update_count = state.rejected_update_count;
    }

    static void update_motion_counts(
        FullAutonomousSlamPipelineReport &report,
        const FullAutonomousSlamFakeMotionPort &motion) {
        report.shadow_motion_command_count = motion.command_count();
        report.active_scan_command_count = motion.active_scan_command_count();
        report.stop_command_count = motion.stop_command_count();
    }

    static FullAutonomousSlamPipelineReport fail(
        FullAutonomousSlamPipelineReport report,
        FullAutonomousSlamPipelineFault fault,
        const std::string &message) {
        report.ok = false;
        report.stage = FullAutonomousSlamPipelineStage::Fault;
        report.fault = fault;
        report.failed.push_back(message);
        report.summary = "full_autonomous_slam_pipeline_failed";
        return report;
    }

    FullAutonomousSlamPipelineOptions options_;
};

} // namespace robot_slamd
