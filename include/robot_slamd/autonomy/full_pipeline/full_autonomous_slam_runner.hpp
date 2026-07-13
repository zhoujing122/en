#pragma once

#include "robot_slamd/autonomy/autonomous_slam_coordinator.hpp"
#include "robot_slamd/autonomy/fake_map/fake_map_storage.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_fake_motion_port.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_scenario_builder.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
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
        return run_once_with_sensor(scenario, replay);
    }

    FullAutonomousSlamPipelineReport run_once_with_sensor(
        const FullAutonomousSlamScenario &scenario,
        std::shared_ptr<RobotSlamSensorPort> sensor_port) const {
        FullAutonomousSlamPipelineReport report;
        report.stage = FullAutonomousSlamPipelineStage::BuildingScenario;
        report.scenario = scenario.kind;
        auto options = effective_options(scenario);
        if (!options.enabled) {
            return fail(report,
                        FullAutonomousSlamPipelineFault::ScenarioInvalid,
                        "full_pipeline_disabled");
        }

        if (!sensor_port || !sensor_port->ready()) {
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
        coordinator_options.max_active_scan_commands =
            options.max_active_scan_commands;
        AutonomousSlamCoordinator coordinator(
            sensor_port,
            motion_port,
            map_port,
            coordinator_options);

        RobotSlamSensorSnapshot last_snapshot;
        bool has_last_snapshot = false;

        for (int step = 0; step < options.max_steps; ++step) {
            report.step_count++;
            const double now_s = 100.0 + 0.1 * static_cast<double>(step);
            const auto state_before = coordinator.state();
            append_trace(report,
                         FullAutonomousSlamTraceEventKind::StepStarted,
                         step,
                         now_s,
                         state_before,
                         "step_started");

            RobotSlamSensorSnapshot step_snapshot;
            const bool consume_sensor =
                !options.phase_aware_sensor_consumption ||
                should_consume_sensor_for_state(state_before, step);
            if (consume_sensor) {
                report.stage = FullAutonomousSlamPipelineStage::RunningSensorReplay;
                step_snapshot = sensor_port->latest_snapshot(now_s);
                if (!snapshot_has_any_payload(step_snapshot)) {
                    return fail(report,
                                FullAutonomousSlamPipelineFault::SensorSnapshotMissing,
                                "full_pipeline_sensor_snapshot_missing",
                                step,
                                now_s,
                                state_before);
                }
                last_snapshot = step_snapshot;
                has_last_snapshot = true;
                report.sensor_snapshot_count++;
                report.sensor_consumed_count++;
                record_sensor_payload(report, step_snapshot);
                append_trace(report,
                             FullAutonomousSlamTraceEventKind::SensorConsumed,
                             step,
                             now_s,
                             state_before,
                             "sensor_consumed");
            } else {
                report.sensor_skipped_count++;
                append_trace(report,
                             FullAutonomousSlamTraceEventKind::SensorSkipped,
                             step,
                             now_s,
                             state_before,
                             "sensor_skipped");
            }

            AutonomousSlamStepInput input;
            input.now_s = now_s;
            input.start_requested = step == 0;
            input.sensors = consume_sensor ? step_snapshot : RobotSlamSensorSnapshot{};
            input.motion_feedback = motion_port->latest_feedback(now_s);

            const auto backend_before = backend->state();
            report.stage = FullAutonomousSlamPipelineStage::UpdatingCoordinator;
            const auto output = coordinator.step(input);
            const auto backend_after = backend->state();
            report.coordinator_step_count++;
            report.final_state = output.state;
            report.final_quality = backend->latest_quality(now_s);
            update_backend_counts(report, backend_after);
            update_motion_counts(report, *motion_port);
            append_trace(report,
                         FullAutonomousSlamTraceEventKind::CoordinatorUpdated,
                         step,
                         now_s,
                         output.state,
                         output.message);
            append_backend_trace(report,
                                 backend_before,
                                 backend_after,
                                 step,
                                 now_s,
                                 output.state);
            append_quality_trace(report, step, now_s, output.state);

            if (output.state == AutonomousSlamState::SendingMotionCommand ||
                output.algorithm_command.kind == AlgorithmMotionKind::ActiveScanTurnLeft ||
                output.algorithm_command.kind == AlgorithmMotionKind::ActiveScanTurnRight) {
                report.stage = FullAutonomousSlamPipelineStage::SendingShadowMotion;
            } else if (output.state == AutonomousSlamState::WaitingMotionSettle) {
                report.stage = FullAutonomousSlamPipelineStage::WaitingMotionSettle;
            } else if (output.state == AutonomousSlamState::Mapping) {
                report.stage = FullAutonomousSlamPipelineStage::UpdatingSlamBackend;
            }

            if (output.command_sent) {
                append_trace(report,
                             FullAutonomousSlamTraceEventKind::MotionCommandSent,
                             step,
                             now_s,
                             output.state,
                             "motion_command_sent");
            }

            if (motion_port->saw_forward_or_backward() &&
                options.require_no_forward_backward) {
                return fail(report,
                            FullAutonomousSlamPipelineFault::MotionRejected,
                            "full_pipeline_forward_backward_seen",
                            step,
                            now_s,
                            output.state);
            }

            if (output.state == AutonomousSlamState::Fault) {
                const auto fault = output.fault == AutonomousSlamFault::MotionRejected
                                       ? FullAutonomousSlamPipelineFault::MotionRejected
                                       : FullAutonomousSlamPipelineFault::CoordinatorFault;
                if (fault == FullAutonomousSlamPipelineFault::MotionRejected) {
                    append_trace(report,
                                 FullAutonomousSlamTraceEventKind::MotionRejected,
                                 step,
                                 now_s,
                                 output.state,
                                 output.message);
                }
                return fail(report, fault, output.message, step, now_s, output.state);
            }

            if (report.backend_rejected_update_count > 0) {
                return fail(report,
                            FullAutonomousSlamPipelineFault::BackendRejected,
                            "full_pipeline_backend_rejected",
                            step,
                            now_s,
                            output.state);
            }

            if (report.active_scan_command_count > options.max_active_scan_commands) {
                return fail(report,
                            FullAutonomousSlamPipelineFault::MapQualityStuck,
                            "full_pipeline_active_scan_limit_exceeded",
                            step,
                            now_s,
                            output.state);
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
                                "full_pipeline_completed_but_quality_not_good",
                                step,
                                now_s,
                                output.state);
                }
                if (report.backend_accepted_update_count <
                    options.min_backend_accepted_updates) {
                    return fail(report,
                                FullAutonomousSlamPipelineFault::MapQualityStuck,
                                "full_pipeline_too_few_backend_updates",
                                step,
                                now_s,
                                output.state);
                }
                report.ok = true;
                report.fault = FullAutonomousSlamPipelineFault::None;
                report.passed.push_back("full_pipeline_completed");
                report.passed.push_back("full_pipeline_map_quality_good");
                report.passed.push_back("full_pipeline_shadow_motion_only");
                if (options.phase_aware_sensor_consumption) {
                    report.passed.push_back("full_pipeline_phase_aware_sensor_consumption");
                }
                auto map_result = build_and_save_fake_map(report, options, now_s);
                if (!map_result.ok) {
                    const auto fault = map_result.artifact.fault == FakeMapArtifactFault::SaveDisabled ||
                                               map_result.summary == "fake_map_save_disabled" ||
                                               map_result.summary == "fake_map_duplicate_artifact"
                                           ? FullAutonomousSlamPipelineFault::FakeMapSaveFailed
                                           : FullAutonomousSlamPipelineFault::FakeMapBuildFailed;
                    return fail(report,
                                fault,
                                map_result.summary,
                                step,
                                now_s,
                                output.state);
                }
                append_trace(report,
                             FullAutonomousSlamTraceEventKind::Completed,
                             step,
                             now_s,
                             output.state,
                             "full_pipeline_completed");
                report.summary = "full_autonomous_slam_pipeline_passed";
                return report;
            }
        }

        update_backend_counts(report, backend->state());
        update_motion_counts(report, *motion_port);
        report.final_quality = backend->latest_quality(100.0);
        (void)last_snapshot;
        (void)has_last_snapshot;
        return fail(report,
                    FullAutonomousSlamPipelineFault::MaxStepsExceeded,
                    "full_pipeline_max_steps_exceeded",
                    options.max_steps,
                    100.0,
                    coordinator.state());
    }

private:
    FullAutonomousSlamPipelineOptions effective_options(
        const FullAutonomousSlamScenario &scenario) const {
        if (options_.enabled) {
            return options_;
        }
        return scenario.pipeline_options;
    }

    bool should_consume_sensor_for_state(
        AutonomousSlamState state,
        int step) const {
        if (step == 0) {
            return true;
        }
        switch (state) {
        case AutonomousSlamState::Idle:
        case AutonomousSlamState::WaitingForSensors:
        case AutonomousSlamState::Initializing:
        case AutonomousSlamState::Mapping:
        case AutonomousSlamState::IntegratingScan:
            return true;
        case AutonomousSlamState::NeedActiveScan:
        case AutonomousSlamState::SendingMotionCommand:
        case AutonomousSlamState::WaitingMotionSettle:
        case AutonomousSlamState::Completed:
        case AutonomousSlamState::Fault:
            return false;
        }
        return false;
    }

    static void record_sensor_payload(
        FullAutonomousSlamPipelineReport &report,
        const RobotSlamSensorSnapshot &snapshot) {
        if (snapshot.has_tof) {
            report.legacy_tof_snapshot_count++;
            if (snapshot.tof.ranges_m.size() == 1 &&
                snapshot.tof.source.find("legacy_scalar_tof_projection") !=
                    std::string::npos) {
                report.saw_legacy_scalar_projection = true;
            }
        }
        if (!snapshot.has_multi_tof) {
            return;
        }
        report.multi_tof_snapshot_count++;
        report.saw_multi_tof_snapshot = true;
        if (snapshot.multi_tof.has_front) {
            report.snapshots_with_front++;
        }
        if (snapshot.multi_tof.has_left) {
            report.snapshots_with_left++;
        }
        if (snapshot.multi_tof.has_right) {
            report.snapshots_with_right++;
        }
        if (snapshot.multi_tof.degraded) {
            report.degraded_multi_tof_snapshot_count++;
        }
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

    static void append_trace(
        FullAutonomousSlamPipelineReport &report,
        FullAutonomousSlamTraceEventKind kind,
        int step,
        double now_s,
        AutonomousSlamState state,
        const std::string &message) {
        FullAutonomousSlamTraceEvent event;
        event.kind = kind;
        event.step_index = step;
        event.now_s = now_s;
        event.state = state;
        event.sensor_consumed = kind == FullAutonomousSlamTraceEventKind::SensorConsumed;
        event.backend_updated = kind == FullAutonomousSlamTraceEventKind::BackendUpdated ||
                                kind == FullAutonomousSlamTraceEventKind::BackendRejected;
        event.motion_command_sent = kind == FullAutonomousSlamTraceEventKind::MotionCommandSent;
        event.map_quality_good = kind == FullAutonomousSlamTraceEventKind::MapQualityGood;
        event.message = message;
        append_full_autonomous_slam_trace_event(report.trace, event);
        report.sensor_consumed_count = report.trace.sensor_consumed_count;
        report.sensor_skipped_count = report.trace.sensor_skipped_count;
    }

    static void append_backend_trace(
        FullAutonomousSlamPipelineReport &report,
        const DeterministicSlamBackendState &before,
        const DeterministicSlamBackendState &after,
        int step,
        double now_s,
        AutonomousSlamState state) {
        if (after.update_call_count <= before.update_call_count) {
            return;
        }
        const auto kind = after.rejected_update_count > before.rejected_update_count
                              ? FullAutonomousSlamTraceEventKind::BackendRejected
                              : FullAutonomousSlamTraceEventKind::BackendUpdated;
        append_trace(report, kind, step, now_s, state, to_string(kind));
    }

    static void append_quality_trace(
        FullAutonomousSlamPipelineReport &report,
        int step,
        double now_s,
        AutonomousSlamState state) {
        if (report.backend_update_count <= 0) {
            return;
        }
        append_trace(report,
                     report.final_quality.good_enough
                         ? FullAutonomousSlamTraceEventKind::MapQualityGood
                         : FullAutonomousSlamTraceEventKind::MapQualityPoor,
                     step,
                     now_s,
                     state,
                     report.final_quality.reason);
    }

    static FakeMapStorageResult build_and_save_fake_map(
        FullAutonomousSlamPipelineReport &report,
        const FullAutonomousSlamPipelineOptions &options,
        double now_s) {
        FakeMapStorageResult result;
        if (!options.build_fake_map_on_completed) {
            result.ok = true;
            return result;
        }

        FakeMapSaveOptions save_options;
        save_options.enabled = options.save_fake_map_on_completed;
        save_options.allow_overwrite = false;
        save_options.require_quality_good = true;
        save_options.require_completed_pipeline = true;
        FakeMapLoadOptions load_options;
        load_options.enabled = true;
        FakeMapStorage storage(save_options, load_options);
        const std::string map_id = options.fake_map_id_prefix + "_" +
                                   to_string(report.scenario);
        auto build = storage.build_from_pipeline_report(report, map_id, now_s);
        if (!build.ok) {
            return build;
        }
        report.fake_map_built = true;
        report.fake_map_artifact = build.artifact;
        report.passed.push_back("full_pipeline_fake_map_built");
        append_trace(report,
                     FullAutonomousSlamTraceEventKind::FakeMapBuilt,
                     report.step_count,
                     now_s,
                     report.final_state,
                     "fake_map_built");

        if (!options.save_fake_map_on_completed) {
            if (options.require_fake_map_saved) {
                result.ok = false;
                result.artifact = report.fake_map_artifact;
                result.artifact.status = FakeMapArtifactStatus::Rejected;
                result.artifact.fault = FakeMapArtifactFault::SaveDisabled;
                result.summary = "fake_map_save_required_but_disabled";
                return result;
            }
            result.ok = true;
            result.artifact = report.fake_map_artifact;
            result.summary = "fake_map_save_skipped";
            return result;
        }
        auto save = storage.save_artifact(report.fake_map_artifact);
        if (!save.ok) {
            return save;
        }
        report.fake_map_saved = true;
        report.fake_map_artifact = save.artifact;
        report.passed.push_back("full_pipeline_fake_map_saved");
        append_trace(report,
                     FullAutonomousSlamTraceEventKind::FakeMapSaved,
                     report.step_count,
                     now_s,
                     report.final_state,
                     "fake_map_saved");
        result.ok = true;
        result.artifact = report.fake_map_artifact;
        result.summary = "fake_map_artifact_saved";
        return result;
    }

    static FullAutonomousSlamPipelineReport fail(
        FullAutonomousSlamPipelineReport report,
        FullAutonomousSlamPipelineFault fault,
        const std::string &message,
        int step = 0,
        double now_s = 0.0,
        AutonomousSlamState state = AutonomousSlamState::Fault) {
        report.ok = false;
        report.stage = FullAutonomousSlamPipelineStage::Fault;
        report.fault = fault;
        report.failed.push_back(message);
        append_trace(report,
                     FullAutonomousSlamTraceEventKind::Fault,
                     step,
                     now_s,
                     state,
                     message);
        report.summary = "full_autonomous_slam_pipeline_failed";
        return report;
    }

    FullAutonomousSlamPipelineOptions options_;
};

} // namespace robot_slamd
