#pragma once

#include "robot_slamd/autonomy/adapters/replay_robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"
#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_scenario_builder.hpp"
#include "robot_slamd/autonomy/map_backend/replay_slam_backend_binding.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"
#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_runner.hpp"

#include <memory>
#include <utility>

namespace robot_slamd {

class AutonomousSlamE2EPreLiveRunner {
public:
    explicit AutonomousSlamE2EPreLiveRunner(
        AutonomousSlamE2EScenarioOptions options = {})
        : options_(options) {}

    AutonomousSlamE2EScenarioReport run() {
        AutonomousSlamE2EScenarioReport report;
        report.scenario_kind = options_.scenario_kind;

        // Configuration gate.
        if (!options_.enabled) {
            block(report,
                  AutonomousSlamE2EBlockReason::ConfigDisabled,
                  "e2e_config_disabled");
            report.summary = "autonomous_slam_e2e_prelive_blocked";
            return report;
        }

        // Scenario construction.
        report.stage = AutonomousSlamE2EStage::BuildingScenario;
        AutonomousSlamE2EScenarioBuilder builder;
        const auto data = builder.build(options_.scenario_kind, options_.start_time_s);
        if (data.sensor_snapshots.empty() ||
            data.backend_update_results.empty() ||
            data.backend_qualities.empty()) {
            block(report,
                  AutonomousSlamE2EBlockReason::ScenarioBuildFailed,
                  "scenario_build_failed");
            return finish_blocked(report);
        }
        report.passed.push_back("scenario_build_passed");

        auto sensor_port = std::make_shared<ReplayRobotSlamSensorPort>(
            data.sensor_snapshots,
            data.sensor_ready);
        auto motion_port = std::make_shared<FakeRobotSlamMotionPort>();
        motion_port->ready_flag = data.motion_ready;
        motion_port->reject_next_command = data.reject_motion;
        motion_port->feedback.fault = data.fail_motion_feedback;
        motion_port->feedback.fault_reason =
            data.fail_motion_feedback ? "fake_motion_feedback_fault" : "";
        auto backend = std::make_shared<ReplaySlamBackendBinding>(
            data.backend_update_results,
            data.backend_qualities,
            data.map_ready);
        auto map_port = std::make_shared<SlamBackendMapPortAdapter>(backend);
        auto time_port = std::make_shared<FixedStepTimePort>(
            options_.start_time_s,
            options_.step_s);

        // Real adapter contract check for replayed adapter output.
        report.stage = AutonomousSlamE2EStage::CheckingRealAdapterContracts;
        RealAdapterContractChecker real_adapter_checker;
        const auto first_snapshot = data.sensor_snapshots.front();
        const auto sensor_report =
            real_adapter_checker.check_sensor_snapshot(first_snapshot,
                                                       options_.start_time_s);
        append(report.warnings, sensor_report.warnings);
        if (!sensor_report.ok) {
            append_violations(report.warnings, sensor_report.violations);
            block(report,
                  AutonomousSlamE2EBlockReason::RealAdapterContractFailed,
                  "real_adapter_contract_failed");
            return finish_blocked(report);
        }
        report.passed.push_back("real_adapter_contract_passed");

        // SLAM backend acceptance check.
        report.stage = AutonomousSlamE2EStage::CheckingSlamBackend;
        if (options_.require_slam_backend_acceptance) {
            SlamBackendAcceptanceRunner acceptance_runner(backend);
            report.slam_backend_acceptance =
                acceptance_runner.run_once(first_snapshot);
            append(report.passed, report.slam_backend_acceptance.passed);
            append(report.failed, report.slam_backend_acceptance.failed);
            if (!report.slam_backend_acceptance.ok) {
                block(report,
                      AutonomousSlamE2EBlockReason::SlamBackendAcceptanceFailed,
                      "slam_backend_acceptance_failed");
                return finish_blocked(report);
            }
        }

        // Pre-live integration runner over the SLAM map-port adapter.
        report.stage = AutonomousSlamE2EStage::RunningPreLive;
        PreLiveAutonomousSlamRunner prelive_runner(sensor_port,
                                                   motion_port,
                                                   map_port,
                                                   time_port,
                                                   {},
                                                   {},
                                                   make_acceptance_options(),
                                                   make_coordinator_options(),
                                                   make_prelive_options(),
                                                   make_gate_options());
        report.prelive_report = prelive_runner.run();
        append(report.warnings, report.prelive_report.warnings);
        if (data.reject_motion && report.prelive_report.counters.motion_reject_count > 0) {
            block(report,
                  AutonomousSlamE2EBlockReason::MotionRejected,
                  "motion_rejected");
            return finish_blocked(report);
        }

        // End-to-end scenario gate.
        report.stage = AutonomousSlamE2EStage::EvaluatingScenario;
        if (options_.require_prelive_pass && !report.prelive_report.ok) {
            block(report,
                  AutonomousSlamE2EBlockReason::PreLiveGateBlocked,
                  "prelive_runner_failed");
        }
        if (options_.require_no_forward_backward &&
            contains_forward_backward(report.prelive_report)) {
            block(report,
                  AutonomousSlamE2EBlockReason::ForwardBackwardCommandObserved,
                  "forward_backward_command_observed");
        }
        if (options_.require_stop_command_seen &&
            report.prelive_report.counters.stop_command_count <= 0) {
            block(report,
                  AutonomousSlamE2EBlockReason::ReportInvalid,
                  "stop_command_not_seen");
        }
        if (options_.scenario_kind ==
                AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood &&
            options_.require_active_scan_when_map_poor &&
            report.prelive_report.counters.active_scan_command_count <= 0) {
            block(report,
                  AutonomousSlamE2EBlockReason::MapQualityNeverGood,
                  "active_scan_command_not_seen");
        }
        if (options_.require_map_quality_good_at_end &&
            !report.prelive_report.final_map_quality.good_enough) {
            block(report,
                  AutonomousSlamE2EBlockReason::MapQualityNeverGood,
                  "map_quality_not_good_at_end");
        }
        if (!report.failed.empty()) {
            return finish_blocked(report);
        }

        report.ok = true;
        report.stage = AutonomousSlamE2EStage::Passed;
        report.block_reason = AutonomousSlamE2EBlockReason::None;
        report.passed.push_back("e2e_prelive_gate_passed");
        report.summary = "autonomous_slam_e2e_prelive_passed";
        return report;
    }

private:
    RealAdapterAcceptanceRunnerOptions make_acceptance_options() const {
        RealAdapterAcceptanceRunnerOptions options;
        options.start_time_s = options_.start_time_s;
        return options;
    }

    AutonomousSlamCoordinatorOptions make_coordinator_options() const {
        AutonomousSlamCoordinatorOptions options;
        options.enabled = true;
        options.max_iterations = options_.max_iterations;
        return options;
    }

    PreLiveIntegrationOptions make_prelive_options() const {
        PreLiveIntegrationOptions options;
        options.enabled = true;
        options.max_iterations = options_.max_iterations;
        options.start_time_s = options_.start_time_s;
        options.step_s = options_.step_s;
        options.require_adapter_acceptance = true;
        options.require_stop_command_seen = options_.require_stop_command_seen;
        options.require_active_scan_command_seen = false;
        options.require_map_quality_good = false;
        return options;
    }

    PreLiveGateOptions make_gate_options() const {
        PreLiveGateOptions options;
        options.require_adapter_acceptance = true;
        options.require_stop_command_seen = options_.require_stop_command_seen;
        options.require_active_scan_command_seen = false;
        options.require_map_quality_good = false;
        options.allow_coordinator_incomplete_for_shadow = true;
        return options;
    }

    static void append(std::vector<std::string> &dst,
                       const std::vector<std::string> &src) {
        dst.insert(dst.end(), src.begin(), src.end());
    }

    static void append_violations(
        std::vector<std::string> &dst,
        const std::vector<RealAdapterContractViolation> &violations) {
        for (const auto &violation : violations) {
            dst.push_back(violation.code);
        }
    }

    static bool contains_forward_backward(const PreLiveIntegrationReport &report) {
        for (const auto &command : report.algorithm_commands) {
            if (command.kind == AlgorithmMotionKind::Forward ||
                command.kind == AlgorithmMotionKind::Backward ||
                command.kind == AlgorithmMotionKind::RecoveryForward ||
                command.kind == AlgorithmMotionKind::RecoveryBackward) {
                return true;
            }
        }
        return false;
    }

    static void block(AutonomousSlamE2EScenarioReport &report,
                      AutonomousSlamE2EBlockReason reason,
                      const std::string &case_name) {
        report.ok = false;
        report.stage = AutonomousSlamE2EStage::Blocked;
        if (report.block_reason == AutonomousSlamE2EBlockReason::None) {
            report.block_reason = reason;
        }
        report.failed.push_back(case_name);
    }

    static AutonomousSlamE2EScenarioReport finish_blocked(
        AutonomousSlamE2EScenarioReport report) {
        report.ok = false;
        report.stage = AutonomousSlamE2EStage::Blocked;
        report.summary = "autonomous_slam_e2e_prelive_blocked";
        return report;
    }

    AutonomousSlamE2EScenarioOptions options_;
};

} // namespace robot_slamd
