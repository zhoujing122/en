#pragma once

#include "robot_slamd/autonomy/autonomous_slam_coordinator.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_acceptance_runner.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_contract_checker.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_readiness_checker.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"
#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_gate.hpp"
#include "robot_slamd/motion/algorithm_motion_command_adapter.hpp"

#include <memory>
#include <string>
#include <utility>

namespace robot_slamd {

class PreLiveAutonomousSlamRunner {
public:
    PreLiveAutonomousSlamRunner(
        std::shared_ptr<RobotSlamSensorPort> sensor_port,
        std::shared_ptr<RobotSlamMotionPort> motion_port,
        std::shared_ptr<RobotSlamMapPort> map_port,
        std::shared_ptr<RobotSlamTimePort> time_port,
        RealAdapterContractChecker contract_checker = {},
        RealAdapterReadinessChecker readiness_checker = {},
        RealAdapterAcceptanceRunnerOptions acceptance_options = {},
        AutonomousSlamCoordinatorOptions coordinator_options = {},
        PreLiveIntegrationOptions prelive_options = {},
        PreLiveGateOptions gate_options = {})
        : sensor_port_(std::move(sensor_port)),
          motion_port_(std::move(motion_port)),
          map_port_(std::move(map_port)),
          time_port_(std::move(time_port)),
          contract_checker_(contract_checker),
          readiness_checker_(readiness_checker),
          acceptance_options_(acceptance_options),
          coordinator_options_(coordinator_options),
          prelive_options_(prelive_options),
          gate_options_(gate_options) {}

    PreLiveIntegrationReport run() {
        PreLiveIntegrationReport report;

        // Configuration gate.
        report.stage = PreLiveIntegrationStage::CheckingReadiness;
        if (!prelive_options_.enabled) {
            report.block_reason = PreLiveBlockReason::ConfigDisabled;
            report.failed.push_back("prelive_config_disabled");
            report.summary = "prelive_autonomous_slam_disabled";
            return report;
        }

        // Real adapter readiness check.
        report.counters.readiness_check_count++;
        const auto readiness_report = readiness_checker_.check_ports(sensor_port_.get(),
                                                                     motion_port_.get(),
                                                                     map_port_.get(),
                                                                     time_port_.get());
        report.readiness = readiness_report.readiness;
        append(report.warnings, readiness_report.checklist);
        if (!readiness_report.ok) {
            report.block_reason = PreLiveBlockReason::ReadinessFailed;
            report.failed.push_back("readiness_check_failed");
            append_violations(report.warnings, readiness_report.violations);
            report.summary = "prelive_autonomous_slam_blocked";
            return report;
        }
        report.passed.push_back("readiness_check_passed");

        // Sensor and map contract checks.
        report.stage = PreLiveIntegrationStage::CheckingContracts;
        double now_s = current_time();
        const auto snapshot = sensor_port_ ? sensor_port_->latest_snapshot(now_s)
                                           : RobotSlamSensorSnapshot{};
        report.counters.contract_check_count++;
        const auto sensor_contract = contract_checker_.check_sensor_snapshot(snapshot, now_s);
        append(report.warnings, sensor_contract.warnings);
        if (!sensor_contract.ok) {
            report.block_reason = PreLiveBlockReason::ContractFailed;
            report.failed.push_back("sensor_contract_failed");
            append_violations(report.warnings, sensor_contract.violations);
            report.summary = "prelive_autonomous_slam_blocked";
            return report;
        }
        report.passed.push_back("sensor_contract_passed");

        report.final_map_quality = map_port_ ? map_port_->latest_quality(now_s)
                                             : RobotSlamMapQuality{};
        report.counters.contract_check_count++;
        const auto map_contract = contract_checker_.check_map_quality(report.final_map_quality);
        append(report.warnings, map_contract.warnings);
        if (!map_contract.ok) {
            report.block_reason = PreLiveBlockReason::ContractFailed;
            report.failed.push_back("map_contract_failed");
            append_violations(report.warnings, map_contract.violations);
            report.summary = "prelive_autonomous_slam_blocked";
            return report;
        }
        report.passed.push_back("map_contract_passed");

        // Adapter acceptance runner.
        report.stage = PreLiveIntegrationStage::RunningAdapterAcceptance;
        if (prelive_options_.require_adapter_acceptance) {
            RealAdapterAcceptanceRunner acceptance_runner(sensor_port_,
                                                          motion_port_,
                                                          map_port_,
                                                          time_port_,
                                                          contract_checker_,
                                                          readiness_checker_,
                                                          acceptance_options_);
            const auto acceptance = acceptance_runner.run_once();
            report.counters.adapter_acceptance_run_count++;
            append(report.passed, acceptance.passed);
            append(report.failed, acceptance.failed);
            append(report.warnings, acceptance.sensor_report.warnings);
            append(report.warnings, acceptance.map_report.warnings);
            if (!acceptance.ok) {
                report.block_reason = PreLiveBlockReason::AdapterAcceptanceFailed;
                report.failed.push_back("adapter_acceptance_failed");
                report.summary = "prelive_autonomous_slam_blocked";
                return report;
            }
        }

        // Autonomous SLAM coordinator shadow loop.
        report.stage = PreLiveIntegrationStage::RunningCoordinator;
        auto coordinator_options = coordinator_options_;
        coordinator_options.enabled = true;
        coordinator_options.max_iterations = prelive_options_.max_iterations;
        AutonomousSlamCoordinator coordinator(sensor_port_,
                                              motion_port_,
                                              map_port_,
                                              coordinator_options);
        bool reached_limit = true;
        for (int i = 0; i < prelive_options_.max_iterations; ++i) {
            now_s = current_time();
            AutonomousSlamStepInput input;
            input.now_s = now_s;
            input.start_requested = i == 0;
            input.stop_requested = false;
            input.sensors = sensor_port_ ? sensor_port_->latest_snapshot(now_s)
                                         : RobotSlamSensorSnapshot{};
            input.motion_feedback = motion_port_ ? motion_port_->latest_feedback(now_s)
                                                 : RobotSlamMotionFeedback{};

            auto output = coordinator.step(input);
            report.counters.coordinator_step_count++;
            report.final_state = output.state;
            report.final_fault = output.fault;
            collect_command(output, report);

            if (output.state == AutonomousSlamState::Fault) {
                report.counters.coordinator_fault_count++;
                report.block_reason = PreLiveBlockReason::CoordinatorFault;
                report.failed.push_back("coordinator_fault");
                reached_limit = false;
                break;
            }
            if (output.completed || output.state == AutonomousSlamState::Completed) {
                reached_limit = false;
                break;
            }
            advance_time();
        }
        report.final_map_quality = map_port_ ? map_port_->latest_quality(current_time())
                                             : report.final_map_quality;
        if (reached_limit && report.final_state != AutonomousSlamState::Completed) {
            report.block_reason = PreLiveBlockReason::MaxIterationReached;
            report.failed.push_back("max_iteration_reached");
        }

        // Final pre-live gate.
        report.stage = PreLiveIntegrationStage::EvaluatingGate;
        report.ok = report.failed.empty() &&
                    report.block_reason == PreLiveBlockReason::None;
        PreLiveAutonomousSlamGate gate(gate_options_);
        const auto gate_result = gate.evaluate(report);
        append(report.passed, gate_result.passed);
        append(report.failed, gate_result.failed);
        append(report.warnings, gate_result.warnings);
        if (gate_result.pass) {
            report.ok = true;
            report.stage = PreLiveIntegrationStage::Passed;
            report.block_reason = PreLiveBlockReason::None;
            report.summary = "prelive_autonomous_slam_passed";
        } else {
            report.ok = false;
            report.stage = PreLiveIntegrationStage::Blocked;
            if (report.block_reason == PreLiveBlockReason::None) {
                report.block_reason = gate_result.block_reason;
            }
            report.summary = "prelive_autonomous_slam_blocked";
        }
        return report;
    }

private:
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

    static bool has_algorithm_command(const AutonomousSlamStepOutput &output) {
        return !output.algorithm_command.reason.empty();
    }

    static bool is_stop_command(AlgorithmMotionKind kind) {
        return kind == AlgorithmMotionKind::Stop ||
               kind == AlgorithmMotionKind::EmergencyStop;
    }

    static bool is_active_scan_command(AlgorithmMotionKind kind) {
        return kind == AlgorithmMotionKind::ActiveScanTurnLeft ||
               kind == AlgorithmMotionKind::ActiveScanTurnRight;
    }

    static bool is_forbidden_translation(AlgorithmMotionKind kind) {
        return kind == AlgorithmMotionKind::Forward ||
               kind == AlgorithmMotionKind::Backward ||
               kind == AlgorithmMotionKind::RecoveryForward ||
               kind == AlgorithmMotionKind::RecoveryBackward;
    }

    void collect_command(const AutonomousSlamStepOutput &output,
                         PreLiveIntegrationReport &report) const {
        if (!has_algorithm_command(output)) {
            return;
        }
        report.algorithm_commands.push_back(output.algorithm_command);
        if (is_stop_command(output.algorithm_command.kind)) {
            report.counters.stop_command_count++;
        } else {
            report.counters.motion_command_count++;
        }
        if (is_active_scan_command(output.algorithm_command.kind)) {
            report.counters.active_scan_command_count++;
        }
        if (is_forbidden_translation(output.algorithm_command.kind)) {
            report.failed.push_back("forward_backward_command_forbidden");
            report.block_reason = PreLiveBlockReason::SafetyGateMissing;
        }
        if (output.state == AutonomousSlamState::Fault &&
            output.fault == AutonomousSlamFault::MotionRejected) {
            report.counters.motion_reject_count++;
        }
        if (!output.software_command.reason.empty()) {
            report.software_commands.push_back(output.software_command);
            return;
        }
        AlgorithmMotionCommandAdapter adapter;
        const auto mapped = adapter.to_software_command(output.algorithm_command);
        if (mapped.ok) {
            report.software_commands.push_back(mapped.command);
        }
    }

    double current_time() const {
        return time_port_ ? time_port_->now_s() : local_now_s_;
    }

    void advance_time() const {
        if (auto fixed = dynamic_cast<FixedStepTimePort *>(time_port_.get())) {
            fixed->advance();
        } else {
            local_now_s_ += prelive_options_.step_s;
        }
    }

    std::shared_ptr<RobotSlamSensorPort> sensor_port_;
    std::shared_ptr<RobotSlamMotionPort> motion_port_;
    std::shared_ptr<RobotSlamMapPort> map_port_;
    std::shared_ptr<RobotSlamTimePort> time_port_;
    RealAdapterContractChecker contract_checker_;
    RealAdapterReadinessChecker readiness_checker_;
    RealAdapterAcceptanceRunnerOptions acceptance_options_;
    AutonomousSlamCoordinatorOptions coordinator_options_;
    PreLiveIntegrationOptions prelive_options_;
    PreLiveGateOptions gate_options_;
    mutable double local_now_s_ = prelive_options_.start_time_s;
};

} // namespace robot_slamd
