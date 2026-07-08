#pragma once

#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct PreLiveGateOptions {
    bool require_adapter_acceptance = true;
    bool require_no_failed_cases = true;
    bool require_no_motion_rejection = true;
    bool require_stop_command_seen = true;
    bool require_active_scan_command_seen = false;
    bool require_map_quality_good = false;
    bool allow_coordinator_incomplete_for_shadow = true;
};

struct PreLiveGateResult {
    bool pass = false;
    PreLiveBlockReason block_reason = PreLiveBlockReason::None;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
};

class PreLiveAutonomousSlamGate {
public:
    explicit PreLiveAutonomousSlamGate(PreLiveGateOptions options = {})
        : options_(options) {}

    PreLiveGateResult evaluate(const PreLiveIntegrationReport &report) const {
        PreLiveGateResult result;

        // report and adapter acceptance validation
        if (!report.ok && report.block_reason != PreLiveBlockReason::None) {
            fail(result, "report_not_ok", report.block_reason);
        } else {
            pass(result, "report_status_checked");
        }
        if (options_.require_adapter_acceptance &&
            report.counters.adapter_acceptance_run_count <= 0) {
            fail(result,
                 "adapter_acceptance_not_run",
                 PreLiveBlockReason::AdapterAcceptanceFailed);
        } else {
            pass(result, "adapter_acceptance_checked");
        }
        if (options_.require_no_failed_cases && !report.failed.empty()) {
            fail(result, "prelive_failed_cases_present", PreLiveBlockReason::ReportInvalid);
        } else {
            pass(result, "prelive_failed_cases_checked");
        }

        // motion safety validation
        if (options_.require_no_motion_rejection &&
            report.counters.motion_reject_count > 0) {
            fail(result, "motion_rejection_seen", PreLiveBlockReason::MotionCommandRejected);
        } else {
            pass(result, "motion_rejection_checked");
        }
        if (options_.require_stop_command_seen &&
            report.counters.stop_command_count <= 0) {
            fail(result, "stop_command_not_seen", PreLiveBlockReason::SafetyGateMissing);
        } else {
            pass(result, "stop_command_checked");
        }
        if (options_.require_active_scan_command_seen &&
            report.counters.active_scan_command_count <= 0) {
            fail(result, "active_scan_command_not_seen", PreLiveBlockReason::SafetyGateMissing);
        } else {
            pass(result, "active_scan_command_checked");
        }
        if (contains_forbidden_translation(report)) {
            fail(result, "forward_backward_command_forbidden", PreLiveBlockReason::SafetyGateMissing);
        } else {
            pass(result, "forward_backward_absent");
        }

        // map and coordinator validation
        if (options_.require_map_quality_good && !report.final_map_quality.good_enough) {
            fail(result, "map_quality_not_good", PreLiveBlockReason::MapQualityPoor);
        } else {
            pass(result, "map_quality_checked");
        }
        if (report.final_state == AutonomousSlamState::Fault) {
            fail(result, "coordinator_fault_state", PreLiveBlockReason::CoordinatorFault);
        } else {
            pass(result, "coordinator_fault_checked");
        }
        if (!options_.allow_coordinator_incomplete_for_shadow &&
            report.final_state != AutonomousSlamState::Completed) {
            fail(result, "coordinator_incomplete", PreLiveBlockReason::CoordinatorIncomplete);
        } else {
            pass(result, "coordinator_completion_checked");
        }

        result.pass = result.failed.empty();
        if (result.pass) {
            result.block_reason = PreLiveBlockReason::None;
        }
        return result;
    }

private:
    static void pass(PreLiveGateResult &result, const std::string &case_name) {
        result.passed.push_back(case_name);
    }

    static void fail(PreLiveGateResult &result,
                     const std::string &case_name,
                     PreLiveBlockReason reason) {
        result.failed.push_back(case_name);
        if (result.block_reason == PreLiveBlockReason::None) {
            result.block_reason = reason;
        }
    }

    static bool contains_forbidden_translation(const PreLiveIntegrationReport &report) {
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

    PreLiveGateOptions options_;
};

} // namespace robot_slamd
