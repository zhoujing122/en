#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_contract_types.hpp"
#include "robot_slamd/motion/algorithm_motion_command.hpp"
#include "robot_slamd/software_motion/software_motion_command.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class PreLiveIntegrationStage {
    NotStarted,
    CheckingReadiness,
    CheckingContracts,
    RunningAdapterAcceptance,
    RunningCoordinator,
    EvaluatingGate,
    Passed,
    Blocked,
    Fault
};

enum class PreLiveBlockReason {
    None,
    ConfigDisabled,
    ReadinessFailed,
    ContractFailed,
    AdapterAcceptanceFailed,
    CoordinatorFault,
    CoordinatorIncomplete,
    MotionCommandRejected,
    MapQualityPoor,
    SafetyGateMissing,
    ReportInvalid,
    MaxIterationReached
};

struct PreLiveIntegrationOptions {
    bool enabled = false;
    int max_iterations = 30;
    double start_time_s = 100.0;
    double step_s = 0.10;
    bool require_adapter_acceptance = true;
    bool require_coordinator_completed = false;
    bool require_no_motion_rejection = true;
    bool require_stop_command_seen = true;
    bool require_active_scan_command_seen = false;
    bool require_map_quality_good = false;
};

struct PreLiveIntegrationCounters {
    int readiness_check_count = 0;
    int contract_check_count = 0;
    int adapter_acceptance_run_count = 0;
    int coordinator_step_count = 0;
    int motion_command_count = 0;
    int stop_command_count = 0;
    int active_scan_command_count = 0;
    int motion_reject_count = 0;
    int coordinator_fault_count = 0;
};

struct PreLiveIntegrationReport {
    bool ok = false;
    PreLiveIntegrationStage stage = PreLiveIntegrationStage::NotStarted;
    PreLiveBlockReason block_reason = PreLiveBlockReason::None;
    RealAdapterReadiness readiness = RealAdapterReadiness::NotReady;
    AutonomousSlamState final_state = AutonomousSlamState::Idle;
    AutonomousSlamFault final_fault = AutonomousSlamFault::None;
    RobotSlamMapQuality final_map_quality;
    PreLiveIntegrationCounters counters;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::vector<AlgorithmMotionCommand> algorithm_commands;
    std::vector<SoftwareMotionCommand> software_commands;
    std::string summary;
};

inline std::string to_string(PreLiveIntegrationStage stage) {
    switch (stage) {
    case PreLiveIntegrationStage::NotStarted:
        return "not_started";
    case PreLiveIntegrationStage::CheckingReadiness:
        return "checking_readiness";
    case PreLiveIntegrationStage::CheckingContracts:
        return "checking_contracts";
    case PreLiveIntegrationStage::RunningAdapterAcceptance:
        return "running_adapter_acceptance";
    case PreLiveIntegrationStage::RunningCoordinator:
        return "running_coordinator";
    case PreLiveIntegrationStage::EvaluatingGate:
        return "evaluating_gate";
    case PreLiveIntegrationStage::Passed:
        return "passed";
    case PreLiveIntegrationStage::Blocked:
        return "blocked";
    case PreLiveIntegrationStage::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(PreLiveBlockReason reason) {
    switch (reason) {
    case PreLiveBlockReason::None:
        return "none";
    case PreLiveBlockReason::ConfigDisabled:
        return "config_disabled";
    case PreLiveBlockReason::ReadinessFailed:
        return "readiness_failed";
    case PreLiveBlockReason::ContractFailed:
        return "contract_failed";
    case PreLiveBlockReason::AdapterAcceptanceFailed:
        return "adapter_acceptance_failed";
    case PreLiveBlockReason::CoordinatorFault:
        return "coordinator_fault";
    case PreLiveBlockReason::CoordinatorIncomplete:
        return "coordinator_incomplete";
    case PreLiveBlockReason::MotionCommandRejected:
        return "motion_command_rejected";
    case PreLiveBlockReason::MapQualityPoor:
        return "map_quality_poor";
    case PreLiveBlockReason::SafetyGateMissing:
        return "safety_gate_missing";
    case PreLiveBlockReason::ReportInvalid:
        return "report_invalid";
    case PreLiveBlockReason::MaxIterationReached:
        return "max_iteration_reached";
    }
    return "unknown";
}

inline int prelive_stage_id(PreLiveIntegrationStage stage) {
    return static_cast<int>(stage);
}

inline int prelive_block_reason_id(PreLiveBlockReason reason) {
    return static_cast<int>(reason);
}

} // namespace robot_slamd
