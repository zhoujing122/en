#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_acceptance_runner.hpp"
#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class AutonomousSlamE2EScenarioKind {
    MinimalMapAlreadyGood,
    ActiveScanThenMapGood,
    SensorContractFailure,
    SlamBackendUpdateRejected,
    SlamBackendQualityInvalid,
    MotionRejected,
    MapQualityStuck
};

enum class AutonomousSlamE2EStage {
    NotStarted,
    BuildingScenario,
    CheckingRealAdapterContracts,
    CheckingSlamBackend,
    RunningPreLive,
    EvaluatingScenario,
    Passed,
    Blocked,
    Fault
};

enum class AutonomousSlamE2EBlockReason {
    None,
    ConfigDisabled,
    ScenarioBuildFailed,
    RealAdapterContractFailed,
    SlamBackendAcceptanceFailed,
    PreLiveRunnerFailed,
    PreLiveGateBlocked,
    MotionRejected,
    ForwardBackwardCommandObserved,
    MapQualityNeverGood,
    ReportInvalid,
    UnexpectedException
};

struct AutonomousSlamE2EScenarioOptions {
    bool enabled = false;
    AutonomousSlamE2EScenarioKind scenario_kind =
        AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood;
    int max_iterations = 30;
    double start_time_s = 100.0;
    double step_s = 0.10;
    bool require_slam_backend_acceptance = true;
    bool require_prelive_pass = true;
    bool require_no_forward_backward = true;
    bool require_stop_command_seen = true;
    bool require_active_scan_when_map_poor = true;
    bool require_map_quality_good_at_end = false;
};

struct AutonomousSlamE2EScenarioData {
    std::vector<RobotSlamSensorSnapshot> sensor_snapshots;
    std::vector<SlamBackendUpdateResult> backend_update_results;
    std::vector<RobotSlamMapQuality> backend_qualities;
    bool sensor_ready = true;
    bool motion_ready = true;
    bool map_ready = true;
    bool reject_motion = false;
    bool fail_motion_feedback = false;
};

struct AutonomousSlamE2EScenarioReport {
    bool ok = false;
    AutonomousSlamE2EStage stage = AutonomousSlamE2EStage::NotStarted;
    AutonomousSlamE2EBlockReason block_reason =
        AutonomousSlamE2EBlockReason::None;
    AutonomousSlamE2EScenarioKind scenario_kind =
        AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood;
    SlamBackendAcceptanceRunnerResult slam_backend_acceptance;
    PreLiveIntegrationReport prelive_report;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(AutonomousSlamE2EScenarioKind kind) {
    switch (kind) {
    case AutonomousSlamE2EScenarioKind::MinimalMapAlreadyGood:
        return "MinimalMapAlreadyGood";
    case AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood:
        return "ActiveScanThenMapGood";
    case AutonomousSlamE2EScenarioKind::SensorContractFailure:
        return "SensorContractFailure";
    case AutonomousSlamE2EScenarioKind::SlamBackendUpdateRejected:
        return "SlamBackendUpdateRejected";
    case AutonomousSlamE2EScenarioKind::SlamBackendQualityInvalid:
        return "SlamBackendQualityInvalid";
    case AutonomousSlamE2EScenarioKind::MotionRejected:
        return "MotionRejected";
    case AutonomousSlamE2EScenarioKind::MapQualityStuck:
        return "MapQualityStuck";
    }
    return "Unknown";
}

inline std::string to_string(AutonomousSlamE2EStage stage) {
    switch (stage) {
    case AutonomousSlamE2EStage::NotStarted:
        return "NotStarted";
    case AutonomousSlamE2EStage::BuildingScenario:
        return "BuildingScenario";
    case AutonomousSlamE2EStage::CheckingRealAdapterContracts:
        return "CheckingRealAdapterContracts";
    case AutonomousSlamE2EStage::CheckingSlamBackend:
        return "CheckingSlamBackend";
    case AutonomousSlamE2EStage::RunningPreLive:
        return "RunningPreLive";
    case AutonomousSlamE2EStage::EvaluatingScenario:
        return "EvaluatingScenario";
    case AutonomousSlamE2EStage::Passed:
        return "Passed";
    case AutonomousSlamE2EStage::Blocked:
        return "Blocked";
    case AutonomousSlamE2EStage::Fault:
        return "Fault";
    }
    return "Unknown";
}

inline std::string to_string(AutonomousSlamE2EBlockReason reason) {
    switch (reason) {
    case AutonomousSlamE2EBlockReason::None:
        return "None";
    case AutonomousSlamE2EBlockReason::ConfigDisabled:
        return "ConfigDisabled";
    case AutonomousSlamE2EBlockReason::ScenarioBuildFailed:
        return "ScenarioBuildFailed";
    case AutonomousSlamE2EBlockReason::RealAdapterContractFailed:
        return "RealAdapterContractFailed";
    case AutonomousSlamE2EBlockReason::SlamBackendAcceptanceFailed:
        return "SlamBackendAcceptanceFailed";
    case AutonomousSlamE2EBlockReason::PreLiveRunnerFailed:
        return "PreLiveRunnerFailed";
    case AutonomousSlamE2EBlockReason::PreLiveGateBlocked:
        return "PreLiveGateBlocked";
    case AutonomousSlamE2EBlockReason::MotionRejected:
        return "MotionRejected";
    case AutonomousSlamE2EBlockReason::ForwardBackwardCommandObserved:
        return "ForwardBackwardCommandObserved";
    case AutonomousSlamE2EBlockReason::MapQualityNeverGood:
        return "MapQualityNeverGood";
    case AutonomousSlamE2EBlockReason::ReportInvalid:
        return "ReportInvalid";
    case AutonomousSlamE2EBlockReason::UnexpectedException:
        return "UnexpectedException";
    }
    return "Unknown";
}

inline int autonomous_slam_e2e_scenario_kind_id(
    AutonomousSlamE2EScenarioKind kind) {
    return static_cast<int>(kind);
}

inline int autonomous_slam_e2e_stage_id(AutonomousSlamE2EStage stage) {
    return static_cast<int>(stage);
}

inline int autonomous_slam_e2e_block_reason_id(
    AutonomousSlamE2EBlockReason reason) {
    return static_cast<int>(reason);
}

} // namespace robot_slamd
