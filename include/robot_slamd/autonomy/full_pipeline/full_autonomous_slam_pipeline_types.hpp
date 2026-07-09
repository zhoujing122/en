#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class FullAutonomousSlamPipelineStage {
    NotStarted,
    BuildingScenario,
    RunningSensorReplay,
    UpdatingSlamBackend,
    UpdatingCoordinator,
    SendingShadowMotion,
    WaitingMotionSettle,
    Completed,
    Fault
};

enum class FullAutonomousSlamPipelineScenarioKind {
    CompletesWithoutActiveScan,
    RequiresActiveScanThenCompletes,
    SensorReplayInvalid,
    BackendRejected,
    MotionRejected,
    MapQualityStuck
};

enum class FullAutonomousSlamPipelineFault {
    None,
    ScenarioInvalid,
    SensorReplayNotReady,
    SensorSnapshotMissing,
    BackendNotReady,
    BackendRejected,
    CoordinatorFault,
    MotionRejected,
    MapQualityStuck,
    MaxStepsExceeded,
    CompletedButQualityNotGood
};

struct FullAutonomousSlamPipelineOptions {
    bool enabled = false;
    bool run_on_startup = false;
    int max_steps = 20;
    int max_active_scan_commands = 6;
    int min_backend_accepted_updates = 3;
    bool require_completion = true;
    bool require_shadow_motion_only = true;
    bool require_no_forward_backward = true;
    bool require_map_quality_good = true;
    double motion_settle_s = 0.20;
};

struct FullAutonomousSlamPipelineReport {
    bool ok = false;
    FullAutonomousSlamPipelineStage stage =
        FullAutonomousSlamPipelineStage::NotStarted;
    FullAutonomousSlamPipelineFault fault =
        FullAutonomousSlamPipelineFault::None;
    FullAutonomousSlamPipelineScenarioKind scenario =
        FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes;
    int step_count = 0;
    int sensor_snapshot_count = 0;
    int backend_update_count = 0;
    int backend_accepted_update_count = 0;
    int backend_rejected_update_count = 0;
    int coordinator_step_count = 0;
    int shadow_motion_command_count = 0;
    int active_scan_command_count = 0;
    int stop_command_count = 0;
    RobotSlamMapQuality final_quality;
    AutonomousSlamState final_state = AutonomousSlamState::Idle;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(FullAutonomousSlamPipelineStage stage) {
    switch (stage) {
    case FullAutonomousSlamPipelineStage::NotStarted:
        return "not_started";
    case FullAutonomousSlamPipelineStage::BuildingScenario:
        return "building_scenario";
    case FullAutonomousSlamPipelineStage::RunningSensorReplay:
        return "running_sensor_replay";
    case FullAutonomousSlamPipelineStage::UpdatingSlamBackend:
        return "updating_slam_backend";
    case FullAutonomousSlamPipelineStage::UpdatingCoordinator:
        return "updating_coordinator";
    case FullAutonomousSlamPipelineStage::SendingShadowMotion:
        return "sending_shadow_motion";
    case FullAutonomousSlamPipelineStage::WaitingMotionSettle:
        return "waiting_motion_settle";
    case FullAutonomousSlamPipelineStage::Completed:
        return "completed";
    case FullAutonomousSlamPipelineStage::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(FullAutonomousSlamPipelineScenarioKind kind) {
    switch (kind) {
    case FullAutonomousSlamPipelineScenarioKind::CompletesWithoutActiveScan:
        return "completes_without_active_scan";
    case FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes:
        return "requires_active_scan_then_completes";
    case FullAutonomousSlamPipelineScenarioKind::SensorReplayInvalid:
        return "sensor_replay_invalid";
    case FullAutonomousSlamPipelineScenarioKind::BackendRejected:
        return "backend_rejected";
    case FullAutonomousSlamPipelineScenarioKind::MotionRejected:
        return "motion_rejected";
    case FullAutonomousSlamPipelineScenarioKind::MapQualityStuck:
        return "map_quality_stuck";
    }
    return "unknown";
}

inline std::string to_string(FullAutonomousSlamPipelineFault fault) {
    switch (fault) {
    case FullAutonomousSlamPipelineFault::None:
        return "none";
    case FullAutonomousSlamPipelineFault::ScenarioInvalid:
        return "scenario_invalid";
    case FullAutonomousSlamPipelineFault::SensorReplayNotReady:
        return "sensor_replay_not_ready";
    case FullAutonomousSlamPipelineFault::SensorSnapshotMissing:
        return "sensor_snapshot_missing";
    case FullAutonomousSlamPipelineFault::BackendNotReady:
        return "backend_not_ready";
    case FullAutonomousSlamPipelineFault::BackendRejected:
        return "backend_rejected";
    case FullAutonomousSlamPipelineFault::CoordinatorFault:
        return "coordinator_fault";
    case FullAutonomousSlamPipelineFault::MotionRejected:
        return "motion_rejected";
    case FullAutonomousSlamPipelineFault::MapQualityStuck:
        return "map_quality_stuck";
    case FullAutonomousSlamPipelineFault::MaxStepsExceeded:
        return "max_steps_exceeded";
    case FullAutonomousSlamPipelineFault::CompletedButQualityNotGood:
        return "completed_but_quality_not_good";
    }
    return "unknown";
}

inline int full_autonomous_slam_pipeline_stage_id(
    FullAutonomousSlamPipelineStage stage) {
    return static_cast<int>(stage);
}

inline int full_autonomous_slam_pipeline_scenario_kind_id(
    FullAutonomousSlamPipelineScenarioKind kind) {
    return static_cast<int>(kind);
}

inline int full_autonomous_slam_pipeline_fault_id(
    FullAutonomousSlamPipelineFault fault) {
    return static_cast<int>(fault);
}

} // namespace robot_slamd
