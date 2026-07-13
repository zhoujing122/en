#pragma once

#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_pipeline_types.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_scenario_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class MultiTofFullPipelineScenarioKind {
    NormalRequireAll,
    ActiveScanRequired,
    AllowMissingRight,
    MissingRightRequireAll,
    LowConfidenceFront,
    InvalidPayloadLength,
    InvalidHexPayload,
    DistanceAboveProtocolMaximum,
    PairwiseSyncFailure,
    ImuSyncFailure,
    WheelSyncFailure,
    EndOfReplayEarly,
    BackendReject,
    NullSensorPort,
    NotReadySensorPort
};

enum class MultiTofFullPipelineFault {
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
    CompletedButQualityNotGood,
    FakeMapBuildFailed,
    FakeMapSaveFailed
};

struct MultiTofFullPipelineScenario {
    MultiTofFullPipelineScenarioKind kind =
        MultiTofFullPipelineScenarioKind::NormalRequireAll;
    std::vector<MultiTofReplayRecord> records;
    MultiTofContractOptions contract_options;
    MultiTofSyncOptions sync_options;
    MultiTofSnapshotBuildOptions snapshot_options;
    MultiTofReplayOptions replay_options;
    FullAutonomousSlamScenario full_pipeline_scenario;
    std::string name;
};

struct MultiTofFullPipelineReport {
    bool ok = false;
    MultiTofFullPipelineScenarioKind scenario =
        MultiTofFullPipelineScenarioKind::NormalRequireAll;
    MultiTofFullPipelineFault fault = MultiTofFullPipelineFault::None;
    FullAutonomousSlamPipelineReport pipeline_report;
    FullAutonomousSlamPipelineReport full_pipeline;
    MultiTofReplayStatus last_replay_status = MultiTofReplayStatus::NotReady;
    MultiTofReplayFault last_replay_fault = MultiTofReplayFault::None;
    std::string last_replay_message;
    MultiTofReplayReadResult last_replay_result;

    int replay_read_call_count = 0;
    int successful_snapshot_count = 0;
    int rejected_snapshot_count = 0;
    int end_of_replay_return_count = 0;

    int multi_tof_snapshot_count = 0;
    int degraded_snapshot_count = 0;
    int front_present_count = 0;
    int left_present_count = 0;
    int right_present_count = 0;
    int front_mapping_usable_count = 0;
    int left_mapping_usable_count = 0;
    int right_mapping_usable_count = 0;
    int legacy_projection_count = 0;
    int maximum_legacy_projection_range_count = 0;

    bool scalar_tof_codec_used = false;
    bool echo_tag_preserved = false;
    bool confidence_preserved = false;
    bool effective_timestamp_preserved = false;
    bool synchronized_time_verified = false;

    bool multi_tof_reached_runner = false;
    bool multi_tof_reached_coordinator = false;
    bool legacy_projection_reached_backend = false;
    bool native_multi_tof_backend_consumption = false;

    bool phase_aware_sensor_consumption_verified = false;
    bool stale_snapshot_reused = false;

    bool forward_seen = false;
    bool backward_seen = false;
    bool fake_motion_only = true;
    bool real_hardware_accessed = false;
    bool real_motion_attempted = false;
    bool real_map_write_attempted = false;
    bool real_pose_writeback_attempted = false;

    bool has_last_success_snapshot = false;
    RobotSlamSensorSnapshot last_success_snapshot;
    int consumed_packet_count = 0;
    bool deterministic_backend_uses_legacy_tof = true;
    bool multi_tof_real_mapping_fusion = false;

    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(MultiTofFullPipelineScenarioKind kind) {
    switch (kind) {
    case MultiTofFullPipelineScenarioKind::NormalRequireAll:
        return "normal_require_all";
    case MultiTofFullPipelineScenarioKind::ActiveScanRequired:
        return "active_scan_required";
    case MultiTofFullPipelineScenarioKind::AllowMissingRight:
        return "allow_missing_right";
    case MultiTofFullPipelineScenarioKind::MissingRightRequireAll:
        return "missing_right_require_all";
    case MultiTofFullPipelineScenarioKind::LowConfidenceFront:
        return "low_confidence_front";
    case MultiTofFullPipelineScenarioKind::InvalidPayloadLength:
        return "invalid_payload_length";
    case MultiTofFullPipelineScenarioKind::InvalidHexPayload:
        return "invalid_hex_payload";
    case MultiTofFullPipelineScenarioKind::DistanceAboveProtocolMaximum:
        return "distance_above_protocol_maximum";
    case MultiTofFullPipelineScenarioKind::PairwiseSyncFailure:
        return "pairwise_sync_failure";
    case MultiTofFullPipelineScenarioKind::ImuSyncFailure:
        return "imu_sync_failure";
    case MultiTofFullPipelineScenarioKind::WheelSyncFailure:
        return "wheel_sync_failure";
    case MultiTofFullPipelineScenarioKind::EndOfReplayEarly:
        return "end_of_replay_early";
    case MultiTofFullPipelineScenarioKind::BackendReject:
        return "backend_reject";
    case MultiTofFullPipelineScenarioKind::NullSensorPort:
        return "null_sensor_port";
    case MultiTofFullPipelineScenarioKind::NotReadySensorPort:
        return "not_ready_sensor_port";
    }
    return "unknown";
}

inline std::string to_string(MultiTofFullPipelineFault fault) {
    switch (fault) {
    case MultiTofFullPipelineFault::None:
        return "none";
    case MultiTofFullPipelineFault::ScenarioInvalid:
        return "scenario_invalid";
    case MultiTofFullPipelineFault::SensorReplayNotReady:
        return "sensor_replay_not_ready";
    case MultiTofFullPipelineFault::SensorSnapshotMissing:
        return "sensor_snapshot_missing";
    case MultiTofFullPipelineFault::BackendNotReady:
        return "backend_not_ready";
    case MultiTofFullPipelineFault::BackendRejected:
        return "backend_rejected";
    case MultiTofFullPipelineFault::CoordinatorFault:
        return "coordinator_fault";
    case MultiTofFullPipelineFault::MotionRejected:
        return "motion_rejected";
    case MultiTofFullPipelineFault::MapQualityStuck:
        return "map_quality_stuck";
    case MultiTofFullPipelineFault::MaxStepsExceeded:
        return "max_steps_exceeded";
    case MultiTofFullPipelineFault::CompletedButQualityNotGood:
        return "completed_but_quality_not_good";
    case MultiTofFullPipelineFault::FakeMapBuildFailed:
        return "fake_map_build_failed";
    case MultiTofFullPipelineFault::FakeMapSaveFailed:
        return "fake_map_save_failed";
    }
    return "unknown";
}

inline int multi_tof_full_pipeline_scenario_kind_id(
    MultiTofFullPipelineScenarioKind kind) {
    return static_cast<int>(kind);
}

inline int multi_tof_full_pipeline_fault_id(
    MultiTofFullPipelineFault fault) {
    return static_cast<int>(fault);
}

inline MultiTofFullPipelineFault to_multi_tof_full_pipeline_fault(
    FullAutonomousSlamPipelineFault fault) {
    switch (fault) {
    case FullAutonomousSlamPipelineFault::None:
        return MultiTofFullPipelineFault::None;
    case FullAutonomousSlamPipelineFault::ScenarioInvalid:
        return MultiTofFullPipelineFault::ScenarioInvalid;
    case FullAutonomousSlamPipelineFault::SensorReplayNotReady:
        return MultiTofFullPipelineFault::SensorReplayNotReady;
    case FullAutonomousSlamPipelineFault::SensorSnapshotMissing:
        return MultiTofFullPipelineFault::SensorSnapshotMissing;
    case FullAutonomousSlamPipelineFault::BackendNotReady:
        return MultiTofFullPipelineFault::BackendNotReady;
    case FullAutonomousSlamPipelineFault::BackendRejected:
        return MultiTofFullPipelineFault::BackendRejected;
    case FullAutonomousSlamPipelineFault::CoordinatorFault:
        return MultiTofFullPipelineFault::CoordinatorFault;
    case FullAutonomousSlamPipelineFault::MotionRejected:
        return MultiTofFullPipelineFault::MotionRejected;
    case FullAutonomousSlamPipelineFault::MapQualityStuck:
        return MultiTofFullPipelineFault::MapQualityStuck;
    case FullAutonomousSlamPipelineFault::MaxStepsExceeded:
        return MultiTofFullPipelineFault::MaxStepsExceeded;
    case FullAutonomousSlamPipelineFault::CompletedButQualityNotGood:
        return MultiTofFullPipelineFault::CompletedButQualityNotGood;
    case FullAutonomousSlamPipelineFault::FakeMapBuildFailed:
        return MultiTofFullPipelineFault::FakeMapBuildFailed;
    case FullAutonomousSlamPipelineFault::FakeMapSaveFailed:
        return MultiTofFullPipelineFault::FakeMapSaveFailed;
    }
    return MultiTofFullPipelineFault::ScenarioInvalid;
}

} // namespace robot_slamd
