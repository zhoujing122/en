#pragma once

#include "robot_slamd/autonomy/fake_relocalization/fake_map_relocalization_runner.hpp"
#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_readiness_gate.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class FakeAutonomousSlamProductAcceptanceStatus {
    NotStarted,
    RunningMappingPipeline,
    RunningRelocalization,
    CheckingReadiness,
    Accepted,
    Rejected,
    Fault
};

enum class FakeAutonomousSlamProductAcceptanceBlockReason {
    None,
    MappingPipelineFailed,
    FakeMapMissing,
    RelocalizationFailed,
    RelocalizationReadinessBlocked,
    PoseWritebackAttempted,
    ForwardBackwardCommandObserved,
    HardwarePathObserved,
    AdapterManifestInvalid
};

struct FakeAutonomousSlamProductAcceptanceOptions {
    bool enabled = false;
    bool require_mapping_pipeline_success = true;
    bool require_fake_map_saved = true;
    bool require_relocalization_success = true;
    bool require_relocalization_readiness = true;
    bool require_no_pose_writeback = true;
    bool require_no_forward_backward = true;
    bool require_adapter_manifest = true;
    bool force_mapping_failure = false;
    bool force_fake_map_missing = false;
    bool force_relocalization_failure = false;
    bool force_readiness_blocked = false;
    bool force_manifest_invalid = false;
};

struct FakeAutonomousSlamProductAcceptanceReport {
    bool ok = false;
    FakeAutonomousSlamProductAcceptanceStatus status =
        FakeAutonomousSlamProductAcceptanceStatus::NotStarted;
    FakeAutonomousSlamProductAcceptanceBlockReason block_reason =
        FakeAutonomousSlamProductAcceptanceBlockReason::None;
    FullAutonomousSlamPipelineReport mapping_report;
    FakeMapRelocalizationRunnerReport relocalization_report;
    FakeRelocalizationReadinessResult readiness_result;
    std::string adapter_manifest_text;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(
    FakeAutonomousSlamProductAcceptanceStatus status) {
    switch (status) {
    case FakeAutonomousSlamProductAcceptanceStatus::NotStarted:
        return "not_started";
    case FakeAutonomousSlamProductAcceptanceStatus::RunningMappingPipeline:
        return "running_mapping_pipeline";
    case FakeAutonomousSlamProductAcceptanceStatus::RunningRelocalization:
        return "running_relocalization";
    case FakeAutonomousSlamProductAcceptanceStatus::CheckingReadiness:
        return "checking_readiness";
    case FakeAutonomousSlamProductAcceptanceStatus::Accepted:
        return "accepted";
    case FakeAutonomousSlamProductAcceptanceStatus::Rejected:
        return "rejected";
    case FakeAutonomousSlamProductAcceptanceStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(
    FakeAutonomousSlamProductAcceptanceBlockReason reason) {
    switch (reason) {
    case FakeAutonomousSlamProductAcceptanceBlockReason::None:
        return "none";
    case FakeAutonomousSlamProductAcceptanceBlockReason::MappingPipelineFailed:
        return "mapping_pipeline_failed";
    case FakeAutonomousSlamProductAcceptanceBlockReason::FakeMapMissing:
        return "fake_map_missing";
    case FakeAutonomousSlamProductAcceptanceBlockReason::RelocalizationFailed:
        return "relocalization_failed";
    case FakeAutonomousSlamProductAcceptanceBlockReason::RelocalizationReadinessBlocked:
        return "relocalization_readiness_blocked";
    case FakeAutonomousSlamProductAcceptanceBlockReason::PoseWritebackAttempted:
        return "pose_writeback_attempted";
    case FakeAutonomousSlamProductAcceptanceBlockReason::ForwardBackwardCommandObserved:
        return "forward_backward_command_observed";
    case FakeAutonomousSlamProductAcceptanceBlockReason::HardwarePathObserved:
        return "hardware_path_observed";
    case FakeAutonomousSlamProductAcceptanceBlockReason::AdapterManifestInvalid:
        return "adapter_manifest_invalid";
    }
    return "unknown";
}

inline int fake_autonomous_slam_product_acceptance_status_id(
    FakeAutonomousSlamProductAcceptanceStatus status) {
    return static_cast<int>(status);
}

inline int fake_autonomous_slam_product_acceptance_block_reason_id(
    FakeAutonomousSlamProductAcceptanceBlockReason reason) {
    return static_cast<int>(reason);
}

} // namespace robot_slamd
