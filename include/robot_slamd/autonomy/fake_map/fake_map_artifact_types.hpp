#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class FakeMapArtifactStatus {
    Empty,
    Built,
    Saved,
    Loaded,
    Rejected
};

enum class FakeMapArtifactFault {
    None,
    PipelineNotCompleted,
    QualityNotGood,
    EmptyMapId,
    SaveDisabled,
    LoadDisabled,
    ArtifactNotFound,
    InvalidArtifact
};

struct FakeMapMetadata {
    std::string map_id;
    std::string scenario_name;
    double created_time_s = 0.0;
    int accepted_update_count = 0;
    int active_scan_command_count = 0;
    double coverage_ratio = 0.0;
    double yaw_coverage_ratio = 0.0;
    int valid_scan_count = 0;
    std::string frame_id = "fake_map";
    std::string source = "full_fake_pipeline";
};

struct FakeMapArtifact {
    FakeMapArtifactStatus status = FakeMapArtifactStatus::Empty;
    FakeMapArtifactFault fault = FakeMapArtifactFault::None;
    FakeMapMetadata metadata;
    RobotSlamMapQuality final_quality;
    std::string serialized_text;
    std::string message;
};

struct FakeMapSaveOptions {
    bool enabled = false;
    bool allow_overwrite = false;
    bool require_quality_good = true;
    bool require_completed_pipeline = true;
};

struct FakeMapLoadOptions {
    bool enabled = false;
    bool require_valid_artifact = true;
};

struct FakeMapStorageResult {
    bool ok = false;
    FakeMapArtifact artifact;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(FakeMapArtifactStatus status) {
    switch (status) {
    case FakeMapArtifactStatus::Empty:
        return "empty";
    case FakeMapArtifactStatus::Built:
        return "built";
    case FakeMapArtifactStatus::Saved:
        return "saved";
    case FakeMapArtifactStatus::Loaded:
        return "loaded";
    case FakeMapArtifactStatus::Rejected:
        return "rejected";
    }
    return "unknown";
}

inline std::string to_string(FakeMapArtifactFault fault) {
    switch (fault) {
    case FakeMapArtifactFault::None:
        return "none";
    case FakeMapArtifactFault::PipelineNotCompleted:
        return "pipeline_not_completed";
    case FakeMapArtifactFault::QualityNotGood:
        return "quality_not_good";
    case FakeMapArtifactFault::EmptyMapId:
        return "empty_map_id";
    case FakeMapArtifactFault::SaveDisabled:
        return "save_disabled";
    case FakeMapArtifactFault::LoadDisabled:
        return "load_disabled";
    case FakeMapArtifactFault::ArtifactNotFound:
        return "artifact_not_found";
    case FakeMapArtifactFault::InvalidArtifact:
        return "invalid_artifact";
    }
    return "unknown";
}

inline int fake_map_artifact_status_id(
    FakeMapArtifactStatus status) {
    return static_cast<int>(status);
}

inline int fake_map_artifact_fault_id(
    FakeMapArtifactFault fault) {
    return static_cast<int>(fault);
}

} // namespace robot_slamd
