#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/autonomy/fake_map/fake_map_artifact_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class FakeRelocalizationStatus {
    NotStarted,
    MapLoaded,
    EvaluatingScan,
    Accepted,
    Rejected,
    Fault
};

enum class FakeRelocalizationFault {
    None,
    Disabled,
    MapNotLoaded,
    InvalidMapArtifact,
    MapQualityTooLow,
    MissingTof,
    EmptyTofRanges,
    TooFewValidRanges,
    ConfidenceTooLow,
    PoseWritebackForbidden,
    RunnerInputInvalid
};

enum class FakeRelocalizationConfidenceBand {
    None,
    Low,
    Medium,
    High
};

struct FakePoseCandidate {
    bool valid = false;
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
    double confidence = 0.0;
    std::string frame_id = "fake_map";
    std::string source = "fake_relocalization";
};

struct FakeRelocalizationOptions {
    bool enabled = false;
    bool allow_pose_writeback = false;
    bool require_map_quality_good = true;
    bool require_tof = true;
    int min_valid_ranges = 3;
    double min_confidence = 0.70;
    double high_confidence_threshold = 0.85;
    double min_map_coverage_ratio = 0.60;
    double min_map_yaw_coverage_ratio = 0.30;
};

struct FakeRelocalizationResult {
    bool ok = false;
    FakeRelocalizationStatus status =
        FakeRelocalizationStatus::NotStarted;
    FakeRelocalizationFault fault =
        FakeRelocalizationFault::None;
    FakeRelocalizationConfidenceBand confidence_band =
        FakeRelocalizationConfidenceBand::None;
    FakePoseCandidate pose;
    FakeMapArtifact map_artifact;
    RobotSlamSensorSnapshot input_snapshot;
    int valid_range_count = 0;
    double valid_range_ratio = 0.0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(
    FakeRelocalizationStatus status) {
    switch (status) {
    case FakeRelocalizationStatus::NotStarted:
        return "not_started";
    case FakeRelocalizationStatus::MapLoaded:
        return "map_loaded";
    case FakeRelocalizationStatus::EvaluatingScan:
        return "evaluating_scan";
    case FakeRelocalizationStatus::Accepted:
        return "accepted";
    case FakeRelocalizationStatus::Rejected:
        return "rejected";
    case FakeRelocalizationStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(
    FakeRelocalizationFault fault) {
    switch (fault) {
    case FakeRelocalizationFault::None:
        return "none";
    case FakeRelocalizationFault::Disabled:
        return "disabled";
    case FakeRelocalizationFault::MapNotLoaded:
        return "map_not_loaded";
    case FakeRelocalizationFault::InvalidMapArtifact:
        return "invalid_map_artifact";
    case FakeRelocalizationFault::MapQualityTooLow:
        return "map_quality_too_low";
    case FakeRelocalizationFault::MissingTof:
        return "missing_tof";
    case FakeRelocalizationFault::EmptyTofRanges:
        return "empty_tof_ranges";
    case FakeRelocalizationFault::TooFewValidRanges:
        return "too_few_valid_ranges";
    case FakeRelocalizationFault::ConfidenceTooLow:
        return "confidence_too_low";
    case FakeRelocalizationFault::PoseWritebackForbidden:
        return "pose_writeback_forbidden";
    case FakeRelocalizationFault::RunnerInputInvalid:
        return "runner_input_invalid";
    }
    return "unknown";
}

inline std::string to_string(
    FakeRelocalizationConfidenceBand band) {
    switch (band) {
    case FakeRelocalizationConfidenceBand::None:
        return "none";
    case FakeRelocalizationConfidenceBand::Low:
        return "low";
    case FakeRelocalizationConfidenceBand::Medium:
        return "medium";
    case FakeRelocalizationConfidenceBand::High:
        return "high";
    }
    return "unknown";
}

inline int fake_relocalization_status_id(
    FakeRelocalizationStatus status) {
    return static_cast<int>(status);
}

inline int fake_relocalization_fault_id(
    FakeRelocalizationFault fault) {
    return static_cast<int>(fault);
}

inline int fake_relocalization_confidence_band_id(
    FakeRelocalizationConfidenceBand band) {
    return static_cast<int>(band);
}

} // namespace robot_slamd
