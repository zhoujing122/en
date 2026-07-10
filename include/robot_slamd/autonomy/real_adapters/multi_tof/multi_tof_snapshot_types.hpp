#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class MultiTofSnapshotBuildStatus {
    Built,
    BuiltDegraded,
    Rejected,
    Fault
};

enum class MultiTofSnapshotBuildFault {
    None,
    SyncFailed,
    MissingFront,
    MissingLeft,
    MissingRight,
    TooFewTofFrames,
    InvalidFrame,
    LegacyPrimaryMissing,
    InvalidSynchronizedTime,
    BuilderDisabled
};

enum class MultiTofLegacyPrimaryMode {
    Front,
    MedianTimeClosest,
    FirstPresent
};

struct MultiTofSnapshotBuildOptions {
    bool enabled = false;
    bool require_sync_pass = true;
    bool populate_legacy_tof = true;
    bool require_legacy_primary = true;
    MultiTofLegacyPrimaryMode legacy_primary_mode =
        MultiTofLegacyPrimaryMode::Front;
    int min_required_tof_count = 3;
};

struct MultiTofSnapshotBuildResult {
    bool ok = false;
    MultiTofSnapshotBuildStatus status =
        MultiTofSnapshotBuildStatus::Rejected;
    MultiTofSnapshotBuildFault fault =
        MultiTofSnapshotBuildFault::None;
    RobotSlamSensorSnapshot snapshot;
    MultiTofSyncResult sync_result;
    int valid_tof_count = 0;
    bool degraded = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string message;
};

inline std::string to_string(MultiTofSnapshotBuildStatus status) {
    switch (status) {
    case MultiTofSnapshotBuildStatus::Built:
        return "built";
    case MultiTofSnapshotBuildStatus::BuiltDegraded:
        return "built_degraded";
    case MultiTofSnapshotBuildStatus::Rejected:
        return "rejected";
    case MultiTofSnapshotBuildStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(MultiTofSnapshotBuildFault fault) {
    switch (fault) {
    case MultiTofSnapshotBuildFault::None:
        return "none";
    case MultiTofSnapshotBuildFault::SyncFailed:
        return "sync_failed";
    case MultiTofSnapshotBuildFault::MissingFront:
        return "missing_front";
    case MultiTofSnapshotBuildFault::MissingLeft:
        return "missing_left";
    case MultiTofSnapshotBuildFault::MissingRight:
        return "missing_right";
    case MultiTofSnapshotBuildFault::TooFewTofFrames:
        return "too_few_tof_frames";
    case MultiTofSnapshotBuildFault::InvalidFrame:
        return "invalid_frame";
    case MultiTofSnapshotBuildFault::LegacyPrimaryMissing:
        return "legacy_primary_missing";
    case MultiTofSnapshotBuildFault::InvalidSynchronizedTime:
        return "invalid_synchronized_time";
    case MultiTofSnapshotBuildFault::BuilderDisabled:
        return "builder_disabled";
    }
    return "unknown";
}

inline std::string to_string(MultiTofLegacyPrimaryMode mode) {
    switch (mode) {
    case MultiTofLegacyPrimaryMode::Front:
        return "front";
    case MultiTofLegacyPrimaryMode::MedianTimeClosest:
        return "median_time_closest";
    case MultiTofLegacyPrimaryMode::FirstPresent:
        return "first_present";
    }
    return "unknown";
}

inline int multi_tof_snapshot_build_status_id(
    MultiTofSnapshotBuildStatus status) {
    return static_cast<int>(status);
}

inline int multi_tof_snapshot_build_fault_id(
    MultiTofSnapshotBuildFault fault) {
    return static_cast<int>(fault);
}

inline int multi_tof_legacy_primary_mode_id(
    MultiTofLegacyPrimaryMode mode) {
    return static_cast<int>(mode);
}

} // namespace robot_slamd
