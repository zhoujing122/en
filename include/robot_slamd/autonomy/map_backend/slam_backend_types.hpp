#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/runtime/multi_tof_measurement_pose_types.hpp"

#include <string>

namespace robot_slamd {

enum class SlamBackendStage {
    Idle,
    ReceivingSnapshot,
    ValidatingInput,
    UpdatingMap,
    EvaluatingQuality,
    SavingMap,
    Ready,
    Fault
};

enum class SlamBackendFault {
    None,
    MissingTof,
    MissingMotionReference,
    InvalidTimestamp,
    InvalidTofScan,
    InvalidImu,
    InvalidWheel,
    UpdateRejected,
    QualityInvalid,
    SaveFailed,
    BackendNotReady
};

enum class SlamBackendUpdateStatus {
    Accepted,
    Rejected,
    Skipped,
    Fault
};

struct SlamBackendInputFrame {
    double timestamp_s = 0.0;
    RobotSlamSensorSnapshot snapshot;
    RobotPose2D predicted_pose;
    bool has_predicted_pose = false;
    MultiTofMeasurementPoseSet multi_tof_measurement_poses;
    bool has_multi_tof_measurement_poses = false;
    std::string source;
};

struct SlamBackendUpdateResult {
    SlamBackendUpdateStatus status = SlamBackendUpdateStatus::Rejected;
    SlamBackendFault fault = SlamBackendFault::None;
    bool map_updated = false;
    bool keyframe_added = false;
    int integrated_scan_count = 0;
    double update_latency_s = 0.0;
    RobotSlamMapQuality quality;
    std::string message;
};

struct SlamBackendSaveResult {
    bool ok = false;
    std::string output_path;
    std::string error;
};

struct SlamBackendContractOptions {
    bool require_tof = true;
    bool require_imu_or_wheel = true;
    bool allow_predicted_pose_missing = true;
    double max_input_age_s = 0.50;
    double max_update_latency_s = 1.00;
    int min_integrated_scan_count_for_quality = 0;
};

inline std::string to_string(SlamBackendStage stage) {
    switch (stage) {
    case SlamBackendStage::Idle:
        return "idle";
    case SlamBackendStage::ReceivingSnapshot:
        return "receiving_snapshot";
    case SlamBackendStage::ValidatingInput:
        return "validating_input";
    case SlamBackendStage::UpdatingMap:
        return "updating_map";
    case SlamBackendStage::EvaluatingQuality:
        return "evaluating_quality";
    case SlamBackendStage::SavingMap:
        return "saving_map";
    case SlamBackendStage::Ready:
        return "ready";
    case SlamBackendStage::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(SlamBackendFault fault) {
    switch (fault) {
    case SlamBackendFault::None:
        return "none";
    case SlamBackendFault::MissingTof:
        return "missing_tof";
    case SlamBackendFault::MissingMotionReference:
        return "missing_motion_reference";
    case SlamBackendFault::InvalidTimestamp:
        return "invalid_timestamp";
    case SlamBackendFault::InvalidTofScan:
        return "invalid_tof_scan";
    case SlamBackendFault::InvalidImu:
        return "invalid_imu";
    case SlamBackendFault::InvalidWheel:
        return "invalid_wheel";
    case SlamBackendFault::UpdateRejected:
        return "update_rejected";
    case SlamBackendFault::QualityInvalid:
        return "quality_invalid";
    case SlamBackendFault::SaveFailed:
        return "save_failed";
    case SlamBackendFault::BackendNotReady:
        return "backend_not_ready";
    }
    return "unknown";
}

inline std::string to_string(SlamBackendUpdateStatus status) {
    switch (status) {
    case SlamBackendUpdateStatus::Accepted:
        return "accepted";
    case SlamBackendUpdateStatus::Rejected:
        return "rejected";
    case SlamBackendUpdateStatus::Skipped:
        return "skipped";
    case SlamBackendUpdateStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline int slam_backend_stage_id(SlamBackendStage stage) {
    return static_cast<int>(stage);
}

inline int slam_backend_fault_id(SlamBackendFault fault) {
    return static_cast<int>(fault);
}

inline int slam_backend_update_status_id(SlamBackendUpdateStatus status) {
    return static_cast<int>(status);
}

} // namespace robot_slamd
