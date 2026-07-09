#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_types.hpp"

#include <string>

namespace robot_slamd {

enum class DeterministicSlamBackendStage {
    Idle,
    ValidatingInput,
    EvaluatingTof,
    UpdatingQuality,
    Ready,
    Fault
};

enum class DeterministicSlamBackendFault {
    None,
    BackendDisabled,
    BackendNotReady,
    MissingTof,
    MissingMotionReference,
    InvalidTimestamp,
    EmptyTofRanges,
    TooFewValidRanges,
    InvalidAngleIncrement,
    InvalidRangeLimit,
    SaveNotImplemented,
    QualityNotGoodEnough
};

enum class DeterministicSlamBackendUpdateKind {
    None,
    ScanIntegrated,
    KeyframeAdded,
    Rejected
};

struct DeterministicTofScanStats {
    bool valid = false;
    int total_range_count = 0;
    int finite_range_count = 0;
    int valid_range_count = 0;
    int near_obstacle_count = 0;
    double valid_range_ratio = 0.0;
    double min_range_m = 0.0;
    double max_range_m = 0.0;
    double mean_range_m = 0.0;
    double horizontal_fov_rad = 0.0;
    std::string reason;
};

struct DeterministicSlamBackendOptions {
    bool enabled = false;
    bool ready = false;
    bool require_tof = true;
    bool require_imu_or_wheel = true;
    bool allow_save_map = false;
    int min_valid_ranges = 3;
    int min_valid_scan_count_for_good = 3;
    double min_valid_range_ratio = 0.30;
    double min_coverage_ratio_for_good = 0.60;
    double min_yaw_coverage_ratio_for_good = 0.30;
    double keyframe_yaw_delta_rad = 0.15;
    double min_range_m = 0.02;
    double max_range_m = 8.00;
    double max_update_latency_s = 0.50;
    double assumed_scan_yaw_span_rad = 0.52;
    double yaw_bin_size_rad = 0.26;
};

struct DeterministicSlamBackendState {
    DeterministicSlamBackendStage stage =
        DeterministicSlamBackendStage::Idle;
    DeterministicSlamBackendFault fault =
        DeterministicSlamBackendFault::None;
    DeterministicSlamBackendUpdateKind last_update_kind =
        DeterministicSlamBackendUpdateKind::None;
    int update_call_count = 0;
    int accepted_update_count = 0;
    int rejected_update_count = 0;
    int keyframe_count = 0;
    int valid_scan_count = 0;
    double last_update_time_s = 0.0;
    double last_keyframe_yaw_rad = 0.0;
    double accumulated_yaw_abs_rad = 0.0;
    double coverage_ratio = 0.0;
    double yaw_coverage_ratio = 0.0;
    std::string reason;
};

inline std::string to_string(DeterministicSlamBackendStage stage) {
    switch (stage) {
    case DeterministicSlamBackendStage::Idle:
        return "idle";
    case DeterministicSlamBackendStage::ValidatingInput:
        return "validating_input";
    case DeterministicSlamBackendStage::EvaluatingTof:
        return "evaluating_tof";
    case DeterministicSlamBackendStage::UpdatingQuality:
        return "updating_quality";
    case DeterministicSlamBackendStage::Ready:
        return "ready";
    case DeterministicSlamBackendStage::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(DeterministicSlamBackendFault fault) {
    switch (fault) {
    case DeterministicSlamBackendFault::None:
        return "none";
    case DeterministicSlamBackendFault::BackendDisabled:
        return "backend_disabled";
    case DeterministicSlamBackendFault::BackendNotReady:
        return "backend_not_ready";
    case DeterministicSlamBackendFault::MissingTof:
        return "missing_tof";
    case DeterministicSlamBackendFault::MissingMotionReference:
        return "missing_motion_reference";
    case DeterministicSlamBackendFault::InvalidTimestamp:
        return "invalid_timestamp";
    case DeterministicSlamBackendFault::EmptyTofRanges:
        return "empty_tof_ranges";
    case DeterministicSlamBackendFault::TooFewValidRanges:
        return "too_few_valid_ranges";
    case DeterministicSlamBackendFault::InvalidAngleIncrement:
        return "invalid_angle_increment";
    case DeterministicSlamBackendFault::InvalidRangeLimit:
        return "invalid_range_limit";
    case DeterministicSlamBackendFault::SaveNotImplemented:
        return "save_not_implemented";
    case DeterministicSlamBackendFault::QualityNotGoodEnough:
        return "quality_not_good_enough";
    }
    return "unknown";
}

inline std::string to_string(DeterministicSlamBackendUpdateKind kind) {
    switch (kind) {
    case DeterministicSlamBackendUpdateKind::None:
        return "none";
    case DeterministicSlamBackendUpdateKind::ScanIntegrated:
        return "scan_integrated";
    case DeterministicSlamBackendUpdateKind::KeyframeAdded:
        return "keyframe_added";
    case DeterministicSlamBackendUpdateKind::Rejected:
        return "rejected";
    }
    return "unknown";
}

inline int deterministic_slam_backend_stage_id(
    DeterministicSlamBackendStage stage) {
    return static_cast<int>(stage);
}

inline int deterministic_slam_backend_fault_id(
    DeterministicSlamBackendFault fault) {
    return static_cast<int>(fault);
}

inline int deterministic_slam_backend_update_kind_id(
    DeterministicSlamBackendUpdateKind kind) {
    return static_cast<int>(kind);
}

} // namespace robot_slamd
