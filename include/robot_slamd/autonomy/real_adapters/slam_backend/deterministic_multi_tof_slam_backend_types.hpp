#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_types.hpp"

#include <string>

namespace robot_slamd {

enum class DeterministicMultiTofBackendStage {
    NotStarted,
    ValidatingInput,
    EvaluatingMultiTof,
    UpdatingQuality,
    Ready,
    Fault
};

enum class DeterministicMultiTofBackendFault {
    None,
    BackendDisabled,
    BackendNotReady,
    InvalidTimestamp,
    MissingMultiTof,
    TooFewTofFrames,
    EmptyFrontRanges,
    EmptyLeftRanges,
    EmptyRightRanges,
    InvalidFrame,
    TooFewValidRanges,
    MissingMotionReference,
    SaveNotImplemented
};

struct DeterministicMultiTofBackendOptions {
    bool enabled = false;
    bool ready = false;
    bool require_multi_tof = true;
    bool require_imu_or_wheel = true;
    int min_required_tof_count = 3;
    int min_valid_ranges_per_present_tof = 2;
    double min_valid_range_ratio = 0.30;
    double min_range_m = 0.02;
    double max_range_m = 8.0;
    double front_coverage_weight = 0.34;
    double left_coverage_weight = 0.33;
    double right_coverage_weight = 0.33;
    double yaw_coverage_per_side_tof = 0.30;
    double yaw_coverage_full_three_tof = 0.90;
    int scans_to_good_quality = 3;
    double min_good_coverage_ratio = 0.60;
    double min_good_yaw_coverage_ratio = 0.30;
};

struct DeterministicMultiTofScanStats {
    bool valid = false;
    int present_tof_count = 0;
    int valid_tof_count = 0;
    int total_range_count = 0;
    int valid_range_count = 0;
    bool has_front = false;
    bool has_left = false;
    bool has_right = false;
    double coverage_contribution = 0.0;
    double yaw_coverage_contribution = 0.0;
    std::string reason;
};

struct DeterministicMultiTofBackendState {
    DeterministicMultiTofBackendStage stage =
        DeterministicMultiTofBackendStage::NotStarted;
    DeterministicMultiTofBackendFault fault =
        DeterministicMultiTofBackendFault::None;
    int update_call_count = 0;
    int accepted_update_count = 0;
    int rejected_update_count = 0;
    int valid_tof_count = 0;
    int valid_scan_count = 0;
    double coverage_ratio = 0.0;
    double yaw_coverage_ratio = 0.0;
    double last_update_time_s = 0.0;
    std::string reason;
};

inline std::string to_string(DeterministicMultiTofBackendStage stage) {
    switch (stage) {
    case DeterministicMultiTofBackendStage::NotStarted:
        return "not_started";
    case DeterministicMultiTofBackendStage::ValidatingInput:
        return "validating_input";
    case DeterministicMultiTofBackendStage::EvaluatingMultiTof:
        return "evaluating_multi_tof";
    case DeterministicMultiTofBackendStage::UpdatingQuality:
        return "updating_quality";
    case DeterministicMultiTofBackendStage::Ready:
        return "ready";
    case DeterministicMultiTofBackendStage::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(DeterministicMultiTofBackendFault fault) {
    switch (fault) {
    case DeterministicMultiTofBackendFault::None:
        return "none";
    case DeterministicMultiTofBackendFault::BackendDisabled:
        return "backend_disabled";
    case DeterministicMultiTofBackendFault::BackendNotReady:
        return "backend_not_ready";
    case DeterministicMultiTofBackendFault::InvalidTimestamp:
        return "invalid_timestamp";
    case DeterministicMultiTofBackendFault::MissingMultiTof:
        return "missing_multi_tof";
    case DeterministicMultiTofBackendFault::TooFewTofFrames:
        return "too_few_tof_frames";
    case DeterministicMultiTofBackendFault::EmptyFrontRanges:
        return "empty_front_ranges";
    case DeterministicMultiTofBackendFault::EmptyLeftRanges:
        return "empty_left_ranges";
    case DeterministicMultiTofBackendFault::EmptyRightRanges:
        return "empty_right_ranges";
    case DeterministicMultiTofBackendFault::InvalidFrame:
        return "invalid_frame";
    case DeterministicMultiTofBackendFault::TooFewValidRanges:
        return "too_few_valid_ranges";
    case DeterministicMultiTofBackendFault::MissingMotionReference:
        return "missing_motion_reference";
    case DeterministicMultiTofBackendFault::SaveNotImplemented:
        return "save_not_implemented";
    }
    return "unknown";
}

inline int deterministic_multi_tof_backend_stage_id(
    DeterministicMultiTofBackendStage stage) {
    return static_cast<int>(stage);
}

inline int deterministic_multi_tof_backend_fault_id(
    DeterministicMultiTofBackendFault fault) {
    return static_cast<int>(fault);
}

} // namespace robot_slamd
