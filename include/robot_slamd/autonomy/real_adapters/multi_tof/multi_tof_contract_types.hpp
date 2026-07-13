#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/scalar_tof_geometry.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_types.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace robot_slamd {

enum class MultiTofMountId {
    Front,
    Left,
    Right
};

enum class MultiTofContractStatus {
    Accepted,
    Rejected,
    Warning,
    Fault
};

enum class MultiTofContractFault {
    None,
    MissingFrontTof,
    MissingLeftTof,
    MissingRightTof,
    TooFewTofFrames,
    DuplicateMountId,
    DuplicateFrameId,
    InvalidMountYaw,
    InvalidRequestTiming,
    RequestLatencyTooHigh,
    RequestLatencyMismatch,
    EstimatedSampleTimeOutsideWindow,
    EstimatedSampleTimeMidpointMismatch,
    FutureTimestamp,
    EmptyFrameId,
    EmptyRanges,
    InvalidAngle,
    InvalidRangeLimits,
    NanRatioTooHigh,
    InvalidPacketTimestamp,
    DistanceBelowProtocolMinimum,
    DistanceAboveProtocolMaximum,
    ConfidenceZero,
    ConfidenceOutOfRange,
    ConfidenceBelowMappingThreshold,
    DistanceOutsideMappingRange,
    InvalidFullFov
};

enum class ScalarTofValidity {
    ValidForMapping,
    DiagnosticOnly,
    Invalid
};

enum class ScalarTofFault {
    None,
    InvalidFrameLength,
    DistanceBelowProtocolMinimum,
    DistanceAboveProtocolMaximum,
    ConfidenceZero,
    ConfidenceOutOfRange,
    ConfidenceBelowMappingThreshold,
    DistanceOutsideMappingRange,
    InvalidRequestTiming,
    RequestLatencyTooHigh,
    FutureTimestamp,
    StaleTimestamp,
    InvalidFullFov
};

struct MultiTofRawFrame {
    MultiTofMountId mount_id = MultiTofMountId::Front;
    uint64_t echo_tag_u48 = 0;
    uint16_t distance_mm = 0;
    uint8_t confidence = 0;
    RealSensorRequestTiming timing;
    std::string frame_id;
    double mount_yaw_rad = 0.0;
    double full_fov_rad = 0.0;
    int sequence = 0;
    std::string source;
};

struct ScalarTofValidatedReading {
    MultiTofRawFrame raw;
    double distance_m = 0.0;
    double effective_timestamp_s = 0.0;
    bool protocol_valid = false;
    bool usable_for_mapping = false;
    ScalarTofValidity validity = ScalarTofValidity::Invalid;
    ScalarTofFault fault = ScalarTofFault::None;
    std::string reason;
};

struct MultiTofRawPacket {
    bool has_front = false;
    bool has_left = false;
    bool has_right = false;
    MultiTofRawFrame front;
    MultiTofRawFrame left;
    MultiTofRawFrame right;
    bool has_imu = false;
    bool has_wheel = false;
    RealSensorRawImuSample imu;
    RealSensorRawWheelSample wheel;
    double packet_timestamp_s = 0.0;
    std::string packet_source;
};

struct MultiTofContractOptions {
    bool require_front = true;
    bool require_left = true;
    bool require_right = true;
    bool require_unique_mount_ids = true;
    bool require_unique_frame_ids = true;
    bool require_request_timing = true;
    int min_required_tof_count = 3;
    double front_mount_yaw_rad = 0.0;
    double left_mount_yaw_rad = 1.5707963267948966;
    double right_mount_yaw_rad = -1.5707963267948966;
    double max_mount_yaw_error_rad = 0.001;
    double max_packet_age_s = 0.50;
    double max_request_latency_s = 0.20;
    double max_request_latency_mismatch_s = 0.001;
    double max_estimated_sample_time_midpoint_error_s = 0.005;
    double max_future_timestamp_skew_s = 0.05;
    int protocol_min_distance_mm = 20;
    int protocol_max_distance_mm = 12000;
    int mapping_min_distance_mm = 50;
    int mapping_max_distance_mm = 12000;
    int mapping_min_confidence = 70;
    double full_fov_deg = 1.6;
    std::string front_frame_id = "tof_front_frame";
    std::string left_frame_id = "tof_left_frame";
    std::string right_frame_id = "tof_right_frame";
};

struct MultiTofContractResult {
    bool ok = false;
    MultiTofContractStatus status = MultiTofContractStatus::Rejected;
    MultiTofContractFault fault = MultiTofContractFault::None;
    int valid_tof_count = 0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string message;
};

inline std::string to_string(MultiTofMountId mount_id) {
    switch (mount_id) {
    case MultiTofMountId::Front:
        return "front";
    case MultiTofMountId::Left:
        return "left";
    case MultiTofMountId::Right:
        return "right";
    }
    return "unknown";
}

inline std::string to_string(MultiTofContractStatus status) {
    switch (status) {
    case MultiTofContractStatus::Accepted:
        return "accepted";
    case MultiTofContractStatus::Rejected:
        return "rejected";
    case MultiTofContractStatus::Warning:
        return "warning";
    case MultiTofContractStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(MultiTofContractFault fault) {
    switch (fault) {
    case MultiTofContractFault::None:
        return "none";
    case MultiTofContractFault::MissingFrontTof:
        return "missing_front_tof";
    case MultiTofContractFault::MissingLeftTof:
        return "missing_left_tof";
    case MultiTofContractFault::MissingRightTof:
        return "missing_right_tof";
    case MultiTofContractFault::TooFewTofFrames:
        return "too_few_tof_frames";
    case MultiTofContractFault::DuplicateMountId:
        return "duplicate_mount_id";
    case MultiTofContractFault::DuplicateFrameId:
        return "duplicate_frame_id";
    case MultiTofContractFault::InvalidMountYaw:
        return "invalid_mount_yaw";
    case MultiTofContractFault::InvalidRequestTiming:
        return "invalid_request_timing";
    case MultiTofContractFault::RequestLatencyTooHigh:
        return "request_latency_too_high";
    case MultiTofContractFault::RequestLatencyMismatch:
        return "request_latency_mismatch";
    case MultiTofContractFault::EstimatedSampleTimeOutsideWindow:
        return "estimated_sample_time_outside_window";
    case MultiTofContractFault::EstimatedSampleTimeMidpointMismatch:
        return "estimated_sample_time_midpoint_mismatch";
    case MultiTofContractFault::FutureTimestamp:
        return "future_timestamp";
    case MultiTofContractFault::EmptyFrameId:
        return "empty_frame_id";
    case MultiTofContractFault::EmptyRanges:
        return "empty_ranges";
    case MultiTofContractFault::InvalidAngle:
        return "invalid_angle";
    case MultiTofContractFault::InvalidRangeLimits:
        return "invalid_range_limits";
    case MultiTofContractFault::NanRatioTooHigh:
        return "nan_ratio_too_high";
    case MultiTofContractFault::InvalidPacketTimestamp:
        return "invalid_packet_timestamp";
    case MultiTofContractFault::DistanceBelowProtocolMinimum:
        return "distance_below_protocol_minimum";
    case MultiTofContractFault::DistanceAboveProtocolMaximum:
        return "distance_above_protocol_maximum";
    case MultiTofContractFault::ConfidenceZero:
        return "confidence_zero";
    case MultiTofContractFault::ConfidenceOutOfRange:
        return "confidence_out_of_range";
    case MultiTofContractFault::ConfidenceBelowMappingThreshold:
        return "confidence_below_mapping_threshold";
    case MultiTofContractFault::DistanceOutsideMappingRange:
        return "distance_outside_mapping_range";
    case MultiTofContractFault::InvalidFullFov:
        return "invalid_full_fov";
    }
    return "unknown";
}

inline int multi_tof_mount_id_id(MultiTofMountId mount_id) {
    return static_cast<int>(mount_id);
}

inline int multi_tof_contract_status_id(MultiTofContractStatus status) {
    return static_cast<int>(status);
}

inline int multi_tof_contract_fault_id(MultiTofContractFault fault) {
    return static_cast<int>(fault);
}

inline double expected_mount_yaw_rad(
    MultiTofMountId mount_id,
    const MultiTofContractOptions &options) {
    switch (mount_id) {
    case MultiTofMountId::Front:
        return options.front_mount_yaw_rad;
    case MultiTofMountId::Left:
        return options.left_mount_yaw_rad;
    case MultiTofMountId::Right:
        return options.right_mount_yaw_rad;
    }
    return 0.0;
}

inline MultiTofRawFrame make_multi_tof_frame(
    MultiTofMountId mount_id,
    const std::string &frame_id,
    double mount_yaw_rad,
    double request_start_s = 99.98,
    double response_received_s = 100.02) {
    MultiTofRawFrame frame;
    frame.mount_id = mount_id;
    frame.timing = make_request_timing(request_start_s, response_received_s);
    frame.frame_id = frame_id;
    frame.echo_tag_u48 = 0x000044332211ULL + static_cast<uint64_t>(multi_tof_mount_id_id(mount_id));
    frame.distance_mm = 2731;
    frame.confidence = 100;
    frame.mount_yaw_rad = mount_yaw_rad;
    frame.full_fov_rad = scalar_tof_default_full_fov_rad();
    frame.sequence = 1;
    frame.source = "deterministic_multi_tof_sample";
    return frame;
}

inline int count_present_tof(const MultiTofRawPacket &packet) {
    int count = 0;
    if (packet.has_front) {
        count++;
    }
    if (packet.has_left) {
        count++;
    }
    if (packet.has_right) {
        count++;
    }
    return count;
}

} // namespace robot_slamd
