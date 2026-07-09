#pragma once

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
    InvalidPacketTimestamp
};

struct MultiTofRawFrame {
    MultiTofMountId mount_id = MultiTofMountId::Front;
    RealSensorRequestTiming timing;
    std::string frame_id;
    std::vector<double> ranges_m;
    double angle_min_rad = 0.0;
    double angle_max_rad = 0.0;
    double angle_increment_rad = 0.0;
    double range_min_m = 0.0;
    double range_max_m = 0.0;
    double mount_yaw_rad = 0.0;
    int sequence = 0;
    std::string source;
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
    bool allow_nan_ranges = true;
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
    double max_nan_ratio = 0.50;
    double min_range_m = 0.02;
    double max_range_m = 8.00;
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
    frame.ranges_m = {1.0, 1.2, 1.4, 1.6};
    frame.angle_min_rad = -0.30;
    frame.angle_max_rad = 0.30;
    frame.angle_increment_rad = 0.20;
    frame.range_min_m = 0.02;
    frame.range_max_m = 8.00;
    frame.mount_yaw_rad = mount_yaw_rad;
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
