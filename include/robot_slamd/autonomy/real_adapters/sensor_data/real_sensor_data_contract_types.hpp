#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class RealSensorDataKind {
    Tof,
    Imu,
    Wheel,
    CombinedPacket
};

enum class RealSensorDataContractStatus {
    Accepted,
    Rejected,
    Warning,
    Fault
};

enum class RealSensorDataFault {
    None,
    MissingTof,
    MissingImuAndWheel,
    NonFiniteTimestamp,
    StaleTimestamp,
    EmptyFrameId,
    EmptyTofRanges,
    InvalidTofAngle,
    InvalidTofRangeLimits,
    TofNanRatioTooHigh,
    InvalidImuTimestamp,
    InvalidImuYawRate,
    InvalidImuAcceleration,
    InvalidWheelTimestamp,
    InvalidWheelFlag,
    InvalidWheelVelocity,
    InvalidRequestTiming,
    RequestLatencyTooHigh,
    SensorSyncTooLarge,
    SnapshotBuildFailed
};

struct RealSensorRequestTiming {
    double request_start_s = 0.0;
    double response_received_s = 0.0;
    double estimated_sample_time_s = 0.0;
    double request_latency_s = 0.0;
    std::string timing_source;
};

struct RealSensorRawTofFrame {
    RealSensorRequestTiming timing;
    std::string frame_id;
    std::vector<double> ranges_m;
    double angle_min_rad = 0.0;
    double angle_max_rad = 0.0;
    double angle_increment_rad = 0.0;
    double range_min_m = 0.0;
    double range_max_m = 0.0;
    int sequence = 0;
    std::string source;
};

struct RealSensorRawImuSample {
    double timestamp_s = 0.0;
    std::string frame_id;
    double yaw_rate_rad_s = 0.0;
    double accel_x_mps2 = 0.0;
    double accel_y_mps2 = 0.0;
    double accel_z_mps2 = 0.0;
    int sequence = 0;
    std::string source;
};

struct RealSensorRawWheelSample {
    RealSensorRequestTiming timing;
    std::string frame_id;
    bool valid = false;
    double linear_velocity_mps = 0.0;
    double angular_velocity_rad_s = 0.0;
    int sequence = 0;
    std::string source;
};

struct RealSensorRawPacket {
    bool has_tof = false;
    bool has_imu = false;
    bool has_wheel = false;
    RealSensorRawTofFrame tof;
    RealSensorRawImuSample imu;
    RealSensorRawWheelSample wheel;
    double packet_timestamp_s = 0.0;
    std::string packet_source;
};

struct RealSensorDataContractOptions {
    bool require_tof = true;
    bool require_imu_or_wheel = true;
    bool require_frame_id = true;
    bool allow_nan_ranges = true;
    bool require_request_timing = true;
    double max_packet_age_s = 0.50;
    double max_sensor_sync_dt_s = 0.10;
    double max_request_latency_s = 0.20;
    double max_tof_nan_ratio = 0.50;
    double min_tof_range_m = 0.02;
    double max_tof_range_m = 8.00;
    double max_abs_yaw_rate_rad_s = 8.00;
    double max_accel_norm_mps2 = 50.00;
    double max_abs_wheel_linear_mps = 2.00;
    double max_abs_wheel_angular_rad_s = 6.00;
    std::string default_tof_frame_id = "tof_frame";
    std::string default_imu_frame_id = "imu_frame";
    std::string default_wheel_frame_id = "wheel_frame";
};

struct RealSensorDataContractResult {
    bool ok = false;
    RealSensorDataContractStatus status =
        RealSensorDataContractStatus::Rejected;
    RealSensorDataFault fault = RealSensorDataFault::None;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string message;
};

struct RealSensorSnapshotBuildResult {
    bool ok = false;
    RobotSlamSensorSnapshot snapshot;
    RealSensorDataContractResult contract_result;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string message;
};

inline std::string to_string(RealSensorDataKind kind) {
    switch (kind) {
    case RealSensorDataKind::Tof:
        return "Tof";
    case RealSensorDataKind::Imu:
        return "Imu";
    case RealSensorDataKind::Wheel:
        return "Wheel";
    case RealSensorDataKind::CombinedPacket:
        return "CombinedPacket";
    }
    return "Unknown";
}

inline std::string to_string(RealSensorDataContractStatus status) {
    switch (status) {
    case RealSensorDataContractStatus::Accepted:
        return "Accepted";
    case RealSensorDataContractStatus::Rejected:
        return "Rejected";
    case RealSensorDataContractStatus::Warning:
        return "Warning";
    case RealSensorDataContractStatus::Fault:
        return "Fault";
    }
    return "Unknown";
}

inline std::string to_string(RealSensorDataFault fault) {
    switch (fault) {
    case RealSensorDataFault::None:
        return "None";
    case RealSensorDataFault::MissingTof:
        return "MissingTof";
    case RealSensorDataFault::MissingImuAndWheel:
        return "MissingImuAndWheel";
    case RealSensorDataFault::NonFiniteTimestamp:
        return "NonFiniteTimestamp";
    case RealSensorDataFault::StaleTimestamp:
        return "StaleTimestamp";
    case RealSensorDataFault::EmptyFrameId:
        return "EmptyFrameId";
    case RealSensorDataFault::EmptyTofRanges:
        return "EmptyTofRanges";
    case RealSensorDataFault::InvalidTofAngle:
        return "InvalidTofAngle";
    case RealSensorDataFault::InvalidTofRangeLimits:
        return "InvalidTofRangeLimits";
    case RealSensorDataFault::TofNanRatioTooHigh:
        return "TofNanRatioTooHigh";
    case RealSensorDataFault::InvalidImuTimestamp:
        return "InvalidImuTimestamp";
    case RealSensorDataFault::InvalidImuYawRate:
        return "InvalidImuYawRate";
    case RealSensorDataFault::InvalidImuAcceleration:
        return "InvalidImuAcceleration";
    case RealSensorDataFault::InvalidWheelTimestamp:
        return "InvalidWheelTimestamp";
    case RealSensorDataFault::InvalidWheelFlag:
        return "InvalidWheelFlag";
    case RealSensorDataFault::InvalidWheelVelocity:
        return "InvalidWheelVelocity";
    case RealSensorDataFault::InvalidRequestTiming:
        return "InvalidRequestTiming";
    case RealSensorDataFault::RequestLatencyTooHigh:
        return "RequestLatencyTooHigh";
    case RealSensorDataFault::SensorSyncTooLarge:
        return "SensorSyncTooLarge";
    case RealSensorDataFault::SnapshotBuildFailed:
        return "SnapshotBuildFailed";
    }
    return "Unknown";
}

inline int real_sensor_data_kind_id(RealSensorDataKind kind) {
    return static_cast<int>(kind);
}

inline int real_sensor_data_contract_status_id(
    RealSensorDataContractStatus status) {
    return static_cast<int>(status);
}

inline int real_sensor_data_fault_id(RealSensorDataFault fault) {
    return static_cast<int>(fault);
}

inline RealSensorRequestTiming make_request_timing(
    double request_start_s,
    double response_received_s,
    const std::string &timing_source = "request_window") {
    RealSensorRequestTiming timing;
    timing.request_start_s = request_start_s;
    timing.response_received_s = response_received_s;
    timing.estimated_sample_time_s =
        0.5 * (request_start_s + response_received_s);
    timing.request_latency_s = response_received_s - request_start_s;
    timing.timing_source = timing_source;
    return timing;
}

inline double real_sensor_effective_time_s(
    const RealSensorRequestTiming &timing) {
    return timing.estimated_sample_time_s;
}

inline double real_sensor_request_latency_s(
    const RealSensorRequestTiming &timing) {
    return timing.request_latency_s;
}

} // namespace robot_slamd
