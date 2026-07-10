#pragma once

#include "robot_slamd/motion/algorithm_motion_command.hpp"
#include "robot_slamd/software_motion/software_motion_command.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class AutonomousSlamState {
    Idle,
    WaitingForSensors,
    Initializing,
    Mapping,
    NeedActiveScan,
    SendingMotionCommand,
    WaitingMotionSettle,
    IntegratingScan,
    Completed,
    Fault
};

enum class AutonomousSlamEvent {
    StartRequested,
    StopRequested,
    SensorsReady,
    SensorsNotReady,
    InitializationOk,
    InitializationFailed,
    MapQualityGood,
    MapQualityPoor,
    MotionCommandAccepted,
    MotionCommandRejected,
    MotionSettled,
    SensorTimeout,
    SafetyFault,
    MaxIterationReached
};

enum class AutonomousSlamFault {
    None,
    SensorTimeout,
    InitializationFailed,
    MotionRejected,
    MotionTransportFailed,
    MapQualityStuck,
    SafetyGateBlocked,
    InvalidStateTransition
};

struct RobotPose2D {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

struct TofScanFrame {
    double timestamp_s = 0.0;
    std::vector<double> ranges_m;
    double angle_min_rad = 0.0;
    double angle_increment_rad = 0.0;
    double max_range_m = 0.0;
    std::string frame_id;
};

struct ImuFrame {
    double timestamp_s = 0.0;
    double yaw_rate_rad_s = 0.0;
    double ax_mps2 = 0.0;
    double ay_mps2 = 0.0;
    double az_mps2 = 0.0;
};

struct WheelOdomFrame {
    double timestamp_s = 0.0;
    double linear_mps = 0.0;
    double angular_rad_s = 0.0;
    bool valid = false;
};

struct MultiTofSnapshot {
    bool has_front = false;
    bool has_left = false;
    bool has_right = false;
    TofScanFrame front;
    TofScanFrame left;
    TofScanFrame right;
    double synchronized_time_s = 0.0;
    int valid_tof_count = 0;
    bool degraded = false;
    double front_left_dt_s = 0.0;
    double front_right_dt_s = 0.0;
    double left_right_dt_s = 0.0;
    double multi_tof_imu_dt_s = 0.0;
    double multi_tof_wheel_dt_s = 0.0;
    std::string source = "multi_tof_snapshot_builder";
};

struct RobotSlamSensorSnapshot {
    bool has_tof = false;
    bool has_multi_tof = false;
    bool has_imu = false;
    bool has_wheel = false;
    TofScanFrame tof;
    MultiTofSnapshot multi_tof;
    ImuFrame imu;
    WheelOdomFrame wheel;
};

struct RobotSlamMapQuality {
    bool good_enough = false;
    double coverage_ratio = 0.0;
    double yaw_coverage_ratio = 0.0;
    int valid_scan_count = 0;
    std::string reason;
};

struct RobotSlamMotionFeedback {
    bool command_active = false;
    bool command_settled = true;
    bool emergency_stop_active = false;
    bool fault = false;
    std::string fault_reason;
    double timestamp_s = 0.0;
};

struct AutonomousSlamStepInput {
    double now_s = 0.0;
    bool start_requested = false;
    bool stop_requested = false;
    RobotSlamSensorSnapshot sensors;
    RobotSlamMotionFeedback motion_feedback;
};

struct AutonomousSlamStepOutput {
    AutonomousSlamState state = AutonomousSlamState::Idle;
    AutonomousSlamFault fault = AutonomousSlamFault::None;
    bool command_sent = false;
    bool completed = false;
    AlgorithmMotionCommand algorithm_command;
    SoftwareMotionCommand software_command;
    std::string message;
};

inline std::string to_string(AutonomousSlamState state) {
    switch (state) {
    case AutonomousSlamState::Idle:
        return "idle";
    case AutonomousSlamState::WaitingForSensors:
        return "waiting_for_sensors";
    case AutonomousSlamState::Initializing:
        return "initializing";
    case AutonomousSlamState::Mapping:
        return "mapping";
    case AutonomousSlamState::NeedActiveScan:
        return "need_active_scan";
    case AutonomousSlamState::SendingMotionCommand:
        return "sending_motion_command";
    case AutonomousSlamState::WaitingMotionSettle:
        return "waiting_motion_settle";
    case AutonomousSlamState::IntegratingScan:
        return "integrating_scan";
    case AutonomousSlamState::Completed:
        return "completed";
    case AutonomousSlamState::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(AutonomousSlamFault fault) {
    switch (fault) {
    case AutonomousSlamFault::None:
        return "none";
    case AutonomousSlamFault::SensorTimeout:
        return "sensor_timeout";
    case AutonomousSlamFault::InitializationFailed:
        return "initialization_failed";
    case AutonomousSlamFault::MotionRejected:
        return "motion_rejected";
    case AutonomousSlamFault::MotionTransportFailed:
        return "motion_transport_failed";
    case AutonomousSlamFault::MapQualityStuck:
        return "map_quality_stuck";
    case AutonomousSlamFault::SafetyGateBlocked:
        return "safety_gate_blocked";
    case AutonomousSlamFault::InvalidStateTransition:
        return "invalid_state_transition";
    }
    return "unknown";
}

inline int autonomous_slam_state_id(AutonomousSlamState state) {
    return static_cast<int>(state);
}

inline int autonomous_slam_fault_id(AutonomousSlamFault fault) {
    return static_cast<int>(fault);
}

} // namespace robot_slamd
