#pragma once
#include "robot_slamd/core/common.hpp"

namespace robot_slamd {

enum class SoftwareMotionDirection {
    Stop,
    TurnLeft,
    TurnRight,
    Forward,
    Backward,
    EmergencyStop,
};

enum class SoftwareMotionCommandSource {
    Unknown,
    ActiveScan,
    Recovery,
    ManualTest,
    SafetyStop,
};

struct SoftwareMotionCommand {
    SoftwareMotionDirection direction = SoftwareMotionDirection::Stop;
    double speed_normalized = 0.0;
    double timestamp_s = 0.0;
    double ttl_s = 0.0;
    SoftwareMotionCommandSource source = SoftwareMotionCommandSource::Unknown;
    std::string reason;
    uint64_t sequence = 0;
};

struct SoftwareMotionCommandValidationResult {
    bool ok = false;
    std::string error;
};

inline std::string to_string(SoftwareMotionDirection direction) {
    switch (direction) {
    case SoftwareMotionDirection::Stop: return "STOP";
    case SoftwareMotionDirection::TurnLeft: return "TURN_LEFT";
    case SoftwareMotionDirection::TurnRight: return "TURN_RIGHT";
    case SoftwareMotionDirection::Forward: return "FORWARD";
    case SoftwareMotionDirection::Backward: return "BACKWARD";
    case SoftwareMotionDirection::EmergencyStop: return "EMERGENCY_STOP";
    }
    return "STOP";
}

inline std::string to_string(SoftwareMotionCommandSource source) {
    switch (source) {
    case SoftwareMotionCommandSource::Unknown: return "UNKNOWN";
    case SoftwareMotionCommandSource::ActiveScan: return "ACTIVE_SCAN";
    case SoftwareMotionCommandSource::Recovery: return "RECOVERY";
    case SoftwareMotionCommandSource::ManualTest: return "MANUAL_TEST";
    case SoftwareMotionCommandSource::SafetyStop: return "SAFETY_STOP";
    }
    return "UNKNOWN";
}

inline int software_motion_direction_id(SoftwareMotionDirection direction) {
    switch (direction) {
    case SoftwareMotionDirection::Stop: return 0;
    case SoftwareMotionDirection::TurnLeft: return 1;
    case SoftwareMotionDirection::TurnRight: return 2;
    case SoftwareMotionDirection::Forward: return 3;
    case SoftwareMotionDirection::Backward: return 4;
    case SoftwareMotionDirection::EmergencyStop: return 5;
    }
    return 0;
}

inline SoftwareMotionCommandValidationResult validate_software_motion_command(
    const SoftwareMotionCommand &command,
    double max_speed_normalized,
    bool allow_translation_commands,
    bool allow_rotation_commands,
    bool allow_emergency_stop) {
    if (!std::isfinite(command.timestamp_s)) return {false, "non_finite_timestamp"};
    if (!std::isfinite(command.ttl_s) || command.ttl_s <= 0.0) return {false, "invalid_ttl"};
    if (!std::isfinite(command.speed_normalized)) return {false, "non_finite_speed"};
    if (command.speed_normalized < 0.0 || command.speed_normalized > 1.0) return {false, "speed_out_of_range"};

    if (command.direction == SoftwareMotionDirection::Stop) {
        if (std::fabs(command.speed_normalized) > 1e-12) return {false, "stop_requires_zero_speed"};
        return {true, ""};
    }
    if (command.direction == SoftwareMotionDirection::EmergencyStop) {
        if (!allow_emergency_stop) return {false, "rotation_commands_disabled"};
        if (std::fabs(command.speed_normalized) > 1e-12) return {false, "emergency_stop_requires_zero_speed"};
        return {true, ""};
    }
    if (command.speed_normalized > max_speed_normalized) return {false, "speed_exceeds_limit"};
    if ((command.direction == SoftwareMotionDirection::TurnLeft || command.direction == SoftwareMotionDirection::TurnRight) &&
        !allow_rotation_commands) {
        return {false, "rotation_commands_disabled"};
    }
    if ((command.direction == SoftwareMotionDirection::Forward || command.direction == SoftwareMotionDirection::Backward) &&
        !allow_translation_commands) {
        return {false, "translation_commands_disabled"};
    }
    if (command.source == SoftwareMotionCommandSource::Unknown) return {false, "source_required_for_motion"};
    return {true, ""};
}

} // namespace robot_slamd
