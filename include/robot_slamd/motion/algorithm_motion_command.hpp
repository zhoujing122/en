#pragma once
#include "robot_slamd/core/common.hpp"

namespace robot_slamd {

enum class AlgorithmMotionKind {
    Stop,
    EmergencyStop,
    TurnLeft,
    TurnRight,
    Forward,
    Backward,
    DirectionProbeLeft,
    DirectionProbeRight,
    ActiveScanTurnLeft,
    ActiveScanTurnRight,
    RecoveryForward,
    RecoveryBackward,
    RecoveryTurnLeft,
    RecoveryTurnRight,
};

enum class AlgorithmMotionProfile {
    Safety,
    DirectionProbe,
    ActiveScan,
    Recovery,
    ManualTest,
};

struct AlgorithmMotionCommand {
    AlgorithmMotionKind kind = AlgorithmMotionKind::Stop;
    AlgorithmMotionProfile profile = AlgorithmMotionProfile::Safety;
    double speed_normalized = 0.0;
    double duration_s = 0.0;
    double timestamp_s = 0.0;
    double ttl_s = 0.0;
    std::string reason;
    uint64_t sequence = 0;
};

struct AlgorithmMotionValidationResult {
    bool ok = false;
    std::string error;
};

inline std::string to_string(AlgorithmMotionKind kind) {
    switch (kind) {
    case AlgorithmMotionKind::Stop:
        return "STOP";
    case AlgorithmMotionKind::EmergencyStop:
        return "EMERGENCY_STOP";
    case AlgorithmMotionKind::TurnLeft:
        return "TURN_LEFT";
    case AlgorithmMotionKind::TurnRight:
        return "TURN_RIGHT";
    case AlgorithmMotionKind::Forward:
        return "FORWARD";
    case AlgorithmMotionKind::Backward:
        return "BACKWARD";
    case AlgorithmMotionKind::DirectionProbeLeft:
        return "DIRECTION_PROBE_LEFT";
    case AlgorithmMotionKind::DirectionProbeRight:
        return "DIRECTION_PROBE_RIGHT";
    case AlgorithmMotionKind::ActiveScanTurnLeft:
        return "ACTIVE_SCAN_TURN_LEFT";
    case AlgorithmMotionKind::ActiveScanTurnRight:
        return "ACTIVE_SCAN_TURN_RIGHT";
    case AlgorithmMotionKind::RecoveryForward:
        return "RECOVERY_FORWARD";
    case AlgorithmMotionKind::RecoveryBackward:
        return "RECOVERY_BACKWARD";
    case AlgorithmMotionKind::RecoveryTurnLeft:
        return "RECOVERY_TURN_LEFT";
    case AlgorithmMotionKind::RecoveryTurnRight:
        return "RECOVERY_TURN_RIGHT";
    }
    return "STOP";
}

inline std::string to_string(AlgorithmMotionProfile profile) {
    switch (profile) {
    case AlgorithmMotionProfile::Safety:
        return "SAFETY";
    case AlgorithmMotionProfile::DirectionProbe:
        return "DIRECTION_PROBE";
    case AlgorithmMotionProfile::ActiveScan:
        return "ACTIVE_SCAN";
    case AlgorithmMotionProfile::Recovery:
        return "RECOVERY";
    case AlgorithmMotionProfile::ManualTest:
        return "MANUAL_TEST";
    }
    return "SAFETY";
}

inline int algorithm_motion_kind_id(AlgorithmMotionKind kind) {
    switch (kind) {
    case AlgorithmMotionKind::Stop:
        return 0;
    case AlgorithmMotionKind::EmergencyStop:
        return 1;
    case AlgorithmMotionKind::TurnLeft:
        return 2;
    case AlgorithmMotionKind::TurnRight:
        return 3;
    case AlgorithmMotionKind::Forward:
        return 4;
    case AlgorithmMotionKind::Backward:
        return 5;
    case AlgorithmMotionKind::DirectionProbeLeft:
        return 6;
    case AlgorithmMotionKind::DirectionProbeRight:
        return 7;
    case AlgorithmMotionKind::ActiveScanTurnLeft:
        return 8;
    case AlgorithmMotionKind::ActiveScanTurnRight:
        return 9;
    case AlgorithmMotionKind::RecoveryForward:
        return 10;
    case AlgorithmMotionKind::RecoveryBackward:
        return 11;
    case AlgorithmMotionKind::RecoveryTurnLeft:
        return 12;
    case AlgorithmMotionKind::RecoveryTurnRight:
        return 13;
    }
    return 0;
}

inline int algorithm_motion_profile_id(AlgorithmMotionProfile profile) {
    switch (profile) {
    case AlgorithmMotionProfile::Safety:
        return 0;
    case AlgorithmMotionProfile::DirectionProbe:
        return 1;
    case AlgorithmMotionProfile::ActiveScan:
        return 2;
    case AlgorithmMotionProfile::Recovery:
        return 3;
    case AlgorithmMotionProfile::ManualTest:
        return 4;
    }
    return 0;
}

inline bool is_algorithm_rotation_kind(AlgorithmMotionKind kind) {
    return kind == AlgorithmMotionKind::TurnLeft ||
           kind == AlgorithmMotionKind::TurnRight ||
           kind == AlgorithmMotionKind::DirectionProbeLeft ||
           kind == AlgorithmMotionKind::DirectionProbeRight ||
           kind == AlgorithmMotionKind::ActiveScanTurnLeft ||
           kind == AlgorithmMotionKind::ActiveScanTurnRight ||
           kind == AlgorithmMotionKind::RecoveryTurnLeft ||
           kind == AlgorithmMotionKind::RecoveryTurnRight;
}

inline bool is_algorithm_translation_kind(AlgorithmMotionKind kind) {
    return kind == AlgorithmMotionKind::Forward ||
           kind == AlgorithmMotionKind::Backward ||
           kind == AlgorithmMotionKind::RecoveryForward ||
           kind == AlgorithmMotionKind::RecoveryBackward;
}

inline bool is_algorithm_recovery_kind(AlgorithmMotionKind kind) {
    return kind == AlgorithmMotionKind::RecoveryForward ||
           kind == AlgorithmMotionKind::RecoveryBackward ||
           kind == AlgorithmMotionKind::RecoveryTurnLeft ||
           kind == AlgorithmMotionKind::RecoveryTurnRight;
}

inline bool is_algorithm_stop_kind(AlgorithmMotionKind kind) {
    return kind == AlgorithmMotionKind::Stop ||
           kind == AlgorithmMotionKind::EmergencyStop;
}

inline AlgorithmMotionValidationResult validate_algorithm_motion_command(
    const AlgorithmMotionCommand &command,
    double max_direction_probe_speed,
    double max_active_scan_speed,
    double max_recovery_speed,
    double max_manual_test_speed,
    double max_direction_probe_duration,
    double max_active_scan_duration,
    double max_recovery_duration,
    double max_manual_test_duration,
    bool allow_translation_commands,
    bool allow_rotation_commands,
    bool allow_recovery_commands,
    bool allow_manual_test_commands) {
    // Finite checks.
    if (!std::isfinite(command.timestamp_s)) {
        return {false, "non_finite_timestamp"};
    }
    if (!std::isfinite(command.ttl_s) || command.ttl_s <= 0.0) {
        return {false, "invalid_ttl"};
    }
    if (!std::isfinite(command.duration_s)) {
        return {false, "non_finite_duration"};
    }
    if (command.duration_s < 0.0) {
        return {false, "negative_duration"};
    }
    if (!std::isfinite(command.speed_normalized)) {
        return {false, "non_finite_speed"};
    }
    if (command.speed_normalized < 0.0 || command.speed_normalized > 1.0) {
        return {false, "speed_out_of_range"};
    }

    // Stop / emergency stop checks.
    if (command.kind == AlgorithmMotionKind::Stop) {
        if (std::fabs(command.speed_normalized) > 1e-12) {
            return {false, "stop_requires_zero_speed"};
        }
        return {true, ""};
    }
    if (command.kind == AlgorithmMotionKind::EmergencyStop) {
        if (std::fabs(command.speed_normalized) > 1e-12) {
            return {false, "emergency_stop_requires_zero_speed"};
        }
        return {true, ""};
    }

    // Duration / reason checks.
    if (command.duration_s <= 0.0) {
        return {false, "motion_requires_positive_duration"};
    }
    if (command.reason.empty()) {
        return {false, "motion_reason_required"};
    }

    // Profile speed/duration limits.
    if (command.profile == AlgorithmMotionProfile::DirectionProbe) {
        if (command.speed_normalized > max_direction_probe_speed) {
            return {false, "direction_probe_speed_too_high"};
        }
        if (command.duration_s > max_direction_probe_duration) {
            return {false, "direction_probe_duration_too_long"};
        }
    } else if (command.profile == AlgorithmMotionProfile::ActiveScan) {
        if (command.speed_normalized > max_active_scan_speed) {
            return {false, "active_scan_speed_too_high"};
        }
        if (command.duration_s > max_active_scan_duration) {
            return {false, "active_scan_duration_too_long"};
        }
    } else if (command.profile == AlgorithmMotionProfile::Recovery) {
        if (command.speed_normalized > max_recovery_speed) {
            return {false, "recovery_speed_too_high"};
        }
        if (command.duration_s > max_recovery_duration) {
            return {false, "recovery_duration_too_long"};
        }
    } else if (command.profile == AlgorithmMotionProfile::ManualTest) {
        if (command.speed_normalized > max_manual_test_speed) {
            return {false, "manual_test_speed_too_high"};
        }
        if (command.duration_s > max_manual_test_duration) {
            return {false, "manual_test_duration_too_long"};
        }
    }

    // Feature gate checks.
    if (is_algorithm_recovery_kind(command.kind) && !allow_recovery_commands) {
        return {false, "recovery_commands_disabled"};
    }
    if (command.profile == AlgorithmMotionProfile::ManualTest && !allow_manual_test_commands) {
        return {false, "manual_test_commands_disabled"};
    }
    if (is_algorithm_rotation_kind(command.kind) && !allow_rotation_commands) {
        return {false, "rotation_commands_disabled"};
    }
    if (is_algorithm_translation_kind(command.kind) && !allow_translation_commands) {
        return {false, "translation_commands_disabled"};
    }
    return {true, ""};
}

} // namespace robot_slamd
