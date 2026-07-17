#pragma once
#include "robot_slamd/motion/algorithm_motion_command.hpp"
#include "robot_slamd/software_motion/software_motion_command.hpp"

namespace robot_slamd {

struct AlgorithmToSoftwareMotionOptions {
    bool allow_translation_commands = false;
    bool allow_rotation_commands = true;
    bool allow_recovery_commands = false;
    bool allow_manual_test_commands = false;

    double max_direction_probe_speed = 0.03;
    double max_active_scan_speed = 0.05;
    double max_recovery_speed = 0.05;
    double max_manual_test_speed = 0.03;

    double max_direction_probe_duration_s = 0.30;
    double max_active_scan_duration_s = 0.50;
    double max_recovery_duration_s = 0.50;
    double max_manual_test_duration_s = 0.30;
};

struct AlgorithmToSoftwareMotionResult {
    bool ok = false;
    std::string error;
    SoftwareMotionCommand command;
};

class AlgorithmMotionCommandAdapter {
public:
    AlgorithmMotionCommandAdapter() = default;

    explicit AlgorithmMotionCommandAdapter(AlgorithmToSoftwareMotionOptions options)
        : options_(options) {}

    AlgorithmToSoftwareMotionResult to_software_command(
        const AlgorithmMotionCommand &command) const {
        AlgorithmToSoftwareMotionResult result;

        // Algorithm validation.
        auto validation = validate_algorithm_motion_command(command,
                                                            options_.max_direction_probe_speed,
                                                            options_.max_active_scan_speed,
                                                            options_.max_recovery_speed,
                                                            options_.max_manual_test_speed,
                                                            options_.max_direction_probe_duration_s,
                                                            options_.max_active_scan_duration_s,
                                                            options_.max_recovery_duration_s,
                                                            options_.max_manual_test_duration_s,
                                                            options_.allow_translation_commands,
                                                            options_.allow_rotation_commands,
                                                            options_.allow_recovery_commands,
                                                            options_.allow_manual_test_commands);
        if (!validation.ok) {
            result.error = validation.error;
            return result;
        }

        // Field mapping.
        result.command.direction = map_direction(command.kind);
        result.command.speed_normalized = command.speed_normalized;
        result.command.timestamp_s = command.timestamp_s;
        result.command.ttl_s = command.ttl_s;
        result.command.source = map_source(command);
        result.command.reason = command.reason;
        result.command.sequence = command.sequence;
        result.command.duration_s = command.duration_s;

        // Software validation.
        auto software_validation = validate_software_motion_command(result.command,
                                                                    max_software_speed(command.profile),
                                                                    options_.allow_translation_commands,
                                                                    options_.allow_rotation_commands,
                                                                    true);
        if (!software_validation.ok) {
            result.error = "software_motion_command_invalid:" + software_validation.error;
            return result;
        }

        // Success.
        result.ok = true;
        return result;
    }

private:
    static SoftwareMotionDirection map_direction(AlgorithmMotionKind kind) {
        switch (kind) {
        case AlgorithmMotionKind::Stop:
            return SoftwareMotionDirection::Stop;
        case AlgorithmMotionKind::EmergencyStop:
            return SoftwareMotionDirection::EmergencyStop;
        case AlgorithmMotionKind::TurnLeft:
        case AlgorithmMotionKind::DirectionProbeLeft:
        case AlgorithmMotionKind::ActiveScanTurnLeft:
        case AlgorithmMotionKind::RecoveryTurnLeft:
            return SoftwareMotionDirection::TurnLeft;
        case AlgorithmMotionKind::TurnRight:
        case AlgorithmMotionKind::DirectionProbeRight:
        case AlgorithmMotionKind::ActiveScanTurnRight:
        case AlgorithmMotionKind::RecoveryTurnRight:
            return SoftwareMotionDirection::TurnRight;
        case AlgorithmMotionKind::Forward:
        case AlgorithmMotionKind::RecoveryForward:
            return SoftwareMotionDirection::Forward;
        case AlgorithmMotionKind::Backward:
        case AlgorithmMotionKind::RecoveryBackward:
            return SoftwareMotionDirection::Backward;
        }
        return SoftwareMotionDirection::Stop;
    }

    static SoftwareMotionCommandSource map_source(const AlgorithmMotionCommand &command) {
        if (command.kind == AlgorithmMotionKind::Stop ||
            command.kind == AlgorithmMotionKind::EmergencyStop) {
            return SoftwareMotionCommandSource::SafetyStop;
        }
        if (command.profile == AlgorithmMotionProfile::DirectionProbe ||
            command.profile == AlgorithmMotionProfile::ManualTest) {
            return SoftwareMotionCommandSource::ManualTest;
        }
        if (command.profile == AlgorithmMotionProfile::ActiveScan) {
            return SoftwareMotionCommandSource::ActiveScan;
        }
        if (command.profile == AlgorithmMotionProfile::Recovery) {
            return SoftwareMotionCommandSource::Recovery;
        }
        return SoftwareMotionCommandSource::Unknown;
    }

    double max_software_speed(AlgorithmMotionProfile profile) const {
        switch (profile) {
        case AlgorithmMotionProfile::DirectionProbe:
            return options_.max_direction_probe_speed;
        case AlgorithmMotionProfile::ActiveScan:
            return options_.max_active_scan_speed;
        case AlgorithmMotionProfile::Recovery:
            return options_.max_recovery_speed;
        case AlgorithmMotionProfile::ManualTest:
            return options_.max_manual_test_speed;
        case AlgorithmMotionProfile::Safety:
            return 0.0;
        }
        return 0.0;
    }

    AlgorithmToSoftwareMotionOptions options_;
};

} // namespace robot_slamd
