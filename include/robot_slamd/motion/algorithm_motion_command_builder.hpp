#pragma once
#include "robot_slamd/motion/algorithm_motion_command.hpp"

namespace robot_slamd {

class AlgorithmMotionCommandBuilder {
public:
    struct Options {
        double default_ttl_s = 0.30;
        double direction_probe_speed = 0.03;
        double direction_probe_duration_s = 0.30;
        double active_scan_speed = 0.05;
        double active_scan_duration_s = 0.50;
        double recovery_speed = 0.05;
        double recovery_duration_s = 0.50;
        double manual_test_speed = 0.03;
        double manual_test_duration_s = 0.30;
    };

    AlgorithmMotionCommandBuilder() = default;

    explicit AlgorithmMotionCommandBuilder(Options options)
        : options_(options) {}

    AlgorithmMotionCommand stop(double timestamp_s, std::string reason = "stop") {
        return make_command(AlgorithmMotionKind::Stop,
                            AlgorithmMotionProfile::Safety,
                            timestamp_s,
                            0.0,
                            0.0,
                            std::move(reason),
                            false);
    }

    AlgorithmMotionCommand emergency_stop(double timestamp_s, std::string reason = "emergency_stop") {
        return make_command(AlgorithmMotionKind::EmergencyStop,
                            AlgorithmMotionProfile::Safety,
                            timestamp_s,
                            0.0,
                            0.0,
                            std::move(reason),
                            false);
    }

    AlgorithmMotionCommand direction_probe_left(double timestamp_s) {
        return make_command(AlgorithmMotionKind::DirectionProbeLeft,
                            AlgorithmMotionProfile::DirectionProbe,
                            timestamp_s,
                            options_.direction_probe_speed,
                            options_.direction_probe_duration_s,
                            "direction_probe_left");
    }

    AlgorithmMotionCommand direction_probe_right(double timestamp_s) {
        return make_command(AlgorithmMotionKind::DirectionProbeRight,
                            AlgorithmMotionProfile::DirectionProbe,
                            timestamp_s,
                            options_.direction_probe_speed,
                            options_.direction_probe_duration_s,
                            "direction_probe_right");
    }

    AlgorithmMotionCommand active_scan_turn_left(double timestamp_s) {
        return make_command(AlgorithmMotionKind::ActiveScanTurnLeft,
                            AlgorithmMotionProfile::ActiveScan,
                            timestamp_s,
                            options_.active_scan_speed,
                            options_.active_scan_duration_s,
                            "active_scan_turn_left");
    }

    AlgorithmMotionCommand active_scan_turn_right(double timestamp_s) {
        return make_command(AlgorithmMotionKind::ActiveScanTurnRight,
                            AlgorithmMotionProfile::ActiveScan,
                            timestamp_s,
                            options_.active_scan_speed,
                            options_.active_scan_duration_s,
                            "active_scan_turn_right");
    }

    AlgorithmMotionCommand recovery_forward(double timestamp_s) {
        return recovery_command(AlgorithmMotionKind::RecoveryForward, timestamp_s, "recovery_forward");
    }

    AlgorithmMotionCommand recovery_backward(double timestamp_s) {
        return recovery_command(AlgorithmMotionKind::RecoveryBackward, timestamp_s, "recovery_backward");
    }

    AlgorithmMotionCommand recovery_turn_left(double timestamp_s) {
        return recovery_command(AlgorithmMotionKind::RecoveryTurnLeft, timestamp_s, "recovery_turn_left");
    }

    AlgorithmMotionCommand recovery_turn_right(double timestamp_s) {
        return recovery_command(AlgorithmMotionKind::RecoveryTurnRight, timestamp_s, "recovery_turn_right");
    }

    AlgorithmMotionCommand manual_turn_left(double timestamp_s) {
        return manual_command(AlgorithmMotionKind::TurnLeft, timestamp_s, "manual_turn_left");
    }

    AlgorithmMotionCommand manual_turn_right(double timestamp_s) {
        return manual_command(AlgorithmMotionKind::TurnRight, timestamp_s, "manual_turn_right");
    }

    AlgorithmMotionCommand manual_forward(double timestamp_s) {
        return manual_command(AlgorithmMotionKind::Forward, timestamp_s, "manual_forward");
    }

    AlgorithmMotionCommand manual_backward(double timestamp_s) {
        return manual_command(AlgorithmMotionKind::Backward, timestamp_s, "manual_backward");
    }

private:
    AlgorithmMotionCommand recovery_command(AlgorithmMotionKind kind,
                                            double timestamp_s,
                                            const std::string &reason) {
        return make_command(kind,
                            AlgorithmMotionProfile::Recovery,
                            timestamp_s,
                            options_.recovery_speed,
                            options_.recovery_duration_s,
                            reason);
    }

    AlgorithmMotionCommand manual_command(AlgorithmMotionKind kind,
                                          double timestamp_s,
                                          const std::string &reason) {
        return make_command(kind,
                            AlgorithmMotionProfile::ManualTest,
                            timestamp_s,
                            options_.manual_test_speed,
                            options_.manual_test_duration_s,
                            reason);
    }

    AlgorithmMotionCommand make_command(AlgorithmMotionKind kind,
                                        AlgorithmMotionProfile profile,
                                        double timestamp_s,
                                        double speed,
                                        double duration_s,
                                        std::string reason,
                                        bool assign_sequence = true) {
        AlgorithmMotionCommand command;
        command.kind = kind;
        command.profile = profile;
        command.speed_normalized = speed;
        command.duration_s = duration_s;
        command.timestamp_s = timestamp_s;
        command.ttl_s = options_.default_ttl_s;
        command.reason = std::move(reason);
        command.sequence = assign_sequence ? ++next_sequence_ : 0;
        return command;
    }

    uint64_t next_sequence_ = 0;
    Options options_;
};

} // namespace robot_slamd
