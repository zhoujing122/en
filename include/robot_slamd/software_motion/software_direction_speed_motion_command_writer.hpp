#pragma once
#include "robot_slamd/motion/motion_command_writer.hpp"
#include "robot_slamd/software_motion/software_motion_transport.hpp"

#include <memory>

namespace robot_slamd {

class SoftwareDirectionSpeedMotionCommandWriter final : public MotionCommandWriter {
public:
    struct Options {
        double max_internal_rpm_for_normalization = 30.0;
        double max_speed_normalized = 0.10;
        double command_ttl_s = 0.30;
        bool allow_rotation_commands = false;
        bool allow_translation_commands = false;
        bool allow_emergency_stop = true;
        bool require_opposite_wheel_sign_for_rotation = true;
        SoftwareMotionCommandSource source = SoftwareMotionCommandSource::ActiveScan;
    };

    SoftwareDirectionSpeedMotionCommandWriter(std::shared_ptr<SoftwareMotionCommandTransport> transport,
                                              Options options)
        : transport_(std::move(transport)), options_(options) {}

    MotionWriteResult write_zero(double timestamp_s) override {
        SoftwareMotionCommand command;
        command.direction = SoftwareMotionDirection::Stop;
        command.speed_normalized = 0.0;
        command.timestamp_s = timestamp_s;
        command.ttl_s = options_.command_ttl_s;
        command.source = SoftwareMotionCommandSource::SafetyStop;
        command.reason = "writer_zero";
        command.sequence = ++sequence_;
        return validate_and_send(command);
    }

    MotionWriteResult write_wheel_rpm(double timestamp_s,
                                      double left_rpm,
                                      double right_rpm) override {
        if (!std::isfinite(timestamp_s)) return {false, "non_finite_timestamp"};
        if (!std::isfinite(left_rpm) || !std::isfinite(right_rpm)) return {false, "non_finite_rpm"};
        if (!std::isfinite(options_.max_internal_rpm_for_normalization) ||
            options_.max_internal_rpm_for_normalization <= 0.0) {
            return {false, "invalid_normalization_rpm"};
        }
        if (std::fabs(left_rpm) <= 1e-12 && std::fabs(right_rpm) <= 1e-12) {
            return write_zero(timestamp_s);
        }

        if (options_.require_opposite_wheel_sign_for_rotation && left_rpm * right_rpm >= 0.0) {
            return {false, "translation_motion_disabled"};
        }

        SoftwareMotionCommand command;
        command.direction = direction_from_wheels(left_rpm, right_rpm);
        if (command.direction == SoftwareMotionDirection::Stop) return {false, "rotation_direction_ambiguous"};
        command.speed_normalized = std::min(options_.max_speed_normalized,
                                            std::max(0.0, std::max(std::fabs(left_rpm), std::fabs(right_rpm)) /
                                                             options_.max_internal_rpm_for_normalization));
        command.timestamp_s = timestamp_s;
        command.ttl_s = options_.command_ttl_s;
        command.source = options_.source;
        command.reason = "writer_wheel_rpm";
        command.sequence = ++sequence_;
        return validate_and_send(command);
    }

private:
    static SoftwareMotionDirection direction_from_wheels(double left_rpm, double right_rpm) {
        // Algorithm-layer convention from MotionSafetyExecutor: positive yaw has left wheel negative,
        // right wheel positive. M2-B1 must verify this sign convention on the real chassis.
        if (left_rpm < 0.0 && right_rpm > 0.0) return SoftwareMotionDirection::TurnLeft;
        if (left_rpm > 0.0 && right_rpm < 0.0) return SoftwareMotionDirection::TurnRight;
        return SoftwareMotionDirection::Stop;
    }

    MotionWriteResult validate_and_send(const SoftwareMotionCommand &command) {
        if (!transport_) return {false, "software_motion_transport_missing"};
        auto validation = validate_software_motion_command(command,
                                                           options_.max_speed_normalized,
                                                           options_.allow_translation_commands,
                                                           options_.allow_rotation_commands,
                                                           options_.allow_emergency_stop);
        if (!validation.ok) return {false, validation.error == "non_finite_timestamp" ? validation.error : "software_motion_command_invalid"};
        SoftwareMotionSendResult result = transport_->send_command(command);
        if (!result.ok) return {false, "software_motion_send_failed"};
        if (!result.accepted) return {false, "software_motion_rejected"};
        return {true, ""};
    }

    std::shared_ptr<SoftwareMotionCommandTransport> transport_;
    Options options_;
    uint64_t sequence_ = 0;
};

} // namespace robot_slamd
