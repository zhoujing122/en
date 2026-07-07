#pragma once
#include "robot_slamd/software_motion/software_motion_transport.hpp"

namespace robot_slamd {

struct SoftwareMotionTransportContractOptions {
    bool require_shadow_mode = true;
    bool allow_rotation_commands = true;
    bool allow_translation_commands = false;
    bool allow_emergency_stop = true;
    double max_speed_normalized = 0.05;
    double max_direction_probe_speed = 0.03;
    double max_ttl_s = 0.50;
    double max_command_age_s = 0.50;
};

struct SoftwareMotionTransportContractCheckResult {
    bool ok = false;
    std::string error;
    std::vector<std::string> warnings;
};

class SoftwareMotionTransportContractChecker {
public:
    explicit SoftwareMotionTransportContractChecker(
        SoftwareMotionTransportContractOptions options = {})
        : options_(options) {}

    SoftwareMotionTransportContractCheckResult check_command(
        const SoftwareMotionCommand &command,
        double now_s) const {
        SoftwareMotionTransportContractCheckResult result;

        if (!std::isfinite(command.timestamp_s)) {
            result.error = "non_finite_timestamp";
            return result;
        }
        if (!std::isfinite(command.ttl_s) || command.ttl_s <= 0.0) {
            result.error = "invalid_ttl";
            return result;
        }
        if (command.ttl_s > options_.max_ttl_s) {
            result.error = "ttl_too_large";
            return result;
        }
        if (!std::isfinite(command.speed_normalized)) {
            result.error = "non_finite_speed";
            return result;
        }
        if (command.speed_normalized < 0.0 || command.speed_normalized > 1.0) {
            result.error = "speed_out_of_range";
            return result;
        }

        if (command.direction == SoftwareMotionDirection::Stop) {
            if (std::fabs(command.speed_normalized) > 1e-12) {
                result.error = "stop_requires_zero_speed";
                return result;
            }
        } else if (command.direction == SoftwareMotionDirection::EmergencyStop) {
            if (!options_.allow_emergency_stop) {
                result.error = "emergency_stop_disabled";
                return result;
            }
            if (std::fabs(command.speed_normalized) > 1e-12) {
                result.error = "emergency_stop_requires_zero_speed";
                return result;
            }
        } else {
            if (command.speed_normalized > options_.max_speed_normalized) {
                result.error = "speed_exceeds_limit";
                return result;
            }
            if (is_translation(command.direction) && !options_.allow_translation_commands) {
                result.error = "translation_commands_disabled";
                return result;
            }
            if (is_rotation(command.direction) && !options_.allow_rotation_commands) {
                result.error = "rotation_commands_disabled";
                return result;
            }
            if (command.sequence == 0) {
                result.warnings.push_back("sequence_zero_for_motion");
            }
        }

        if (std::isfinite(now_s) && now_s - command.timestamp_s > options_.max_command_age_s) {
            result.error = "command_too_old";
            result.ok = false;
            return result;
        }
        if (command.reason.empty()) {
            result.error = "reason_required";
            return result;
        }

        result.ok = true;
        return result;
    }

    SoftwareMotionTransportContractCheckResult check_send_result(
        const SoftwareMotionCommand &command,
        const SoftwareMotionSendResult &send_result,
        double /*now_s*/) const {
        SoftwareMotionTransportContractCheckResult result;

        if (!std::isfinite(send_result.timestamp_s)) {
            result.error = "non_finite_result_timestamp";
            return result;
        }
        if (send_result.timestamp_s + 1e-9 < command.timestamp_s) {
            result.error = "result_before_command";
            return result;
        }
        if (!send_result.ok && send_result.error.empty()) {
            result.error = "failed_result_requires_error";
            return result;
        }
        if (send_result.ok && !send_result.accepted && send_result.error.empty()) {
            result.error = "rejected_result_requires_error";
            return result;
        }
        if (have_last_transport_sequence_ &&
            send_result.transport_sequence < last_transport_sequence_) {
            result.error = "transport_sequence_regressed";
            return result;
        }

        have_last_transport_sequence_ = true;
        last_transport_sequence_ = send_result.transport_sequence;
        result.ok = true;
        return result;
    }

private:
    static bool is_rotation(SoftwareMotionDirection direction) {
        return direction == SoftwareMotionDirection::TurnLeft ||
               direction == SoftwareMotionDirection::TurnRight;
    }

    static bool is_translation(SoftwareMotionDirection direction) {
        return direction == SoftwareMotionDirection::Forward ||
               direction == SoftwareMotionDirection::Backward;
    }

    SoftwareMotionTransportContractOptions options_;
    mutable bool have_last_transport_sequence_ = false;
    mutable uint64_t last_transport_sequence_ = 0;
};

} // namespace robot_slamd
