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

        // Timestamp / ttl validation.
        if (!is_finite(command.timestamp_s)) {
            return make_error("non_finite_timestamp");
        }
        if (!is_finite(command.ttl_s) || command.ttl_s <= 0.0) {
            return make_error("invalid_ttl");
        }
        if (command.ttl_s > options_.max_ttl_s) {
            return make_error("ttl_too_large");
        }

        // Speed validation.
        if (!is_finite(command.speed_normalized)) {
            return make_error("non_finite_speed");
        }
        if (command.speed_normalized < 0.0 || command.speed_normalized > 1.0) {
            return make_error("speed_out_of_range");
        }

        // Direction gate validation.
        if (command.direction == SoftwareMotionDirection::Stop) {
            if (std::fabs(command.speed_normalized) > 1e-12) {
                return make_error("stop_requires_zero_speed");
            }
        } else if (command.direction == SoftwareMotionDirection::EmergencyStop) {
            if (!options_.allow_emergency_stop) {
                return make_error("emergency_stop_disabled");
            }
            if (std::fabs(command.speed_normalized) > 1e-12) {
                return make_error("emergency_stop_requires_zero_speed");
            }
        } else {
            if (command.speed_normalized > options_.max_speed_normalized) {
                return make_error("speed_exceeds_limit");
            }
            if (is_translation(command.direction) && !options_.allow_translation_commands) {
                return make_error("translation_commands_disabled");
            }
            if (is_rotation(command.direction) && !options_.allow_rotation_commands) {
                return make_error("rotation_commands_disabled");
            }
            if (command.sequence == 0) {
                append_warning(result, "sequence_zero_for_motion");
            }
        }

        // Command age validation.
        if (is_finite(now_s) && now_s - command.timestamp_s > options_.max_command_age_s) {
            result.error = "command_too_old";
            result.ok = false;
            return result;
        }

        // Reason / sequence validation.
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
        // Result timestamp validation.
        if (!is_finite(send_result.timestamp_s)) {
            return make_error("non_finite_result_timestamp");
        }
        if (send_result.timestamp_s + 1e-9 < command.timestamp_s) {
            return make_error("result_before_command");
        }

        // Error semantics validation.
        if (!send_result.ok && send_result.error.empty()) {
            return make_error("failed_result_requires_error");
        }
        if (send_result.ok && !send_result.accepted && send_result.error.empty()) {
            return make_error("rejected_result_requires_error");
        }

        // Transport sequence validation.
        if (have_last_transport_sequence_ &&
            send_result.transport_sequence < last_transport_sequence_) {
            return make_error("transport_sequence_regressed");
        }

        have_last_transport_sequence_ = true;
        last_transport_sequence_ = send_result.transport_sequence;
        return make_ok();
    }

private:
    static SoftwareMotionTransportContractCheckResult make_ok() {
        return {true, "", {}};
    }

    static SoftwareMotionTransportContractCheckResult make_error(
        const std::string &error) {
        return {false, error, {}};
    }

    static void append_warning(SoftwareMotionTransportContractCheckResult &result,
                               const std::string &warning) {
        result.warnings.push_back(warning);
    }

    static bool is_stop_like(SoftwareMotionDirection direction) {
        return direction == SoftwareMotionDirection::Stop ||
               direction == SoftwareMotionDirection::EmergencyStop;
    }

    static bool is_translation(SoftwareMotionDirection direction) {
        return direction == SoftwareMotionDirection::Forward ||
               direction == SoftwareMotionDirection::Backward;
    }

    static bool is_rotation(SoftwareMotionDirection direction) {
        return direction == SoftwareMotionDirection::TurnLeft ||
               direction == SoftwareMotionDirection::TurnRight;
    }

    static bool is_finite(double value) {
        return std::isfinite(value);
    }

    SoftwareMotionTransportContractOptions options_;
    mutable bool have_last_transport_sequence_ = false;
    mutable uint64_t last_transport_sequence_ = 0;
};

} // namespace robot_slamd
