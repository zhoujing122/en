#pragma once
#include "robot_slamd/core/common.hpp"

namespace robot_slamd {

enum class MotionHardwarePreflightMode {
    LiftedDirectionProbe,
    ConfirmedLiftedLive
};

struct MotionHardwarePreflightInput {
    MotionHardwarePreflightMode mode = MotionHardwarePreflightMode::ConfirmedLiftedLive;
    bool operator_present = false;
    bool robot_lifted_or_wheels_free = false;
    bool emergency_stop_available = false;
    bool software_transport_ready = false;
    bool software_transport_shadow_mode_verified = false;
    bool stop_command_verified = false;
    bool ttl_stop_verified = false;
    bool ack_semantics_verified = false;
    bool left_right_direction_convention_verified = false;
    bool encoder_feedback_healthy = false;
    bool motor_status_zero = false;
    bool current_within_limit = false;
    bool obstacle_clear = false;
    double requested_max_speed_normalized = 0.0;
    double requested_max_duration_s = 0.0;
};

struct MotionHardwarePreflightResult {
    bool ok = false;
    bool direction_convention_pending = false;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

inline MotionHardwarePreflightResult run_motion_hardware_preflight(const MotionHardwarePreflightInput &input) {
    MotionHardwarePreflightResult out;
    auto add = [&](const std::string &e) { out.errors.push_back(e); };
    auto warn = [&](const std::string &w) { out.warnings.push_back(w); };
    if (!input.operator_present) add("operator_not_present");
    if (!input.robot_lifted_or_wheels_free) add("robot_not_lifted_or_wheels_not_free");
    if (!input.emergency_stop_available) add("emergency_stop_missing");
    if (!input.software_transport_ready) add("software_transport_not_ready");
    if (!input.software_transport_shadow_mode_verified) add("shadow_mode_not_verified");
    if (!input.stop_command_verified) add("stop_command_not_verified");
    if (!input.ttl_stop_verified) add("ttl_stop_not_verified");
    if (!input.ack_semantics_verified) add("ack_semantics_not_verified");
    if (!input.encoder_feedback_healthy) add("encoder_feedback_unhealthy");
    if (!input.motor_status_zero) add("motor_status_nonzero");
    if (!input.current_within_limit) add("current_limit_not_ok");
    if (!input.obstacle_clear) add("obstacle_not_clear");
    if (!std::isfinite(input.requested_max_speed_normalized) || input.requested_max_speed_normalized <= 0.0) add("invalid_requested_speed");
    if (!std::isfinite(input.requested_max_duration_s) || input.requested_max_duration_s <= 0.0) add("invalid_requested_duration");

    switch (input.mode) {
    case MotionHardwarePreflightMode::LiftedDirectionProbe:
        if (!input.left_right_direction_convention_verified) {
            out.direction_convention_pending = true;
            warn("direction_convention_pending");
        }
        if (std::isfinite(input.requested_max_speed_normalized) &&
            input.requested_max_speed_normalized > 0.03) {
            add("direction_probe_speed_too_high");
        }
        if (std::isfinite(input.requested_max_duration_s) &&
            input.requested_max_duration_s > 0.30) {
            add("direction_probe_duration_too_long");
        }
        break;
    case MotionHardwarePreflightMode::ConfirmedLiftedLive:
        if (!input.left_right_direction_convention_verified) add("direction_convention_not_verified");
        if (std::isfinite(input.requested_max_speed_normalized) &&
            input.requested_max_speed_normalized > 0.05) {
            add("requested_speed_too_high");
        }
        if (std::isfinite(input.requested_max_duration_s) &&
            input.requested_max_duration_s > 0.5) {
            add("requested_duration_too_long");
        }
        break;
    default:
        add("invalid_preflight_mode");
        break;
    }
    out.ok = out.errors.empty();
    return out;
}

} // namespace robot_slamd
