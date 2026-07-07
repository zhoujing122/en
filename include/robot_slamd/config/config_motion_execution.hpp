#pragma once
#include "robot_slamd/core/common.hpp"

inline void parse_motion_execution_config(Config &c,
                                          const std::unordered_map<std::string, std::string> &kv) {
    // basic motion execution
    c.motion_execution_enabled = get_bool(kv, "motion_execution.enabled", c.motion_execution_enabled);
    c.motion_execution_mode = get_string(kv, "motion_execution.mode", c.motion_execution_mode);
    c.motion_execution_log_hz = get_double(kv, "motion_execution.log_hz", c.motion_execution_log_hz);
    c.motion_execution_command_source = get_string(kv,
                                                   "motion_execution.command_source",
                                                   c.motion_execution_command_source);
    c.motion_execution_allow_recovery_scan_request_as_reason = get_bool(
        kv,
        "motion_execution.allow_recovery_scan_request_as_reason",
        c.motion_execution_allow_recovery_scan_request_as_reason);
    c.motion_execution_require_active_scan_command = get_bool(
        kv,
        "motion_execution.require_active_scan_command",
        c.motion_execution_require_active_scan_command);
    c.motion_execution_require_command_planner_verifying_or_commanding = get_bool(
        kv,
        "motion_execution.require_command_planner_verifying_or_commanding",
        c.motion_execution_require_command_planner_verifying_or_commanding);

    // safety limits
    c.motion_execution_wheel_base_m = get_double(kv,
                                                 "motion_execution.wheel_base_m",
                                                 c.motion_execution_wheel_base_m);
    c.motion_execution_wheel_radius_m = get_double(kv,
                                                   "motion_execution.wheel_radius_m",
                                                   c.motion_execution_wheel_radius_m);
    c.motion_execution_max_linear_speed_mps = get_double(
        kv,
        "motion_execution.max_linear_speed_mps",
        c.motion_execution_max_linear_speed_mps);
    c.motion_execution_max_abs_yaw_rate_dps = get_double(
        kv,
        "motion_execution.max_abs_yaw_rate_dps",
        c.motion_execution_max_abs_yaw_rate_dps);
    c.motion_execution_max_wheel_speed_rpm = get_double(
        kv,
        "motion_execution.max_wheel_speed_rpm",
        c.motion_execution_max_wheel_speed_rpm);
    c.motion_execution_min_wheel_speed_rpm = get_double(
        kv,
        "motion_execution.min_wheel_speed_rpm",
        c.motion_execution_min_wheel_speed_rpm);
    c.motion_execution_max_command_duration_s = get_double(
        kv,
        "motion_execution.max_command_duration_s",
        c.motion_execution_max_command_duration_s);
    c.motion_execution_deadman_timeout_s = get_double(kv,
                                                      "motion_execution.deadman_timeout_s",
                                                      c.motion_execution_deadman_timeout_s);
    c.motion_execution_command_stale_timeout_s = get_double(
        kv,
        "motion_execution.command_stale_timeout_s",
        c.motion_execution_command_stale_timeout_s);
    c.motion_execution_require_localizer_initialized = get_bool(
        kv,
        "motion_execution.require_localizer_initialized",
        c.motion_execution_require_localizer_initialized);
    c.motion_execution_require_supervisor_not_lost = get_bool(
        kv,
        "motion_execution.require_supervisor_not_lost",
        c.motion_execution_require_supervisor_not_lost);
    c.motion_execution_require_tof_recent = get_bool(kv,
                                                     "motion_execution.require_tof_recent",
                                                     c.motion_execution_require_tof_recent);
    c.motion_execution_max_tof_age_s = get_double(kv,
                                                  "motion_execution.max_tof_age_s",
                                                  c.motion_execution_max_tof_age_s);
    c.motion_execution_obstacle_stop_enabled = get_bool(
        kv,
        "motion_execution.obstacle_stop_enabled",
        c.motion_execution_obstacle_stop_enabled);
    c.motion_execution_min_front_distance_m = get_double(
        kv,
        "motion_execution.min_front_distance_m",
        c.motion_execution_min_front_distance_m);
    c.motion_execution_min_side_distance_m = get_double(
        kv,
        "motion_execution.min_side_distance_m",
        c.motion_execution_min_side_distance_m);
    c.motion_execution_require_encoder_feedback_recent = get_bool(
        kv,
        "motion_execution.require_encoder_feedback_recent",
        c.motion_execution_require_encoder_feedback_recent);
    c.motion_execution_max_encoder_age_s = get_double(kv,
                                                      "motion_execution.max_encoder_age_s",
                                                      c.motion_execution_max_encoder_age_s);
    c.motion_execution_current_safety_enabled = get_bool(
        kv,
        "motion_execution.current_safety_enabled",
        c.motion_execution_current_safety_enabled);
    c.motion_execution_max_motor_current_a = get_double(
        kv,
        "motion_execution.max_motor_current_a",
        c.motion_execution_max_motor_current_a);
    c.motion_execution_stall_detection_enabled = get_bool(
        kv,
        "motion_execution.stall_detection_enabled",
        c.motion_execution_stall_detection_enabled);
    c.motion_execution_stall_speed_rpm = get_double(kv,
                                                    "motion_execution.stall_speed_rpm",
                                                    c.motion_execution_stall_speed_rpm);
    c.motion_execution_stall_current_a = get_double(kv,
                                                    "motion_execution.stall_current_a",
                                                    c.motion_execution_stall_current_a);
    c.motion_execution_stall_confirm_count = get_int(kv,
                                                     "motion_execution.stall_confirm_count",
                                                     c.motion_execution_stall_confirm_count);

    // writer dispatch gates
    c.motion_execution_dry_run_write_motor_commands = get_bool(
        kv,
        "motion_execution.dry_run_write_motor_commands",
        c.motion_execution_dry_run_write_motor_commands);
    c.motion_execution_hardware_write_enabled = get_bool(
        kv,
        "motion_execution.hardware_write_enabled",
        c.motion_execution_hardware_write_enabled);
    c.motion_execution_writeback_acknowledgement = get_string(
        kv,
        "motion_execution.writeback_acknowledgement",
        c.motion_execution_writeback_acknowledgement);
    c.motion_execution_required_writeback_acknowledgement = get_string(
        kv,
        "motion_execution.required_writeback_acknowledgement",
        c.motion_execution_required_writeback_acknowledgement);
    c.motion_execution_write_mode_acknowledgement = get_string(
        kv,
        "motion_execution.write_mode_acknowledgement",
        c.motion_execution_write_mode_acknowledgement);
    c.motion_execution_required_write_mode_acknowledgement = get_string(
        kv,
        "motion_execution.required_write_mode_acknowledgement",
        c.motion_execution_required_write_mode_acknowledgement);
    c.motion_execution_max_allowed_write_yaw_rate_dps = get_double(
        kv,
        "motion_execution.max_allowed_write_yaw_rate_dps",
        c.motion_execution_max_allowed_write_yaw_rate_dps);
    c.motion_execution_max_allowed_write_duration_s = get_double(
        kv,
        "motion_execution.max_allowed_write_duration_s",
        c.motion_execution_max_allowed_write_duration_s);
    c.motion_execution_allow_writer_dispatch = get_bool(
        kv,
        "motion_execution.allow_writer_dispatch",
        c.motion_execution_allow_writer_dispatch);
    c.motion_execution_writer_backend = get_string(kv,
                                                   "motion_execution.writer_backend",
                                                   c.motion_execution_writer_backend);

    // software motion
    c.motion_execution_software_motion_max_internal_rpm_for_normalization = get_double(
        kv,
        "motion_execution.software_motion.max_internal_rpm_for_normalization",
        c.motion_execution_software_motion_max_internal_rpm_for_normalization);
    c.motion_execution_software_motion_max_speed_normalized = get_double(
        kv,
        "motion_execution.software_motion.max_speed_normalized",
        c.motion_execution_software_motion_max_speed_normalized);
    c.motion_execution_software_motion_command_ttl_s = get_double(
        kv,
        "motion_execution.software_motion.command_ttl_s",
        c.motion_execution_software_motion_command_ttl_s);
    c.motion_execution_software_motion_allow_rotation_commands = get_bool(
        kv,
        "motion_execution.software_motion.allow_rotation_commands",
        c.motion_execution_software_motion_allow_rotation_commands);
    c.motion_execution_software_motion_allow_translation_commands = get_bool(
        kv,
        "motion_execution.software_motion.allow_translation_commands",
        c.motion_execution_software_motion_allow_translation_commands);
    c.motion_execution_software_motion_allow_emergency_stop = get_bool(
        kv,
        "motion_execution.software_motion.allow_emergency_stop",
        c.motion_execution_software_motion_allow_emergency_stop);
    c.motion_execution_software_motion_require_opposite_wheel_sign_for_rotation = get_bool(
        kv,
        "motion_execution.software_motion.require_opposite_wheel_sign_for_rotation",
        c.motion_execution_software_motion_require_opposite_wheel_sign_for_rotation);
    c.motion_execution_software_motion_production_interface_enabled = get_bool(
        kv,
        "motion_execution.software_motion.production_interface_enabled",
        c.motion_execution_software_motion_production_interface_enabled);
    c.motion_execution_software_motion_production_transport_backend = get_string(
        kv,
        "motion_execution.software_motion.production_transport_backend",
        c.motion_execution_software_motion_production_transport_backend);
    c.motion_execution_software_motion_loopback_shadow_mode = get_bool(
        kv,
        "motion_execution.software_motion.loopback_shadow_mode",
        c.motion_execution_software_motion_loopback_shadow_mode);

    // algorithm motion
    c.motion_execution_algorithm_motion_enabled = get_bool(
        kv,
        "motion_execution.algorithm_motion.enabled",
        c.motion_execution_algorithm_motion_enabled);
    c.motion_execution_algorithm_motion_allow_rotation_commands = get_bool(
        kv,
        "motion_execution.algorithm_motion.allow_rotation_commands",
        c.motion_execution_algorithm_motion_allow_rotation_commands);
    c.motion_execution_algorithm_motion_allow_translation_commands = get_bool(
        kv,
        "motion_execution.algorithm_motion.allow_translation_commands",
        c.motion_execution_algorithm_motion_allow_translation_commands);
    c.motion_execution_algorithm_motion_allow_recovery_commands = get_bool(
        kv,
        "motion_execution.algorithm_motion.allow_recovery_commands",
        c.motion_execution_algorithm_motion_allow_recovery_commands);
    c.motion_execution_algorithm_motion_allow_manual_test_commands = get_bool(
        kv,
        "motion_execution.algorithm_motion.allow_manual_test_commands",
        c.motion_execution_algorithm_motion_allow_manual_test_commands);
    c.motion_execution_algorithm_motion_default_ttl_s = get_double(
        kv,
        "motion_execution.algorithm_motion.default_ttl_s",
        c.motion_execution_algorithm_motion_default_ttl_s);
    c.motion_execution_algorithm_motion_direction_probe_speed = get_double(
        kv,
        "motion_execution.algorithm_motion.direction_probe_speed",
        c.motion_execution_algorithm_motion_direction_probe_speed);
    c.motion_execution_algorithm_motion_direction_probe_duration_s = get_double(
        kv,
        "motion_execution.algorithm_motion.direction_probe_duration_s",
        c.motion_execution_algorithm_motion_direction_probe_duration_s);
    c.motion_execution_algorithm_motion_active_scan_speed = get_double(
        kv,
        "motion_execution.algorithm_motion.active_scan_speed",
        c.motion_execution_algorithm_motion_active_scan_speed);
    c.motion_execution_algorithm_motion_active_scan_duration_s = get_double(
        kv,
        "motion_execution.algorithm_motion.active_scan_duration_s",
        c.motion_execution_algorithm_motion_active_scan_duration_s);
    c.motion_execution_algorithm_motion_recovery_speed = get_double(
        kv,
        "motion_execution.algorithm_motion.recovery_speed",
        c.motion_execution_algorithm_motion_recovery_speed);
    c.motion_execution_algorithm_motion_recovery_duration_s = get_double(
        kv,
        "motion_execution.algorithm_motion.recovery_duration_s",
        c.motion_execution_algorithm_motion_recovery_duration_s);
    c.motion_execution_algorithm_motion_manual_test_speed = get_double(
        kv,
        "motion_execution.algorithm_motion.manual_test_speed",
        c.motion_execution_algorithm_motion_manual_test_speed);
    c.motion_execution_algorithm_motion_manual_test_duration_s = get_double(
        kv,
        "motion_execution.algorithm_motion.manual_test_duration_s",
        c.motion_execution_algorithm_motion_manual_test_duration_s);

    // M2-B1 preflight
    c.motion_execution_m2b1_preflight_enabled = get_bool(
        kv,
        "motion_execution.m2b1_preflight.enabled",
        c.motion_execution_m2b1_preflight_enabled);
    c.motion_execution_m2b1_preflight_mode = get_string(
        kv,
        "motion_execution.m2b1_preflight.mode",
        c.motion_execution_m2b1_preflight_mode);
    c.motion_execution_m2b1_preflight_require_operator_present = get_bool(
        kv,
        "motion_execution.m2b1_preflight.require_operator_present",
        c.motion_execution_m2b1_preflight_require_operator_present);
    c.motion_execution_m2b1_preflight_require_robot_lifted_or_wheels_free = get_bool(
        kv,
        "motion_execution.m2b1_preflight.require_robot_lifted_or_wheels_free",
        c.motion_execution_m2b1_preflight_require_robot_lifted_or_wheels_free);
    c.motion_execution_m2b1_preflight_require_emergency_stop_available = get_bool(
        kv,
        "motion_execution.m2b1_preflight.require_emergency_stop_available",
        c.motion_execution_m2b1_preflight_require_emergency_stop_available);
    c.motion_execution_m2b1_preflight_max_live_speed_normalized = get_double(
        kv,
        "motion_execution.m2b1_preflight.max_live_speed_normalized",
        c.motion_execution_m2b1_preflight_max_live_speed_normalized);
    c.motion_execution_m2b1_preflight_max_live_duration_s = get_double(
        kv,
        "motion_execution.m2b1_preflight.max_live_duration_s",
        c.motion_execution_m2b1_preflight_max_live_duration_s);
    c.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized = get_double(
        kv,
        "motion_execution.m2b1_preflight.direction_probe_max_speed_normalized",
        c.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized);
    c.motion_execution_m2b1_preflight_direction_probe_max_duration_s = get_double(
        kv,
        "motion_execution.m2b1_preflight.direction_probe_max_duration_s",
        c.motion_execution_m2b1_preflight_direction_probe_max_duration_s);
    c.motion_execution_m2b1_preflight_require_shadow_transport_first = get_bool(
        kv,
        "motion_execution.m2b1_preflight.require_shadow_transport_first",
        c.motion_execution_m2b1_preflight_require_shadow_transport_first);
    c.motion_execution_m2b1_preflight_require_left_right_direction_confirmation = get_bool(
        kv,
        "motion_execution.m2b1_preflight.require_left_right_direction_confirmation",
        c.motion_execution_m2b1_preflight_require_left_right_direction_confirmation);

    // manual arm
    c.motion_execution_manual_arm_enable_live_motion = get_bool(
        kv,
        "motion_execution.manual_arm.enable_live_motion",
        c.motion_execution_manual_arm_enable_live_motion);
    c.motion_execution_manual_arm_confirmation_phrase = get_string(
        kv,
        "motion_execution.manual_arm.confirmation_phrase",
        c.motion_execution_manual_arm_confirmation_phrase);
    c.motion_execution_apply_log_enabled = get_bool(kv,
                                                    "motion_execution.apply_log_enabled",
                                                    c.motion_execution_apply_log_enabled);
}

inline void validate_motion_execution_config(const Config &c, std::vector<std::string> &errors) {
    auto positive = [&](const char *name, double v) {
        if (!(v > 0.0) || !std::isfinite(v)) errors.push_back(std::string(name) + " must be > 0");
    };
    auto non_negative = [&](const char *name, double v) {
        if (v < 0.0 || !std::isfinite(v)) errors.push_back(std::string(name) + " must be >= 0");
    };

    // fail-closed production gates
    if (!one_of(c.motion_execution_mode, {"disabled", "dry_run"})) {
        errors.push_back("motion_execution.mode must be disabled or dry_run; motor write unsupported in M1");
    }
    if (c.motion_execution_hardware_write_enabled) {
        errors.push_back("motion_execution.hardware_write_enabled=true unsupported in M1 dry-run");
    }
    if (c.motion_execution_dry_run_write_motor_commands) {
        errors.push_back("motion_execution.dry_run_write_motor_commands=true unsupported in M1 dry-run");
    }
    if (c.motion_execution_allow_writer_dispatch) {
        errors.push_back("motion_execution.allow_writer_dispatch=true unsupported in M2-A production config");
    }
    if (!one_of(c.motion_execution_writer_backend, {"null", "software_direction_speed_test_only"})) {
        errors.push_back("motion_execution.writer_backend must be null or software_direction_speed_test_only");
    }
    if (c.motion_execution_writer_backend != "null") {
        errors.push_back("software_direction_speed_writer_is_test_only_in_m2b0");
    }
    if (c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("motion_execution.software_motion.production_interface_enabled=true unsupported in M2-B0");
    }
    if (!one_of(c.motion_execution_software_motion_production_transport_backend,
                {"none", "loopback_shadow"})) {
        errors.push_back(
            "motion_execution.software_motion.production_transport_backend must be none or loopback_shadow");
    }
    if (c.motion_execution_software_motion_production_transport_backend == "loopback_shadow" &&
        !c.motion_execution_software_motion_loopback_shadow_mode) {
        errors.push_back("loopback_transport_non_shadow_unsupported_in_m2b1_pre");
    }

    // numeric ranges
    positive("motion_execution.log_hz", c.motion_execution_log_hz);
    positive("motion_execution.wheel_base_m", c.motion_execution_wheel_base_m);
    positive("motion_execution.wheel_radius_m", c.motion_execution_wheel_radius_m);
    non_negative("motion_execution.max_abs_yaw_rate_dps", c.motion_execution_max_abs_yaw_rate_dps);
    non_negative("motion_execution.max_wheel_speed_rpm", c.motion_execution_max_wheel_speed_rpm);
    non_negative("motion_execution.min_wheel_speed_rpm", c.motion_execution_min_wheel_speed_rpm);
    if (c.motion_execution_min_wheel_speed_rpm > c.motion_execution_max_wheel_speed_rpm) {
        errors.push_back("motion_execution.min_wheel_speed_rpm must be <= max_wheel_speed_rpm");
    }
    positive("motion_execution.max_command_duration_s", c.motion_execution_max_command_duration_s);
    positive("motion_execution.deadman_timeout_s", c.motion_execution_deadman_timeout_s);
    positive("motion_execution.command_stale_timeout_s", c.motion_execution_command_stale_timeout_s);
    non_negative("motion_execution.max_tof_age_s", c.motion_execution_max_tof_age_s);
    non_negative("motion_execution.min_front_distance_m", c.motion_execution_min_front_distance_m);
    non_negative("motion_execution.min_side_distance_m", c.motion_execution_min_side_distance_m);
    non_negative("motion_execution.max_encoder_age_s", c.motion_execution_max_encoder_age_s);
    non_negative("motion_execution.max_motor_current_a", c.motion_execution_max_motor_current_a);
    non_negative("motion_execution.stall_speed_rpm", c.motion_execution_stall_speed_rpm);
    non_negative("motion_execution.stall_current_a", c.motion_execution_stall_current_a);
    if (c.motion_execution_stall_confirm_count < 0) {
        errors.push_back("motion_execution.stall_confirm_count must be >= 0");
    }
    positive("motion_execution.max_allowed_write_yaw_rate_dps",
             c.motion_execution_max_allowed_write_yaw_rate_dps);
    positive("motion_execution.max_allowed_write_duration_s",
             c.motion_execution_max_allowed_write_duration_s);

    // software motion constraints
    positive("motion_execution.software_motion.max_internal_rpm_for_normalization",
             c.motion_execution_software_motion_max_internal_rpm_for_normalization);
    if (!std::isfinite(c.motion_execution_software_motion_max_speed_normalized) ||
        c.motion_execution_software_motion_max_speed_normalized < 0.0 ||
        c.motion_execution_software_motion_max_speed_normalized > 1.0) {
        errors.push_back("motion_execution.software_motion.max_speed_normalized must be in [0,1]");
    }
    positive("motion_execution.software_motion.command_ttl_s",
             c.motion_execution_software_motion_command_ttl_s);

    // algorithm motion constraints
    if (!std::isfinite(c.motion_execution_algorithm_motion_default_ttl_s) ||
        c.motion_execution_algorithm_motion_default_ttl_s <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.default_ttl_s must be > 0");
    } else if (c.motion_execution_algorithm_motion_default_ttl_s > 0.5) {
        errors.push_back("motion_execution.algorithm_motion.default_ttl_s must be <= 0.5");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_direction_probe_speed) ||
        c.motion_execution_algorithm_motion_direction_probe_speed <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.direction_probe_speed must be > 0");
    } else if (c.motion_execution_algorithm_motion_direction_probe_speed > 0.03) {
        errors.push_back("motion_execution.algorithm_motion.direction_probe_speed must be <= 0.03");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_direction_probe_duration_s) ||
        c.motion_execution_algorithm_motion_direction_probe_duration_s <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.direction_probe_duration_s must be > 0");
    } else if (c.motion_execution_algorithm_motion_direction_probe_duration_s > 0.30) {
        errors.push_back("motion_execution.algorithm_motion.direction_probe_duration_s must be <= 0.30");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_active_scan_speed) ||
        c.motion_execution_algorithm_motion_active_scan_speed <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.active_scan_speed must be > 0");
    } else if (c.motion_execution_algorithm_motion_active_scan_speed > 0.05) {
        errors.push_back("motion_execution.algorithm_motion.active_scan_speed must be <= 0.05");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_active_scan_duration_s) ||
        c.motion_execution_algorithm_motion_active_scan_duration_s <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.active_scan_duration_s must be > 0");
    } else if (c.motion_execution_algorithm_motion_active_scan_duration_s > 0.50) {
        errors.push_back("motion_execution.algorithm_motion.active_scan_duration_s must be <= 0.50");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_recovery_speed) ||
        c.motion_execution_algorithm_motion_recovery_speed <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.recovery_speed must be > 0");
    } else if (c.motion_execution_algorithm_motion_recovery_speed > 0.05) {
        errors.push_back("motion_execution.algorithm_motion.recovery_speed must be <= 0.05");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_recovery_duration_s) ||
        c.motion_execution_algorithm_motion_recovery_duration_s <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.recovery_duration_s must be > 0");
    } else if (c.motion_execution_algorithm_motion_recovery_duration_s > 0.50) {
        errors.push_back("motion_execution.algorithm_motion.recovery_duration_s must be <= 0.50");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_manual_test_speed) ||
        c.motion_execution_algorithm_motion_manual_test_speed <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.manual_test_speed must be > 0");
    } else if (c.motion_execution_algorithm_motion_manual_test_speed > 0.03) {
        errors.push_back("motion_execution.algorithm_motion.manual_test_speed must be <= 0.03");
    }
    if (!std::isfinite(c.motion_execution_algorithm_motion_manual_test_duration_s) ||
        c.motion_execution_algorithm_motion_manual_test_duration_s <= 0.0) {
        errors.push_back("motion_execution.algorithm_motion.manual_test_duration_s must be > 0");
    } else if (c.motion_execution_algorithm_motion_manual_test_duration_s > 0.30) {
        errors.push_back("motion_execution.algorithm_motion.manual_test_duration_s must be <= 0.30");
    }

    // M2-B1 preflight constraints
    if (!one_of(c.motion_execution_m2b1_preflight_mode,
                {"lifted_direction_probe", "confirmed_lifted_live"})) {
        errors.push_back(
            "motion_execution.m2b1_preflight.mode must be lifted_direction_probe or confirmed_lifted_live");
    }
    if (!std::isfinite(c.motion_execution_m2b1_preflight_max_live_speed_normalized) ||
        c.motion_execution_m2b1_preflight_max_live_speed_normalized <= 0.0) {
        errors.push_back("motion_execution.m2b1_preflight.max_live_speed_normalized must be > 0");
    } else if (c.motion_execution_m2b1_preflight_max_live_speed_normalized > 0.05) {
        errors.push_back("motion_execution.m2b1_preflight.max_live_speed_normalized must be <= 0.05");
    }
    if (!std::isfinite(c.motion_execution_m2b1_preflight_max_live_duration_s) ||
        c.motion_execution_m2b1_preflight_max_live_duration_s <= 0.0) {
        errors.push_back("motion_execution.m2b1_preflight.max_live_duration_s must be > 0");
    } else if (c.motion_execution_m2b1_preflight_max_live_duration_s > 0.5) {
        errors.push_back("motion_execution.m2b1_preflight.max_live_duration_s must be <= 0.5");
    }
    if (!std::isfinite(c.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized) ||
        c.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized <= 0.0) {
        errors.push_back(
            "motion_execution.m2b1_preflight.direction_probe_max_speed_normalized must be > 0");
    } else if (c.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized > 0.03) {
        errors.push_back(
            "motion_execution.m2b1_preflight.direction_probe_max_speed_normalized must be <= 0.03");
    }
    if (!std::isfinite(c.motion_execution_m2b1_preflight_direction_probe_max_duration_s) ||
        c.motion_execution_m2b1_preflight_direction_probe_max_duration_s <= 0.0) {
        errors.push_back("motion_execution.m2b1_preflight.direction_probe_max_duration_s must be > 0");
    } else if (c.motion_execution_m2b1_preflight_direction_probe_max_duration_s > 0.30) {
        errors.push_back("motion_execution.m2b1_preflight.direction_probe_max_duration_s must be <= 0.30");
    }
}
