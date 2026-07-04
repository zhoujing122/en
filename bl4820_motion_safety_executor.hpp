#pragma once
#include "active_scan_command_planner.hpp"
#include "active_scan_manager.hpp"
#include "common.hpp"
#include "mapping_supervisor.hpp"
#include "startup_lost_recovery_manager.hpp"

namespace robot_slamd {

enum class MotionSafetyExecutorState {
    DISABLED,
    IDLE,
    PRECHECK,
    WOULD_COMMAND,
    WOULD_ZERO,
    BLOCKED,
    FAULT,
    COOLDOWN,
};

inline const char *motion_safety_executor_state_name(MotionSafetyExecutorState s) {
    switch (s) {
    case MotionSafetyExecutorState::DISABLED: return "DISABLED";
    case MotionSafetyExecutorState::IDLE: return "IDLE";
    case MotionSafetyExecutorState::PRECHECK: return "PRECHECK";
    case MotionSafetyExecutorState::WOULD_COMMAND: return "WOULD_COMMAND";
    case MotionSafetyExecutorState::WOULD_ZERO: return "WOULD_ZERO";
    case MotionSafetyExecutorState::BLOCKED: return "BLOCKED";
    case MotionSafetyExecutorState::FAULT: return "FAULT";
    case MotionSafetyExecutorState::COOLDOWN: return "COOLDOWN";
    }
    return "IDLE";
}

struct MotionSafetyExecutorInput {
    double timestamp_s = 0.0;
    ActiveScanCommandSnapshot active_scan_command;
    ActiveScanSnapshot active_scan;
    MappingSupervisorSnapshot supervisor;
    RecoveryManagerSnapshot recovery;
    bool localizer_initialized = false;
    Pose current_pose;
    double linear_speed_mps = 0.0;
    double yaw_rate_radps = 0.0;
    bool tof_recent = false;
    double latest_tof_age_s = 0.0;
    double front_distance_m = std::numeric_limits<double>::infinity();
    double left_distance_m = std::numeric_limits<double>::infinity();
    double right_distance_m = std::numeric_limits<double>::infinity();
    bool encoder_feedback_recent = false;
    double latest_encoder_age_s = 0.0;
    double left_speed_rpm = 0.0;
    double right_speed_rpm = 0.0;
    double left_current_a = 0.0;
    double right_current_a = 0.0;
    int left_status = 0;
    int right_status = 0;
};

struct MotionSafetyExecutorSnapshot {
    double timestamp_s = 0.0;
    std::string state = "IDLE";
    std::string previous_state = "IDLE";
    std::string reason = "idle";
    bool command_seen = false;
    bool would_command = false;
    bool would_zero = false;
    bool blocked = false;
    bool fault = false;
    double desired_linear_speed_mps = 0.0;
    double desired_yaw_rate_radps = 0.0;
    double desired_yaw_rate_dps = 0.0;
    double target_left_wheel_mps = 0.0;
    double target_right_wheel_mps = 0.0;
    double target_left_rpm = 0.0;
    double target_right_rpm = 0.0;
    bool localizer_ok = false;
    bool supervisor_ok = false;
    bool tof_ok = false;
    bool obstacle_ok = false;
    bool encoder_ok = false;
    bool current_ok = false;
    bool status_ok = false;
    bool command_fresh = false;
    bool deadman_ok = false;
    double front_distance_m = 0.0;
    double left_distance_m = 0.0;
    double right_distance_m = 0.0;
    double left_current_a = 0.0;
    double right_current_a = 0.0;
    double left_speed_rpm = 0.0;
    double right_speed_rpm = 0.0;
    int left_status = 0;
    int right_status = 0;
    std::string command_source = "active_scan_command";
    std::string active_scan_command_state = "IDLE";
    std::string active_scan_command_reason = "idle";
    std::string supervisor_state = "INIT";
    std::string recovery_state = "IDLE";
    uint64_t command_session_id = 0;
    double command_session_duration_s = 0.0;
    bool command_duration_ok = true;
    double command_age_s = -1.0;
    double deadman_age_s = -1.0;
    bool write_authorization_present = false;
    bool write_authorization_valid = false;
    bool feedback_finite_ok = true;
    bool wheel_direction_ok = true;
    bool command_duration_latched = false;
    uint64_t command_duration_latch_session_id = 0;
};

class BL4820MotionSafetyExecutor {
public:
    explicit BL4820MotionSafetyExecutor(const Config &cfg) : cfg_(cfg) {
        if (!enabled()) {
            state_ = MotionSafetyExecutorState::DISABLED;
            latest_.state = "DISABLED";
            latest_.previous_state = "DISABLED";
            latest_.reason = "disabled";
        }
    }

    bool update(const MotionSafetyExecutorInput &in) {
        last_update_s_ = in.timestamp_s;

        MotionSafetyExecutorSnapshot s = base_snapshot(in);
        if (!enabled()) return set_snapshot(s, MotionSafetyExecutorState::DISABLED, "disabled");
        if (cfg_.motion_execution_mode != "dry_run") return set_snapshot(s, MotionSafetyExecutorState::FAULT, "fault");

        const bool active_present = active_command_present(in.active_scan_command);
        const bool seen = command_seen(in);
        s.command_seen = seen;
        s.command_fresh = command_fresh(in);
        s.deadman_ok = deadman_ok(in);
        s.deadman_age_s = deadman_age(in);
        s.command_duration_latched = command_duration_exceeded_latched_;
        s.command_duration_latch_session_id = duration_latch_session_id_;
        compute_wheel_targets(s, in.active_scan_command.desired_yaw_rate_radps);
        const bool wheel_limited = limit_wheel_targets(s);
        s.wheel_direction_ok = wheel_direction_ok(s);

        if (!seen) {
            clear_duration_latch();
            s.command_duration_latched = false;
            s.command_duration_latch_session_id = 0;
            if (zero_after_command_required_ || last_would_command_ || command_session_active_) {
                return zero(s, "command_lost_zero");
            }
            zero_after_command_emitted_ = false;
            last_would_command_ = false;
            return set_snapshot(s, MotionSafetyExecutorState::IDLE, "idle");
        }

        if (command_duration_exceeded_latched_) {
            if (in.active_scan_command.zero_command) {
                clear_duration_latch();
                s.command_duration_latched = false;
                s.command_duration_latch_session_id = 0;
                return zero(s, "zero_from_planner");
            }
            if (command_stop_state(in.active_scan_command.state)) {
                clear_duration_latch();
                s.command_duration_latched = false;
                s.command_duration_latch_session_id = 0;
                return zero(s, "command_completed_zero");
            }
            if (active_present) return zero(s, "command_duration_latched");
            clear_duration_latch();
            s.command_duration_latched = false;
            s.command_duration_latch_session_id = 0;
        }

        if (zero_after_command_required_) {
            return zero(s, "command_lost_zero");
        }
        if (in.active_scan_command.zero_command) return zero(s, "zero_from_planner");
        if (command_stop_state(in.active_scan_command.state)) return zero(s, "command_completed_zero");
        if (!s.command_fresh) return zero(s, "command_stale");
        if (!s.deadman_ok) return zero(s, "deadman_timeout");

        std::string reason;
        bool zero_required = false;
        if (!s.localizer_ok) reason = "localizer_not_initialized";
        else if (!s.supervisor_ok) { reason = "supervisor_lost"; zero_required = true; }
        else if (!s.tof_ok) reason = "tof_stale";
        else if (!s.obstacle_ok) { reason = obstacle_reason(in); zero_required = true; }
        else if (!s.encoder_ok) reason = "encoder_stale";
        else if (!s.feedback_finite_ok) reason = "feedback_not_finite";
        else if (!s.status_ok) { reason = "motor_status_error"; zero_required = true; }
        else if (!s.current_ok) { reason = "motor_current_high"; zero_required = true; }
        else if (stall_detected(in)) { reason = "stall_detected"; zero_required = true; }
        else if (!allowed_command_state(in.active_scan_command.state)) reason = "command_state_not_allowed";
        else if (cfg_.motion_execution_require_active_scan_command && !active_present) reason = "command_not_active";
        else if (!std::isfinite(in.active_scan_command.desired_yaw_rate_radps) || !std::isfinite(in.active_scan_command.desired_yaw_rate_dps)) reason = "fault";
        else if (std::fabs(in.active_scan_command.desired_yaw_rate_dps) > cfg_.motion_execution_max_abs_yaw_rate_dps) reason = "yaw_rate_too_high";
        else if (!std::isfinite(in.active_scan_command.desired_linear_speed_mps) || std::fabs(in.active_scan_command.desired_linear_speed_mps) > cfg_.motion_execution_max_linear_speed_mps) reason = "linear_speed_not_zero";
        else if (!s.wheel_direction_ok) reason = "wheel_direction_invalid";

        if (!reason.empty()) {
            if (last_would_command_ || command_session_active_) return zero(s, reason == "fault" ? "fault_zero" : (zero_required ? reason : "command_blocked_zero"));
            if (zero_required) return zero(s, reason);
            return blocked(s, reason);
        }

        begin_or_continue_session(s, in.timestamp_s);
        if (!s.command_duration_ok) return zero(s, "command_duration_exceeded");

        if (active_present && s.command_fresh) last_command_seen_s_ = in.timestamp_s;
        s.deadman_age_s = deadman_age(in);
        s.would_command = true;
        last_would_command_ = true;
        zero_after_command_required_ = false;
        zero_after_command_emitted_ = false;
        return set_snapshot(s, MotionSafetyExecutorState::WOULD_COMMAND, wheel_limited ? "wheel_speed_limited" : "would_command");
    }

    bool should_log(double now_s) const {
        if (!enabled()) return false;
        if (state_changed_since_log_) return true;
        if (last_log_s_ < 0.0) return true;
        return now_s - last_log_s_ >= (1.0 / cfg_.motion_execution_log_hz);
    }

    void mark_logged(double now_s, const MotionSafetyExecutorSnapshot &) {
        last_log_s_ = now_s;
        state_changed_since_log_ = false;
    }

    MotionSafetyExecutorSnapshot snapshot() const { return latest_; }

    MotionSafetyExecutorRunStats run_stats(double) const {
        MotionSafetyExecutorRunStats out = stats_;
        out.state_last = motion_safety_executor_state_name(state_);
        return out;
    }

private:
    bool enabled() const { return cfg_.motion_execution_enabled && cfg_.motion_execution_mode != "disabled"; }

    static bool active_command_present(const ActiveScanCommandSnapshot &c) {
        return c.command_active || c.would_emit_command;
    }

    bool allowed_command_state(const std::string &state) const {
        if (cfg_.motion_execution_require_command_planner_verifying_or_commanding) {
            return state == "COMMANDING_ROTATION" || state == "VERIFYING_ROTATION";
        }
        return state == "PRECHECK" || state == "COMMANDING_ROTATION" || state == "VERIFYING_ROTATION";
    }

    static bool command_stop_state(const std::string &state) {
        return state == "COMPLETED" || state == "ABORTED" || state == "BLOCKED" || state == "FAULT";
    }

    bool command_seen(const MotionSafetyExecutorInput &in) const {
        const auto &c = in.active_scan_command;
        if (c.zero_command || c.command_active || c.would_emit_command) return true;
        if (command_stop_state(c.state)) return true;
        return cfg_.motion_execution_allow_recovery_scan_request_as_reason && in.recovery.recovery_scan_recommended;
    }

    double command_age(const MotionSafetyExecutorInput &in) const {
        if (!std::isfinite(in.active_scan_command.timestamp_s) || !std::isfinite(in.timestamp_s)) return -1.0;
        return in.timestamp_s - in.active_scan_command.timestamp_s;
    }

    bool command_fresh(const MotionSafetyExecutorInput &in) const {
        const double age = command_age(in);
        return age >= 0.0 && age <= cfg_.motion_execution_command_stale_timeout_s;
    }

    double deadman_age(const MotionSafetyExecutorInput &in) const {
        if (!command_session_active_ || last_command_seen_s_ < 0.0 || !std::isfinite(in.timestamp_s)) return -1.0;
        return in.timestamp_s - last_command_seen_s_;
    }

    bool deadman_ok(const MotionSafetyExecutorInput &in) const {
        if (!command_session_active_) return true;
        const double age = deadman_age(in);
        return age >= 0.0 && age <= cfg_.motion_execution_deadman_timeout_s;
    }

    MotionSafetyExecutorSnapshot base_snapshot(const MotionSafetyExecutorInput &in) const {
        MotionSafetyExecutorSnapshot s;
        s.timestamp_s = in.timestamp_s;
        s.state = motion_safety_executor_state_name(state_);
        s.previous_state = motion_safety_executor_state_name(previous_state_);
        s.desired_linear_speed_mps = in.active_scan_command.desired_linear_speed_mps;
        s.desired_yaw_rate_radps = in.active_scan_command.desired_yaw_rate_radps;
        s.desired_yaw_rate_dps = in.active_scan_command.desired_yaw_rate_dps;
        s.localizer_ok = !cfg_.motion_execution_require_localizer_initialized || in.localizer_initialized;
        s.supervisor_ok = !cfg_.motion_execution_require_supervisor_not_lost || in.supervisor.state != "LOST";
        s.tof_ok = !cfg_.motion_execution_require_tof_recent || (in.tof_recent && std::isfinite(in.latest_tof_age_s) && in.latest_tof_age_s <= cfg_.motion_execution_max_tof_age_s);
        s.obstacle_ok = obstacle_ok(in);
        s.encoder_ok = !cfg_.motion_execution_require_encoder_feedback_recent || (in.encoder_feedback_recent && std::isfinite(in.latest_encoder_age_s) && in.latest_encoder_age_s <= cfg_.motion_execution_max_encoder_age_s);
        s.feedback_finite_ok = feedback_finite_ok(in);
        s.current_ok = !cfg_.motion_execution_current_safety_enabled || (s.feedback_finite_ok && std::fabs(in.left_current_a) <= cfg_.motion_execution_max_motor_current_a && std::fabs(in.right_current_a) <= cfg_.motion_execution_max_motor_current_a);
        s.status_ok = in.left_status == 0 && in.right_status == 0;
        s.command_age_s = command_age(in);
        s.deadman_age_s = deadman_age(in);
        s.write_authorization_present = !cfg_.motion_execution_write_mode_acknowledgement.empty();
        s.write_authorization_valid = cfg_.motion_execution_write_mode_acknowledgement == cfg_.motion_execution_required_write_mode_acknowledgement;
        s.command_duration_latched = command_duration_exceeded_latched_;
        s.command_duration_latch_session_id = duration_latch_session_id_;
        s.command_session_id = command_session_id_;
        s.command_session_duration_s = command_session_active_ && command_session_start_s_ >= 0.0 ? std::max(0.0, in.timestamp_s - command_session_start_s_) : 0.0;
        s.command_duration_ok = true;
        s.front_distance_m = in.front_distance_m;
        s.left_distance_m = in.left_distance_m;
        s.right_distance_m = in.right_distance_m;
        s.left_current_a = in.left_current_a;
        s.right_current_a = in.right_current_a;
        s.left_speed_rpm = in.left_speed_rpm;
        s.right_speed_rpm = in.right_speed_rpm;
        s.left_status = in.left_status;
        s.right_status = in.right_status;
        s.command_source = cfg_.motion_execution_command_source;
        s.active_scan_command_state = in.active_scan_command.state;
        s.active_scan_command_reason = in.active_scan_command.reason;
        s.supervisor_state = in.supervisor.state;
        s.recovery_state = in.recovery.state;
        return s;
    }

    bool obstacle_ok(const MotionSafetyExecutorInput &in) const {
        if (!cfg_.motion_execution_obstacle_stop_enabled) return true;
        if (std::isfinite(in.front_distance_m) && in.front_distance_m < cfg_.motion_execution_min_front_distance_m) return false;
        if (std::isfinite(in.left_distance_m) && in.left_distance_m < cfg_.motion_execution_min_side_distance_m) return false;
        if (std::isfinite(in.right_distance_m) && in.right_distance_m < cfg_.motion_execution_min_side_distance_m) return false;
        return true;
    }

    std::string obstacle_reason(const MotionSafetyExecutorInput &in) const {
        if (std::isfinite(in.front_distance_m) && in.front_distance_m < cfg_.motion_execution_min_front_distance_m) return "front_obstacle_too_close";
        if (std::isfinite(in.left_distance_m) && in.left_distance_m < cfg_.motion_execution_min_side_distance_m) return "left_obstacle_too_close";
        if (std::isfinite(in.right_distance_m) && in.right_distance_m < cfg_.motion_execution_min_side_distance_m) return "right_obstacle_too_close";
        return "blocked";
    }

    static bool feedback_finite_ok(const MotionSafetyExecutorInput &in) {
        return std::isfinite(in.left_speed_rpm) && std::isfinite(in.right_speed_rpm) &&
               std::isfinite(in.left_current_a) && std::isfinite(in.right_current_a);
    }

    bool stall_detected(const MotionSafetyExecutorInput &in) {
        if (!cfg_.motion_execution_stall_detection_enabled) {
            stall_count_ = 0;
            return false;
        }
        const bool slow = std::fabs(in.left_speed_rpm) <= cfg_.motion_execution_stall_speed_rpm ||
                          std::fabs(in.right_speed_rpm) <= cfg_.motion_execution_stall_speed_rpm;
        const bool loaded = std::fabs(in.left_current_a) >= cfg_.motion_execution_stall_current_a ||
                            std::fabs(in.right_current_a) >= cfg_.motion_execution_stall_current_a;
        if (slow && loaded) stall_count_++;
        else stall_count_ = 0;
        return cfg_.motion_execution_stall_confirm_count > 0 && stall_count_ >= cfg_.motion_execution_stall_confirm_count;
    }

    void compute_wheel_targets(MotionSafetyExecutorSnapshot &s, double yaw_rate_radps) const {
        if (!std::isfinite(yaw_rate_radps)) {
            s.target_left_wheel_mps = 0.0;
            s.target_right_wheel_mps = 0.0;
            s.target_left_rpm = 0.0;
            s.target_right_rpm = 0.0;
            return;
        }
        s.target_left_wheel_mps = -yaw_rate_radps * cfg_.motion_execution_wheel_base_m * 0.5;
        s.target_right_wheel_mps = yaw_rate_radps * cfg_.motion_execution_wheel_base_m * 0.5;
        const double circumference = 2.0 * kPi * cfg_.motion_execution_wheel_radius_m;
        s.target_left_rpm = (s.target_left_wheel_mps / circumference) * 60.0;
        s.target_right_rpm = (s.target_right_wheel_mps / circumference) * 60.0;
    }

    bool limit_wheel_targets(MotionSafetyExecutorSnapshot &s) const {
        bool limited = false;
        auto limit = [&](double &rpm, double &mps) {
            if (std::fabs(rpm) > cfg_.motion_execution_max_wheel_speed_rpm) {
                rpm = std::copysign(cfg_.motion_execution_max_wheel_speed_rpm, rpm);
                mps = (rpm / 60.0) * (2.0 * kPi * cfg_.motion_execution_wheel_radius_m);
                limited = true;
            }
            if (std::fabs(rpm) > 0.0 && std::fabs(rpm) < cfg_.motion_execution_min_wheel_speed_rpm) {
                rpm = 0.0;
                mps = 0.0;
                limited = true;
            }
        };
        limit(s.target_left_rpm, s.target_left_wheel_mps);
        limit(s.target_right_rpm, s.target_right_wheel_mps);
        return limited;
    }

    static bool wheel_direction_ok(const MotionSafetyExecutorSnapshot &s) {
        if (std::fabs(s.desired_yaw_rate_radps) < 1e-12) return true;
        if (std::fabs(s.target_left_rpm) < 1e-12 || std::fabs(s.target_right_rpm) < 1e-12) return true;
        if (s.desired_yaw_rate_radps > 0.0) return s.target_left_rpm < 0.0 && s.target_right_rpm > 0.0;
        return s.target_left_rpm > 0.0 && s.target_right_rpm < 0.0;
    }

    void begin_or_continue_session(MotionSafetyExecutorSnapshot &s, double now_s) {
        if (!command_session_active_) {
            command_session_active_ = true;
            command_session_start_s_ = now_s;
            command_session_id_++;
        }
        s.command_session_id = command_session_id_;
        s.command_session_duration_s = std::max(0.0, now_s - command_session_start_s_);
        s.command_duration_ok = s.command_session_duration_s <= cfg_.motion_execution_max_command_duration_s;
    }

    void clear_duration_latch() {
        command_duration_exceeded_latched_ = false;
        duration_latch_command_timestamp_s_ = -1.0;
        duration_latch_session_id_ = 0;
    }

    bool set_snapshot(MotionSafetyExecutorSnapshot &s, MotionSafetyExecutorState next, const std::string &reason) {
        previous_state_ = state_;
        const bool changed = next != state_;
        state_ = next;
        s.state = motion_safety_executor_state_name(state_);
        s.previous_state = motion_safety_executor_state_name(previous_state_);
        s.reason = reason;
        s.blocked = state_ == MotionSafetyExecutorState::BLOCKED;
        s.fault = state_ == MotionSafetyExecutorState::FAULT;
        latest_ = s;
        if (changed) {
            stats_.state_changes++;
            state_changed_since_log_ = true;
        }
        update_stats(s);
        return changed;
    }

    bool blocked(MotionSafetyExecutorSnapshot &s, const std::string &reason) {
        s.blocked = true;
        if (last_would_command_ || command_session_active_) zero_after_command_required_ = true;
        last_would_command_ = false;
        return set_snapshot(s, MotionSafetyExecutorState::BLOCKED, reason.empty() ? "blocked" : reason);
    }

    bool zero(MotionSafetyExecutorSnapshot &s, const std::string &reason) {
        s.would_zero = true;
        s.target_left_wheel_mps = 0.0;
        s.target_right_wheel_mps = 0.0;
        s.target_left_rpm = 0.0;
        s.target_right_rpm = 0.0;
        s.command_session_id = command_session_id_;
        s.command_session_duration_s = command_session_active_ && command_session_start_s_ >= 0.0 ? std::max(0.0, s.timestamp_s - command_session_start_s_) : s.command_session_duration_s;
        if (reason == "command_duration_exceeded") {
            s.command_duration_ok = false;
            command_duration_exceeded_latched_ = true;
            duration_latch_command_timestamp_s_ = s.command_age_s >= 0.0 ? s.timestamp_s - s.command_age_s : -1.0;
            duration_latch_session_id_ = command_session_id_;
            s.command_duration_latched = true;
            s.command_duration_latch_session_id = duration_latch_session_id_;
        } else {
            s.command_duration_latched = command_duration_exceeded_latched_;
            s.command_duration_latch_session_id = duration_latch_session_id_;
        }
        command_session_active_ = false;
        command_session_start_s_ = -1.0;
        last_would_command_ = false;
        zero_after_command_required_ = false;
        zero_after_command_emitted_ = true;
        return set_snapshot(s, MotionSafetyExecutorState::WOULD_ZERO, reason.empty() ? "would_zero" : reason);
    }

    void update_stats(const MotionSafetyExecutorSnapshot &s) {
        stats_.state_last = s.state;
        stats_.last_reason = s.reason;
        stats_.last_target_left_rpm = s.target_left_rpm;
        stats_.last_target_right_rpm = s.target_right_rpm;
        stats_.last_desired_yaw_rate_dps = s.desired_yaw_rate_dps;
        stats_.last_front_distance_m = s.front_distance_m;
        stats_.last_left_distance_m = s.left_distance_m;
        stats_.last_right_distance_m = s.right_distance_m;
        stats_.last_command_session_id = s.command_session_id;
        stats_.last_command_session_duration_s = s.command_session_duration_s;
        stats_.last_command_age_s = s.command_age_s;
        stats_.last_deadman_age_s = s.deadman_age_s;
        stats_.write_authorization_valid_last = s.write_authorization_valid;
        if (s.command_seen) stats_.command_seen_count++;
        if (s.would_command) stats_.would_command_count++;
        if (s.would_zero) stats_.would_zero_count++;
        if (s.blocked) stats_.blocked_count++;
        if (s.fault) stats_.fault_count++;
        if (s.reason == "front_obstacle_too_close" || s.reason == "left_obstacle_too_close" || s.reason == "right_obstacle_too_close") stats_.obstacle_stop_count++;
        if (s.reason == "tof_stale") stats_.tof_stale_block_count++;
        if (s.reason == "encoder_stale") stats_.encoder_stale_block_count++;
        if (s.reason == "motor_current_high") stats_.current_high_block_count++;
        if (s.reason == "motor_status_error") stats_.status_error_block_count++;
        if (s.reason == "stall_detected") stats_.stall_block_count++;
        if (s.reason == "supervisor_lost") stats_.supervisor_lost_block_count++;
        if (s.reason == "command_duration_exceeded") stats_.command_duration_exceeded_count++;
        if (s.reason == "command_duration_latched") stats_.command_duration_latched_count++;
        stats_.command_duration_latch_active_last = s.command_duration_latched;
        if (s.reason == "command_stale") stats_.command_stale_count++;
        if (s.reason == "deadman_timeout" || s.reason == "deadman_zero") stats_.deadman_timeout_count++;
        if (s.reason == "feedback_not_finite") stats_.feedback_not_finite_block_count++;
        if (s.reason == "wheel_direction_invalid") stats_.wheel_direction_invalid_count++;
    }

    const Config &cfg_;
    MotionSafetyExecutorState state_ = MotionSafetyExecutorState::IDLE;
    MotionSafetyExecutorState previous_state_ = MotionSafetyExecutorState::IDLE;
    MotionSafetyExecutorSnapshot latest_;
    MotionSafetyExecutorRunStats stats_;
    double last_update_s_ = -1.0;
    double last_log_s_ = -1.0;
    double last_command_seen_s_ = -1.0;
    double command_session_start_s_ = -1.0;
    uint64_t command_session_id_ = 0;
    bool command_session_active_ = false;
    bool last_would_command_ = false;
    bool zero_after_command_required_ = false;
    bool zero_after_command_emitted_ = false;
    bool state_changed_since_log_ = false;
    bool command_duration_exceeded_latched_ = false;
    double duration_latch_command_timestamp_s_ = -1.0;
    uint64_t duration_latch_session_id_ = 0;
    int stall_count_ = 0;
};

inline void write_motion_safety_executor_header(std::ostream &o) {
    o << "timestamp_s,state,previous_state,reason,command_seen,would_command,would_zero,blocked,fault,desired_linear_speed_mps,desired_yaw_rate_radps,desired_yaw_rate_dps,target_left_wheel_mps,target_right_wheel_mps,target_left_rpm,target_right_rpm,localizer_ok,supervisor_ok,tof_ok,obstacle_ok,encoder_ok,current_ok,status_ok,command_fresh,deadman_ok,front_distance_m,left_distance_m,right_distance_m,left_current_a,right_current_a,left_speed_rpm,right_speed_rpm,left_status,right_status,command_source,active_scan_command_state,active_scan_command_reason,supervisor_state,recovery_state,command_session_id,command_session_duration_s,command_duration_ok,command_age_s,deadman_age_s,write_authorization_present,write_authorization_valid,feedback_finite_ok,wheel_direction_ok,command_duration_latched,command_duration_latch_session_id\n";
}

inline void write_motion_safety_executor_row(std::ostream &o, const MotionSafetyExecutorSnapshot &s) {
    o << std::fixed << std::setprecision(6)
      << s.timestamp_s << ',' << s.state << ',' << s.previous_state << ',' << s.reason << ','
      << (s.command_seen ? 1 : 0) << ',' << (s.would_command ? 1 : 0) << ',' << (s.would_zero ? 1 : 0) << ','
      << (s.blocked ? 1 : 0) << ',' << (s.fault ? 1 : 0) << ',' << s.desired_linear_speed_mps << ','
      << s.desired_yaw_rate_radps << ',' << s.desired_yaw_rate_dps << ',' << s.target_left_wheel_mps << ','
      << s.target_right_wheel_mps << ',' << s.target_left_rpm << ',' << s.target_right_rpm << ','
      << (s.localizer_ok ? 1 : 0) << ',' << (s.supervisor_ok ? 1 : 0) << ',' << (s.tof_ok ? 1 : 0) << ','
      << (s.obstacle_ok ? 1 : 0) << ',' << (s.encoder_ok ? 1 : 0) << ',' << (s.current_ok ? 1 : 0) << ','
      << (s.status_ok ? 1 : 0) << ',' << (s.command_fresh ? 1 : 0) << ',' << (s.deadman_ok ? 1 : 0) << ','
      << s.front_distance_m << ',' << s.left_distance_m << ',' << s.right_distance_m << ',' << s.left_current_a << ','
      << s.right_current_a << ',' << s.left_speed_rpm << ',' << s.right_speed_rpm << ',' << s.left_status << ','
      << s.right_status << ',' << s.command_source << ',' << s.active_scan_command_state << ','
      << s.active_scan_command_reason << ',' << s.supervisor_state << ',' << s.recovery_state << ','
      << s.command_session_id << ',' << s.command_session_duration_s << ',' << (s.command_duration_ok ? 1 : 0) << ','
      << s.command_age_s << ',' << s.deadman_age_s << ',' << (s.write_authorization_present ? 1 : 0) << ','
      << (s.write_authorization_valid ? 1 : 0) << ',' << (s.feedback_finite_ok ? 1 : 0) << ','
      << (s.wheel_direction_ok ? 1 : 0) << ',' << (s.command_duration_latched ? 1 : 0) << ','
      << s.command_duration_latch_session_id << "\n";
}

} // namespace robot_slamd
