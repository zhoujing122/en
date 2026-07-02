#pragma once
#include "active_scan_manager.hpp"
#include "common.hpp"
#include "map_quality.hpp"
#include "mapping_supervisor.hpp"

namespace robot_slamd {

enum class ActiveScanExecutionState {
    DISABLED,
    IDLE,
    PRECHECK,
    COMMANDING_ROTATION,
    VERIFYING_ROTATION,
    COMPLETED,
    ABORTED,
    COOLDOWN,
    BLOCKED,
    FAULT,
};

inline const char *active_scan_execution_state_name(ActiveScanExecutionState s) {
    switch (s) {
    case ActiveScanExecutionState::DISABLED: return "DISABLED";
    case ActiveScanExecutionState::IDLE: return "IDLE";
    case ActiveScanExecutionState::PRECHECK: return "PRECHECK";
    case ActiveScanExecutionState::COMMANDING_ROTATION: return "COMMANDING_ROTATION";
    case ActiveScanExecutionState::VERIFYING_ROTATION: return "VERIFYING_ROTATION";
    case ActiveScanExecutionState::COMPLETED: return "COMPLETED";
    case ActiveScanExecutionState::ABORTED: return "ABORTED";
    case ActiveScanExecutionState::COOLDOWN: return "COOLDOWN";
    case ActiveScanExecutionState::BLOCKED: return "BLOCKED";
    case ActiveScanExecutionState::FAULT: return "FAULT";
    }
    return "IDLE";
}

struct ActiveScanCommandInput {
    double timestamp_s = 0.0;
    ActiveScanSnapshot active_scan;
    MappingSupervisorSnapshot supervisor;
    MapQualitySnapshot map_quality;
    double pose_x_m = 0.0;
    double pose_y_m = 0.0;
    double yaw_rad = 0.0;
    double linear_speed_mps = 0.0;
    double yaw_rate_radps = 0.0;
    bool localization_valid = true;
};

struct ActiveScanCommandSnapshot {
    double timestamp_s = 0.0;
    std::string state = "IDLE";
    std::string previous_state = "IDLE";
    std::string reason = "idle";
    std::string request_type = "NONE";
    std::string mode = "dry_run";
    bool command_active = false;
    bool would_emit_command = false;
    bool zero_command = false;
    bool blocked = false;
    bool completed = false;
    bool aborted = false;
    bool cooldown = false;
    double desired_linear_speed_mps = 0.0;
    double desired_yaw_rate_radps = 0.0;
    double desired_yaw_rate_dps = 0.0;
    double target_scan_angle_deg = 0.0;
    double observed_yaw_delta_deg = 0.0;
    double observed_yaw_rate_dps = 0.0;
    double state_duration_s = 0.0;
    double command_duration_s = 0.0;
    double time_since_last_execution_s = 0.0;
    double quality_score = 0.0;
    std::string supervisor_state = "INIT";
    std::string active_scan_state = "IDLE";
    double linear_speed_mps = 0.0;
    bool localization_valid = true;
};

class ActiveScanCommandPlanner {
public:
    explicit ActiveScanCommandPlanner(const Config &cfg) : cfg_(cfg) {
        if (!enabled()) {
            state_ = ActiveScanExecutionState::DISABLED;
            previous_state_ = ActiveScanExecutionState::DISABLED;
            last_reason_ = "disabled";
            last_snapshot_.state = "DISABLED";
            last_snapshot_.previous_state = "DISABLED";
            last_snapshot_.reason = "disabled";
            last_snapshot_.mode = cfg_.active_scan_execution_mode;
        }
    }

    bool update(const ActiveScanCommandInput &input) {
        double now_s = input.timestamp_s;
        if (last_update_s_ < 0.0) {
            state_enter_s_ = now_s;
            last_update_s_ = now_s;
            last_yaw_rad_ = input.yaw_rad;
            have_last_yaw_ = true;
        } else {
            accumulate_state_time(now_s - last_update_s_);
            last_update_s_ = now_s;
        }

        if (!enabled()) {
            bool changed = state_ != ActiveScanExecutionState::DISABLED;
            if (changed) transition_to(ActiveScanExecutionState::DISABLED, now_s);
            last_reason_ = "disabled";
            last_snapshot_ = make_snapshot(input, now_s, last_reason_);
            return changed;
        }

        bool request_present = has_request(input);
        bool cooldown_active = cooldown_until_s_ >= 0.0 && now_s < cooldown_until_s_;
        bool ready = precheck_ready(input, cooldown_active);
        std::string block = block_reason(input, cooldown_active);

        ActiveScanExecutionState next = state_;
        std::string reason = "idle";

        switch (state_) {
        case ActiveScanExecutionState::DISABLED:
            next = ActiveScanExecutionState::IDLE;
            reason = "idle";
            break;
        case ActiveScanExecutionState::IDLE:
            reset_execution(input.yaw_rad);
            if (ready) {
                next = ActiveScanExecutionState::PRECHECK;
                reason = "precheck";
                precheck_start_s_ = now_s;
                current_request_type_ = input.active_scan.request_type;
            } else if (request_present) {
                next = ActiveScanExecutionState::BLOCKED;
                reason = block;
                current_request_type_ = input.active_scan.request_type;
            } else {
                reason = "idle";
                current_request_type_ = "NONE";
            }
            break;
        case ActiveScanExecutionState::PRECHECK:
            if (!request_present) {
                next = ActiveScanExecutionState::IDLE;
                reason = "idle";
                current_request_type_ = "NONE";
            } else if (!ready) {
                next = ActiveScanExecutionState::BLOCKED;
                reason = block;
                current_request_type_ = input.active_scan.request_type;
            } else if (precheck_start_s_ >= 0.0 && (now_s - precheck_start_s_) >= cfg_.active_scan_execution_precheck_hold_s) {
                next = ActiveScanExecutionState::COMMANDING_ROTATION;
                reason = "commanding_rotation";
                reset_execution(input.yaw_rad);
                command_start_s_ = now_s;
                last_execution_s_ = now_s;
                current_request_type_ = input.active_scan.request_type;
            } else {
                reason = "precheck";
                current_request_type_ = input.active_scan.request_type;
            }
            break;
        case ActiveScanExecutionState::COMMANDING_ROTATION:
            if (abort_condition(input, now_s, reason)) {
                next = ActiveScanExecutionState::ABORTED;
            } else {
                next = ActiveScanExecutionState::VERIFYING_ROTATION;
                reason = "verifying_rotation";
                current_request_type_ = input.active_scan.request_type;
            }
            break;
        case ActiveScanExecutionState::VERIFYING_ROTATION:
            if (abort_condition(input, now_s, reason)) {
                next = ActiveScanExecutionState::ABORTED;
            } else {
                accumulate_yaw_if_valid(input);
                if (observed_yaw_delta_deg_ >= cfg_.active_scan_execution_complete_scan_angle_deg) {
                    next = ActiveScanExecutionState::COMPLETED;
                    reason = "completed";
                } else {
                    reason = "verifying_rotation";
                }
                current_request_type_ = input.active_scan.request_type;
            }
            break;
        case ActiveScanExecutionState::COMPLETED:
            next = ActiveScanExecutionState::COOLDOWN;
            reason = "cooldown";
            cooldown_until_s_ = now_s + cfg_.active_scan_execution_cooldown_s;
            break;
        case ActiveScanExecutionState::ABORTED:
            next = ActiveScanExecutionState::COOLDOWN;
            reason = "cooldown";
            cooldown_until_s_ = now_s + cfg_.active_scan_execution_cooldown_s;
            break;
        case ActiveScanExecutionState::COOLDOWN:
            if (!cooldown_active) {
                next = ActiveScanExecutionState::IDLE;
                reason = "idle";
                current_request_type_ = "NONE";
                reset_execution(input.yaw_rad);
            } else {
                reason = "cooldown";
            }
            break;
        case ActiveScanExecutionState::BLOCKED:
            if (!request_present) {
                next = ActiveScanExecutionState::IDLE;
                reason = "idle";
                current_request_type_ = "NONE";
            } else if (ready) {
                next = ActiveScanExecutionState::PRECHECK;
                reason = "precheck";
                precheck_start_s_ = now_s;
                current_request_type_ = input.active_scan.request_type;
            } else {
                reason = block;
                current_request_type_ = input.active_scan.request_type;
            }
            break;
        case ActiveScanExecutionState::FAULT:
            reason = "hardware_mode_unsupported";
            break;
        }

        bool changed = next != state_;
        if (changed) {
            transition_to(next, now_s);
            if (state_ == ActiveScanExecutionState::PRECHECK) run_stats_.precheck_count++;
            if (state_ == ActiveScanExecutionState::COMMANDING_ROTATION) run_stats_.command_start_count++;
            if (state_ == ActiveScanExecutionState::BLOCKED) run_stats_.blocked_count++;
            if (state_ == ActiveScanExecutionState::COMPLETED) {
                run_stats_.completed_count++;
                if (cfg_.active_scan_execution_emit_zero_command_on_stop) run_stats_.zero_command_count++;
            }
            if (state_ == ActiveScanExecutionState::ABORTED) {
                run_stats_.aborted_count++;
                if (cfg_.active_scan_execution_emit_zero_command_on_stop) run_stats_.zero_command_count++;
            }
        }

        last_reason_ = reason;
        last_snapshot_ = make_snapshot(input, now_s, reason);
        return changed;
    }

    bool should_log(double now_s) const {
        if (!enabled()) return false;
        if (state_changed_since_log_) return true;
        double period_s = 1.0 / cfg_.active_scan_execution_log_hz;
        if (last_log_s_ < 0.0) return now_s >= period_s;
        return (now_s - last_log_s_) >= period_s;
    }

    ActiveScanCommandSnapshot snapshot() const { return last_snapshot_; }

    void mark_logged(double now_s, const ActiveScanCommandSnapshot &s) {
        last_log_s_ = now_s;
        state_changed_since_log_ = false;
        run_stats_.state_last = s.state;
        run_stats_.last_reason = s.reason;
        run_stats_.last_request_type = s.request_type;
        run_stats_.last_observed_yaw_delta_deg = s.observed_yaw_delta_deg;
        run_stats_.max_observed_yaw_delta_deg = std::max(run_stats_.max_observed_yaw_delta_deg, s.observed_yaw_delta_deg);
    }

    ActiveScanCommandRunStats run_stats(double now_s) const {
        ActiveScanCommandRunStats out = run_stats_;
        if (last_update_s_ >= 0.0 && now_s > last_update_s_) add_state_time(out, state_, now_s - last_update_s_);
        out.state_last = active_scan_execution_state_name(state_);
        out.last_reason = last_reason_;
        out.last_request_type = current_request_type_;
        out.last_observed_yaw_delta_deg = observed_yaw_delta_deg_;
        out.max_observed_yaw_delta_deg = std::max(out.max_observed_yaw_delta_deg, observed_yaw_delta_deg_);
        return out;
    }

private:
    bool enabled() const {
        return cfg_.active_scan_execution_enabled && cfg_.active_scan_execution_mode != "disabled";
    }

    void transition_to(ActiveScanExecutionState next, double now_s) {
        previous_state_ = state_;
        state_ = next;
        state_enter_s_ = now_s;
        run_stats_.state_changes++;
        state_changed_since_log_ = true;
    }

    void accumulate_state_time(double dt_s) {
        if (dt_s <= 0.0 || !std::isfinite(dt_s)) return;
        add_state_time(run_stats_, state_, dt_s);
    }

    static void add_state_time(ActiveScanCommandRunStats &stats, ActiveScanExecutionState state, double dt_s) {
        if (state == ActiveScanExecutionState::COMMANDING_ROTATION || state == ActiveScanExecutionState::VERIFYING_ROTATION) stats.command_active_seconds += dt_s;
        if (state == ActiveScanExecutionState::VERIFYING_ROTATION) stats.verifying_seconds += dt_s;
        else if (state == ActiveScanExecutionState::COOLDOWN) stats.cooldown_seconds += dt_s;
        else if (state == ActiveScanExecutionState::BLOCKED) stats.blocked_seconds += dt_s;
    }

    static bool has_request(const ActiveScanCommandInput &in) {
        return in.active_scan.scan_recommended || in.active_scan.request_type != "NONE";
    }

    bool precheck_ready(const ActiveScanCommandInput &in, bool cooldown_active) const {
        if (cooldown_active) return false;
        if (cfg_.active_scan_execution_require_active_scan_would_start && !in.active_scan.would_start_scan) return false;
        if (cfg_.active_scan_execution_require_scan_recommended && !in.active_scan.scan_recommended) return false;
        if (cfg_.active_scan_execution_require_localization_valid && !in.localization_valid) return false;
        if (std::fabs(in.linear_speed_mps) > cfg_.active_scan_execution_max_linear_speed_mps) return false;
        return in.active_scan.request_type != "NONE";
    }

    std::string block_reason(const ActiveScanCommandInput &in, bool cooldown_active) const {
        if (cooldown_active) return "blocked_cooldown";
        if (cfg_.active_scan_execution_require_active_scan_would_start && !in.active_scan.would_start_scan) return "blocked_no_would_start";
        if (cfg_.active_scan_execution_require_scan_recommended && !in.active_scan.scan_recommended) return "blocked_not_recommended";
        if (cfg_.active_scan_execution_require_localization_valid && !in.localization_valid) return "blocked_localization_invalid";
        if (std::fabs(in.linear_speed_mps) > cfg_.active_scan_execution_max_linear_speed_mps) return "blocked_linear_speed";
        return "precheck";
    }

    bool abort_condition(const ActiveScanCommandInput &in, double now_s, std::string &reason) const {
        double command_duration = command_start_s_ < 0.0 ? 0.0 : now_s - command_start_s_;
        if (command_duration > cfg_.active_scan_execution_command_timeout_s) {
            reason = "aborted_timeout";
            return true;
        }
        if (cfg_.active_scan_execution_require_localization_valid && !in.localization_valid) {
            reason = "aborted_localization_invalid";
            return true;
        }
        if (std::fabs(in.linear_speed_mps) > cfg_.active_scan_execution_max_linear_speed_mps) {
            reason = "aborted_linear_speed";
            return true;
        }
        double yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        if (yaw_rate_dps > cfg_.active_scan_execution_max_observed_yaw_rate_dps) {
            reason = "aborted_yaw_rate_too_high";
            return true;
        }
        if (!has_request(in)) {
            reason = "aborted_request_cleared";
            return true;
        }
        return false;
    }

    void reset_execution(double yaw_rad) {
        observed_yaw_delta_deg_ = 0.0;
        last_yaw_rad_ = yaw_rad;
        have_last_yaw_ = true;
        command_start_s_ = -1.0;
        precheck_start_s_ = -1.0;
    }

    void accumulate_yaw_if_valid(const ActiveScanCommandInput &in) {
        double yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        if (yaw_rate_dps >= cfg_.active_scan_execution_min_observed_yaw_rate_dps && yaw_rate_dps <= cfg_.active_scan_execution_max_observed_yaw_rate_dps) {
            if (have_last_yaw_) {
                observed_yaw_delta_deg_ += std::fabs(wrap_pi(in.yaw_rad - last_yaw_rad_)) * 180.0 / kPi;
            }
            last_yaw_rad_ = in.yaw_rad;
            have_last_yaw_ = true;
        }
    }

    bool command_state() const {
        return state_ == ActiveScanExecutionState::COMMANDING_ROTATION || state_ == ActiveScanExecutionState::VERIFYING_ROTATION;
    }

    bool zero_command_state() const {
        return cfg_.active_scan_execution_emit_zero_command_on_stop &&
               (state_ == ActiveScanExecutionState::COMPLETED || state_ == ActiveScanExecutionState::ABORTED);
    }

    ActiveScanCommandSnapshot make_snapshot(const ActiveScanCommandInput &in, double now_s, const std::string &reason) const {
        ActiveScanCommandSnapshot out;
        out.timestamp_s = now_s;
        out.state = active_scan_execution_state_name(state_);
        out.previous_state = active_scan_execution_state_name(previous_state_);
        out.reason = reason;
        out.request_type = current_request_type_.empty() ? in.active_scan.request_type : current_request_type_;
        out.mode = cfg_.active_scan_execution_mode;
        out.command_active = command_state();
        out.would_emit_command = command_state();
        out.zero_command = zero_command_state();
        out.blocked = state_ == ActiveScanExecutionState::BLOCKED;
        out.completed = state_ == ActiveScanExecutionState::COMPLETED;
        out.aborted = state_ == ActiveScanExecutionState::ABORTED;
        out.cooldown = state_ == ActiveScanExecutionState::COOLDOWN;
        out.desired_linear_speed_mps = 0.0;
        if (out.zero_command) {
            out.desired_yaw_rate_dps = 0.0;
            out.desired_yaw_rate_radps = 0.0;
        } else if (out.command_active) {
            out.desired_yaw_rate_dps = cfg_.active_scan_execution_target_yaw_rate_dps;
            out.desired_yaw_rate_radps = deg2rad(cfg_.active_scan_execution_target_yaw_rate_dps);
        }
        out.target_scan_angle_deg = cfg_.active_scan_execution_target_scan_angle_deg;
        out.observed_yaw_delta_deg = observed_yaw_delta_deg_;
        out.observed_yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        out.state_duration_s = now_s - state_enter_s_;
        out.command_duration_s = command_start_s_ < 0.0 ? 0.0 : now_s - command_start_s_;
        out.time_since_last_execution_s = last_execution_s_ < 0.0 ? -1.0 : now_s - last_execution_s_;
        out.quality_score = in.supervisor.quality_score;
        out.supervisor_state = in.supervisor.state;
        out.active_scan_state = in.active_scan.state;
        out.linear_speed_mps = in.linear_speed_mps;
        out.localization_valid = in.localization_valid;
        return out;
    }

    const Config &cfg_;
    ActiveScanExecutionState state_ = ActiveScanExecutionState::IDLE;
    ActiveScanExecutionState previous_state_ = ActiveScanExecutionState::IDLE;
    double state_enter_s_ = 0.0;
    double last_update_s_ = -1.0;
    double last_log_s_ = -1.0;
    double precheck_start_s_ = -1.0;
    double command_start_s_ = -1.0;
    double last_execution_s_ = -1.0;
    double cooldown_until_s_ = -1.0;
    double last_yaw_rad_ = 0.0;
    bool have_last_yaw_ = false;
    double observed_yaw_delta_deg_ = 0.0;
    std::string current_request_type_ = "NONE";
    std::string last_reason_ = "idle";
    bool state_changed_since_log_ = false;
    ActiveScanCommandSnapshot last_snapshot_;
    ActiveScanCommandRunStats run_stats_;
};

inline void write_active_scan_command_log_header(std::ofstream &o) {
    o << "timestamp_s,state,previous_state,state_duration_s,reason,mode,request_type,command_active,would_emit_command,zero_command,blocked,completed,aborted,cooldown,desired_linear_speed_mps,desired_yaw_rate_radps,desired_yaw_rate_dps,target_scan_angle_deg,observed_yaw_delta_deg,observed_yaw_rate_dps,command_duration_s,time_since_last_execution_s,quality_score,supervisor_state,active_scan_state,linear_speed_mps,localization_valid\n";
}

inline void write_active_scan_command_log_row(std::ofstream &o, const ActiveScanCommandSnapshot &s) {
    o << s.timestamp_s << "," << s.state << "," << s.previous_state << "," << s.state_duration_s << ","
      << s.reason << "," << s.mode << "," << s.request_type << "," << (s.command_active ? 1 : 0) << ","
      << (s.would_emit_command ? 1 : 0) << "," << (s.zero_command ? 1 : 0) << "," << (s.blocked ? 1 : 0) << ","
      << (s.completed ? 1 : 0) << "," << (s.aborted ? 1 : 0) << "," << (s.cooldown ? 1 : 0) << ","
      << s.desired_linear_speed_mps << "," << s.desired_yaw_rate_radps << "," << s.desired_yaw_rate_dps << ","
      << s.target_scan_angle_deg << "," << s.observed_yaw_delta_deg << "," << s.observed_yaw_rate_dps << ","
      << s.command_duration_s << "," << s.time_since_last_execution_s << "," << s.quality_score << ","
      << s.supervisor_state << "," << s.active_scan_state << "," << s.linear_speed_mps << ","
      << (s.localization_valid ? 1 : 0) << "\n";
}

} // namespace robot_slamd
