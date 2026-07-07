#pragma once
#include "robot_slamd/core/common.hpp"
#include "robot_slamd/mapping/map_quality.hpp"
#include "robot_slamd/mapping/mapping_supervisor.hpp"

namespace robot_slamd {

enum class ActiveScanState {
    IDLE,
    ARMED,
    PENDING,
    WOULD_SCAN,
    OBSERVING_ROTATION,
    COMPLETED,
    COOLDOWN,
    BLOCKED,
};

inline const char *active_scan_state_name(ActiveScanState s) {
    switch (s) {
    case ActiveScanState::IDLE: return "IDLE";
    case ActiveScanState::ARMED: return "ARMED";
    case ActiveScanState::PENDING: return "PENDING";
    case ActiveScanState::WOULD_SCAN: return "WOULD_SCAN";
    case ActiveScanState::OBSERVING_ROTATION: return "OBSERVING_ROTATION";
    case ActiveScanState::COMPLETED: return "COMPLETED";
    case ActiveScanState::COOLDOWN: return "COOLDOWN";
    case ActiveScanState::BLOCKED: return "BLOCKED";
    }
    return "IDLE";
}

enum class ActiveScanRequestType {
    NONE,
    STARTUP_SCAN,
    LOW_QUALITY_SCAN,
    LOW_UPDATE_SCAN,
    SINGLE_TOF_ROUTE_SCAN,
    DEGRADED_SCAN,
    LOST_RECOVERY_SCAN,
};

inline const char *active_scan_request_name(ActiveScanRequestType t) {
    switch (t) {
    case ActiveScanRequestType::NONE: return "NONE";
    case ActiveScanRequestType::STARTUP_SCAN: return "STARTUP_SCAN";
    case ActiveScanRequestType::LOW_QUALITY_SCAN: return "LOW_QUALITY_SCAN";
    case ActiveScanRequestType::LOW_UPDATE_SCAN: return "LOW_UPDATE_SCAN";
    case ActiveScanRequestType::SINGLE_TOF_ROUTE_SCAN: return "SINGLE_TOF_ROUTE_SCAN";
    case ActiveScanRequestType::DEGRADED_SCAN: return "DEGRADED_SCAN";
    case ActiveScanRequestType::LOST_RECOVERY_SCAN: return "LOST_RECOVERY_SCAN";
    }
    return "NONE";
}

struct ActiveScanInput {
    double timestamp_s = 0.0;
    MapQualitySnapshot map_quality;
    MappingSupervisorSnapshot supervisor;
    double pose_x_m = 0.0;
    double pose_y_m = 0.0;
    double yaw_rad = 0.0;
    double linear_speed_mps = 0.0;
    double yaw_rate_radps = 0.0;
    bool localization_valid = false;
};

struct ActiveScanSnapshot {
    double timestamp_s = 0.0;
    std::string state = "IDLE";
    std::string previous_state = "IDLE";
    std::string request_type = "NONE";
    std::string reason = "idle";
    bool scan_recommended = false;
    bool would_start_scan = false;
    bool blocked = false;
    bool observing_natural_rotation = false;
    bool completed = false;
    bool useful_scan_observed = false;
    double state_duration_s = 0.0;
    double time_since_last_scan_s = 0.0;
    double pending_duration_s = 0.0;
    double recommended_scan_angle_deg = 0.0;
    double observed_yaw_delta_deg = 0.0;
    double observed_yaw_rate_dps = 0.0;
    double quality_score = 0.0;
    std::string supervisor_state = "INIT";
    std::string supervisor_reason = "startup";
    double tof_valid_ratio = 0.0;
    int valid_tof_routes = 0;
    double map_update_rate_hz = 0.0;
    double linear_speed_mps = 0.0;
};

class ActiveScanManager {
public:
    explicit ActiveScanManager(const Config &cfg) : cfg_(cfg) {}

    bool update(const ActiveScanInput &input) {
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

        ActiveScanRequestType req = determine_request(input);
        bool request_present = req != ActiveScanRequestType::NONE;
        bool cooldown_active = (cooldown_until_s_ >= 0.0 && now_s < cooldown_until_s_) ||
                               (last_scan_s_ >= 0.0 && (now_s - last_scan_s_) < cfg_.active_scan_min_interval_s);
        bool supervisor_ok = supervisor_allows_request(input, req);
        bool request_allowed = request_present && supervisor_ok;
        bool can_scan = can_start_scan(input);
        std::string block_reason = blocked_reason(input, cooldown_active);
        bool yaw_rate_ok = yaw_rate_in_observed_range(input.yaw_rate_radps);

        ActiveScanState next = state_;
        ActiveScanRequestType next_req = req;
        std::string reason = request_reason(req);

        if (!cfg_.active_scan_enabled) {
            next = ActiveScanState::IDLE;
            next_req = ActiveScanRequestType::NONE;
            reason = "idle";
        } else {
            switch (state_) {
            case ActiveScanState::IDLE:
                reset_observed_session(input.yaw_rad);
                if (request_allowed && !cooldown_active) {
                    next = ActiveScanState::ARMED;
                    reason = request_reason(req);
                    request_start_s_ = now_s;
                    pending_start_s_ = -1.0;
                    current_request_ = req;
                    if (req == ActiveScanRequestType::STARTUP_SCAN) startup_scan_requested_ = true;
                } else if (request_present && cooldown_active) {
                    next = ActiveScanState::BLOCKED;
                    reason = "blocked_cooldown";
                    current_request_ = req;
                } else {
                    next_req = ActiveScanRequestType::NONE;
                    reason = "idle";
                    current_request_ = ActiveScanRequestType::NONE;
                }
                break;
            case ActiveScanState::ARMED:
                if (!request_allowed) {
                    next = ActiveScanState::IDLE;
                    next_req = ActiveScanRequestType::NONE;
                    reason = "request_cleared";
                    current_request_ = ActiveScanRequestType::NONE;
                    request_start_s_ = -1.0;
                } else if (cooldown_active) {
                    next = ActiveScanState::BLOCKED;
                    reason = "blocked_cooldown";
                    current_request_ = req;
                } else if (request_start_s_ >= 0.0 && (now_s - request_start_s_) >= cfg_.active_scan_request_hold_s) {
                    next = ActiveScanState::PENDING;
                    reason = "pending_scan";
                    current_request_ = req;
                    pending_start_s_ = now_s;
                } else {
                    reason = "request_hold";
                    current_request_ = req;
                }
                break;
            case ActiveScanState::PENDING:
                if (!request_allowed) {
                    next = ActiveScanState::IDLE;
                    next_req = ActiveScanRequestType::NONE;
                    reason = "request_cleared";
                    current_request_ = ActiveScanRequestType::NONE;
                    pending_start_s_ = -1.0;
                } else if (!can_scan || cooldown_active) {
                    next = ActiveScanState::BLOCKED;
                    reason = block_reason;
                    current_request_ = req;
                } else {
                    next = ActiveScanState::WOULD_SCAN;
                    reason = "would_start_scan";
                    current_request_ = req;
                    last_scan_s_ = now_s;
                    cooldown_until_s_ = now_s + std::max(cfg_.active_scan_cooldown_s, cfg_.active_scan_min_interval_s);
                }
                break;
            case ActiveScanState::WOULD_SCAN:
                if (!request_allowed) {
                    next = ActiveScanState::IDLE;
                    next_req = ActiveScanRequestType::NONE;
                    reason = "request_cleared";
                    current_request_ = ActiveScanRequestType::NONE;
                } else if (cfg_.active_scan_observe_natural_rotation && yaw_rate_ok) {
                    next = ActiveScanState::OBSERVING_ROTATION;
                    reason = "observing_natural_rotation";
                    current_request_ = req;
                    reset_observed_session(input.yaw_rad);
                } else if (!can_scan) {
                    next = ActiveScanState::BLOCKED;
                    reason = block_reason;
                    current_request_ = req;
                } else {
                    reason = "would_start_scan";
                    current_request_ = req;
                }
                break;
            case ActiveScanState::OBSERVING_ROTATION:
                current_request_ = req;
                if (cfg_.active_scan_observe_natural_rotation && yaw_rate_ok) {
                    accumulate_yaw(input.yaw_rad);
                } else if (!can_scan) {
                    next = ActiveScanState::BLOCKED;
                    reason = block_reason;
                    break;
                }
                if (observed_yaw_delta_deg_ >= cfg_.active_scan_complete_scan_angle_deg) {
                    next = ActiveScanState::COMPLETED;
                    reason = "scan_completed";
                    useful_scan_observed_ = true;
                } else {
                    reason = "observing_natural_rotation";
                    if (!request_allowed && observed_yaw_delta_deg_ < cfg_.active_scan_min_useful_scan_angle_deg) {
                        next = ActiveScanState::IDLE;
                        next_req = ActiveScanRequestType::NONE;
                        reason = "request_cleared";
                        current_request_ = ActiveScanRequestType::NONE;
                    }
                }
                break;
            case ActiveScanState::COMPLETED:
                next = ActiveScanState::COOLDOWN;
                reason = "scan_cooldown";
                cooldown_until_s_ = std::max(cooldown_until_s_, now_s + std::max(cfg_.active_scan_cooldown_s, cfg_.active_scan_min_interval_s));
                current_request_ = req;
                break;
            case ActiveScanState::COOLDOWN:
                reason = "scan_cooldown";
                if (!cooldown_active) {
                    next = ActiveScanState::IDLE;
                    next_req = ActiveScanRequestType::NONE;
                    reason = "idle";
                    current_request_ = ActiveScanRequestType::NONE;
                    reset_observed_session(input.yaw_rad);
                } else {
                    current_request_ = req;
                }
                break;
            case ActiveScanState::BLOCKED:
                if (!request_allowed) {
                    next = ActiveScanState::IDLE;
                    next_req = ActiveScanRequestType::NONE;
                    reason = "request_cleared";
                    current_request_ = ActiveScanRequestType::NONE;
                } else if (cooldown_active) {
                    reason = "blocked_cooldown";
                    current_request_ = req;
                } else if (can_scan) {
                    next = ActiveScanState::PENDING;
                    reason = "pending_scan";
                    current_request_ = req;
                    if (pending_start_s_ < 0.0) pending_start_s_ = now_s;
                } else {
                    reason = block_reason;
                    current_request_ = req;
                }
                break;
            }
        }

        if (state_ != ActiveScanState::OBSERVING_ROTATION) {
            last_yaw_rad_ = input.yaw_rad;
            have_last_yaw_ = true;
        }

        bool changed = next != state_;
        if (changed) {
            previous_state_ = state_;
            state_ = next;
            state_enter_s_ = now_s;
            run_stats_.state_changes++;
            state_changed_since_log_ = true;
            if (state_ == ActiveScanState::ARMED) run_stats_.request_count++;
            if (state_ == ActiveScanState::WOULD_SCAN) run_stats_.would_start_count++;
            if (state_ == ActiveScanState::BLOCKED) run_stats_.blocked_count++;
            if (state_ == ActiveScanState::COMPLETED) run_stats_.completed_count++;
            if (state_ == ActiveScanState::COMPLETED && useful_scan_observed_) run_stats_.useful_observed_count++;
        }
        if (state_ == ActiveScanState::COMPLETED && observed_yaw_delta_deg_ >= cfg_.active_scan_min_useful_scan_angle_deg) useful_scan_observed_ = true;
        if (observed_yaw_delta_deg_ >= cfg_.active_scan_min_useful_scan_angle_deg) useful_scan_observed_ = true;

        if (current_request_ == ActiveScanRequestType::NONE) current_request_ = next_req;
        if (next_req != ActiveScanRequestType::NONE) current_request_ = next_req;
        last_reason_ = reason;
        last_snapshot_ = make_snapshot(input, now_s, reason);
        return changed;
    }

    bool should_log(double now_s) const {
        if (!cfg_.active_scan_enabled) return false;
        if (state_changed_since_log_) return true;
        double period_s = 1.0 / cfg_.active_scan_log_hz;
        if (last_log_s_ < 0.0) return now_s >= period_s;
        return (now_s - last_log_s_) >= period_s;
    }

    ActiveScanSnapshot snapshot() const { return last_snapshot_; }

    void mark_logged(double now_s, const ActiveScanSnapshot &s) {
        last_log_s_ = now_s;
        state_changed_since_log_ = false;
        run_stats_.state_last = s.state;
        run_stats_.last_request_type = s.request_type;
        run_stats_.last_reason = s.reason;
        run_stats_.last_observed_yaw_delta_deg = s.observed_yaw_delta_deg;
        run_stats_.max_observed_yaw_delta_deg = std::max(run_stats_.max_observed_yaw_delta_deg, s.observed_yaw_delta_deg);
    }

    ActiveScanRunStats run_stats(double now_s) const {
        ActiveScanRunStats out = run_stats_;
        if (last_update_s_ >= 0.0 && now_s > last_update_s_) add_state_time(out, state_, now_s - last_update_s_);
        out.state_last = active_scan_state_name(state_);
        out.last_request_type = active_scan_request_name(current_request_);
        out.last_reason = last_reason_;
        out.last_observed_yaw_delta_deg = observed_yaw_delta_deg_;
        out.max_observed_yaw_delta_deg = std::max(out.max_observed_yaw_delta_deg, observed_yaw_delta_deg_);
        return out;
    }

private:
    void accumulate_state_time(double dt_s) {
        if (dt_s <= 0.0 || !std::isfinite(dt_s)) return;
        add_state_time(run_stats_, state_, dt_s);
    }

    static void add_state_time(ActiveScanRunStats &stats, ActiveScanState state, double dt_s) {
        if (state == ActiveScanState::ARMED || state == ActiveScanState::PENDING || state == ActiveScanState::WOULD_SCAN) stats.recommended_seconds += dt_s;
        else if (state == ActiveScanState::OBSERVING_ROTATION) stats.observing_seconds += dt_s;
        else if (state == ActiveScanState::COOLDOWN) stats.cooldown_seconds += dt_s;
        else if (state == ActiveScanState::BLOCKED) stats.blocked_seconds += dt_s;
    }

    static bool contains(const std::string &s, const char *needle) {
        return s.find(needle) != std::string::npos;
    }

    ActiveScanRequestType determine_request(const ActiveScanInput &in) const {
        const auto &s = in.supervisor;
        if (cfg_.active_scan_lost_scan_enabled && (s.lost || s.state == "LOST")) return ActiveScanRequestType::LOST_RECOVERY_SCAN;
        if (cfg_.active_scan_degraded_scan_enabled && (s.degraded || s.state == "DEGRADED") && s.valid_tof_routes >= 1) return ActiveScanRequestType::DEGRADED_SCAN;
        bool supervisor_recommends = s.active_scan_recommended || s.state == "ACTIVE_SCAN_RECOMMENDED";
        if (cfg_.active_scan_low_quality_scan_enabled && supervisor_recommends) {
            if (s.valid_tof_routes < 2 || contains(s.reason, "single_tof_route_active_scan") || contains(s.reason, "single_tof_route_only")) return ActiveScanRequestType::SINGLE_TOF_ROUTE_SCAN;
            if (contains(s.reason, "low_map_update_active_scan") || contains(s.reason, "low_map_update_rate")) return ActiveScanRequestType::LOW_UPDATE_SCAN;
            if (contains(s.reason, "low_quality_active_scan") || contains(s.reason, "low_tof_valid_ratio") || contains(s.reason, "active_scan_recommended")) return ActiveScanRequestType::LOW_QUALITY_SCAN;
            return ActiveScanRequestType::LOW_QUALITY_SCAN;
        }
        if (cfg_.active_scan_startup_scan_enabled && !startup_scan_requested_ && (s.state == "MAPPING_READY" || s.state == "MAPPING")) return ActiveScanRequestType::STARTUP_SCAN;
        return ActiveScanRequestType::NONE;
    }

    bool supervisor_allows_request(const ActiveScanInput &in, ActiveScanRequestType req) const {
        if (req == ActiveScanRequestType::NONE) return false;
        if (!cfg_.active_scan_require_supervisor_recommendation) return true;
        const auto &s = in.supervisor;
        if (req == ActiveScanRequestType::STARTUP_SCAN) return true;
        return s.active_scan_recommended || s.state == "ACTIVE_SCAN_RECOMMENDED" || s.degraded || s.lost || s.state == "DEGRADED" || s.state == "LOST";
    }

    bool can_start_scan(const ActiveScanInput &in) const {
        return std::fabs(in.linear_speed_mps) <= cfg_.active_scan_max_linear_speed_for_scan_mps &&
               in.supervisor.valid_tof_routes >= cfg_.active_scan_min_valid_tof_routes_for_scan &&
               in.supervisor.tof_valid_ratio >= cfg_.active_scan_min_tof_valid_ratio_for_scan &&
               in.localization_valid;
    }

    std::string blocked_reason(const ActiveScanInput &in, bool cooldown_active) const {
        if (cooldown_active) return "blocked_cooldown";
        if (std::fabs(in.linear_speed_mps) > cfg_.active_scan_max_linear_speed_for_scan_mps) return "blocked_linear_speed";
        if (in.supervisor.valid_tof_routes < cfg_.active_scan_min_valid_tof_routes_for_scan) return "blocked_no_valid_tof";
        if (in.supervisor.tof_valid_ratio < cfg_.active_scan_min_tof_valid_ratio_for_scan) return "blocked_low_tof_valid_ratio";
        if (!in.localization_valid) return "blocked_localization_invalid";
        return "pending_scan";
    }

    bool yaw_rate_in_observed_range(double yaw_rate_radps) const {
        double dps = std::fabs(yaw_rate_radps) * 180.0 / kPi;
        return dps >= cfg_.active_scan_min_yaw_rate_for_observed_scan_dps && dps <= cfg_.active_scan_max_yaw_rate_for_observed_scan_dps;
    }

    void reset_observed_session(double yaw_rad) {
        observed_yaw_delta_deg_ = 0.0;
        useful_scan_observed_ = false;
        last_yaw_rad_ = yaw_rad;
        have_last_yaw_ = true;
    }

    void accumulate_yaw(double yaw_rad) {
        if (have_last_yaw_) {
            double delta = wrap_pi(yaw_rad - last_yaw_rad_);
            observed_yaw_delta_deg_ += std::fabs(delta) * 180.0 / kPi;
        }
        last_yaw_rad_ = yaw_rad;
        have_last_yaw_ = true;
    }

    std::string request_reason(ActiveScanRequestType req) const {
        switch (req) {
        case ActiveScanRequestType::STARTUP_SCAN: return "startup_scan";
        case ActiveScanRequestType::LOW_QUALITY_SCAN: return "low_quality_scan";
        case ActiveScanRequestType::LOW_UPDATE_SCAN: return "low_update_scan";
        case ActiveScanRequestType::SINGLE_TOF_ROUTE_SCAN: return "single_tof_route_scan";
        case ActiveScanRequestType::DEGRADED_SCAN: return "degraded_scan";
        case ActiveScanRequestType::LOST_RECOVERY_SCAN: return "lost_recovery_scan";
        case ActiveScanRequestType::NONE: return "idle";
        }
        return "idle";
    }

    ActiveScanSnapshot make_snapshot(const ActiveScanInput &in, double now_s, const std::string &reason) const {
        ActiveScanSnapshot out;
        out.timestamp_s = now_s;
        out.state = active_scan_state_name(state_);
        out.previous_state = active_scan_state_name(previous_state_);
        out.request_type = active_scan_request_name(current_request_);
        out.reason = reason;
        out.scan_recommended = current_request_ != ActiveScanRequestType::NONE && state_ != ActiveScanState::IDLE && state_ != ActiveScanState::COOLDOWN;
        out.would_start_scan = state_ == ActiveScanState::WOULD_SCAN;
        out.blocked = state_ == ActiveScanState::BLOCKED;
        out.observing_natural_rotation = state_ == ActiveScanState::OBSERVING_ROTATION;
        out.completed = state_ == ActiveScanState::COMPLETED;
        out.useful_scan_observed = useful_scan_observed_ || observed_yaw_delta_deg_ >= cfg_.active_scan_min_useful_scan_angle_deg;
        out.state_duration_s = now_s - state_enter_s_;
        out.time_since_last_scan_s = last_scan_s_ < 0.0 ? -1.0 : now_s - last_scan_s_;
        out.pending_duration_s = pending_start_s_ < 0.0 ? 0.0 : now_s - pending_start_s_;
        out.recommended_scan_angle_deg = cfg_.active_scan_recommended_scan_angle_deg;
        out.observed_yaw_delta_deg = observed_yaw_delta_deg_;
        out.observed_yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        out.quality_score = in.supervisor.quality_score;
        out.supervisor_state = in.supervisor.state;
        out.supervisor_reason = in.supervisor.reason;
        out.tof_valid_ratio = in.supervisor.tof_valid_ratio;
        out.valid_tof_routes = in.supervisor.valid_tof_routes;
        out.map_update_rate_hz = in.supervisor.map_update_rate_hz;
        out.linear_speed_mps = in.linear_speed_mps;
        return out;
    }

    const Config &cfg_;
    ActiveScanState state_ = ActiveScanState::IDLE;
    ActiveScanState previous_state_ = ActiveScanState::IDLE;
    ActiveScanRequestType current_request_ = ActiveScanRequestType::NONE;
    double state_enter_s_ = 0.0;
    double last_update_s_ = -1.0;
    double last_log_s_ = -1.0;
    double request_start_s_ = -1.0;
    double pending_start_s_ = -1.0;
    double last_scan_s_ = -1.0;
    double cooldown_until_s_ = -1.0;
    double last_yaw_rad_ = 0.0;
    bool have_last_yaw_ = false;
    double observed_yaw_delta_deg_ = 0.0;
    bool useful_scan_observed_ = false;
    bool startup_scan_requested_ = false;
    std::string last_reason_ = "idle";
    bool state_changed_since_log_ = false;
    ActiveScanSnapshot last_snapshot_;
    ActiveScanRunStats run_stats_;
};

inline void write_active_scan_log_header(std::ofstream &o) {
    o << "timestamp_s,state,previous_state,state_duration_s,request_type,reason,scan_recommended,would_start_scan,blocked,observing_natural_rotation,completed,useful_scan_observed,recommended_scan_angle_deg,observed_yaw_delta_deg,observed_yaw_rate_dps,time_since_last_scan_s,pending_duration_s,quality_score,supervisor_state,supervisor_reason,tof_valid_ratio,valid_tof_routes,map_update_rate_hz,linear_speed_mps\n";
}

inline void write_active_scan_log_row(std::ofstream &o, const ActiveScanSnapshot &s) {
    o << s.timestamp_s << "," << s.state << "," << s.previous_state << "," << s.state_duration_s << ","
      << s.request_type << "," << s.reason << "," << (s.scan_recommended ? 1 : 0) << ","
      << (s.would_start_scan ? 1 : 0) << "," << (s.blocked ? 1 : 0) << ","
      << (s.observing_natural_rotation ? 1 : 0) << "," << (s.completed ? 1 : 0) << ","
      << (s.useful_scan_observed ? 1 : 0) << "," << s.recommended_scan_angle_deg << ","
      << s.observed_yaw_delta_deg << "," << s.observed_yaw_rate_dps << "," << s.time_since_last_scan_s << ","
      << s.pending_duration_s << "," << s.quality_score << "," << s.supervisor_state << ","
      << s.supervisor_reason << "," << s.tof_valid_ratio << "," << s.valid_tof_routes << ","
      << s.map_update_rate_hz << "," << s.linear_speed_mps << "\n";
}

} // namespace robot_slamd
