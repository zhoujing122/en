#pragma once
#include "robot_slamd/active_scan/active_scan_command_planner.hpp"
#include "robot_slamd/active_scan/active_scan_manager.hpp"
#include "robot_slamd/core/common.hpp"
#include "robot_slamd/mapping/map_quality.hpp"
#include "robot_slamd/mapping/mapping_supervisor.hpp"
#include "robot_slamd/active_scan/sparse_scan_yaw_matcher.hpp"
#include "robot_slamd/active_scan/yaw_correction_apply.hpp"
#include "robot_slamd/active_scan/yaw_correction_gate.hpp"
#include "robot_slamd/active_scan/yaw_correction_post_apply_validator.hpp"

namespace robot_slamd {

enum class RecoveryManagerState {
    DISABLED,
    IDLE,
    STARTUP_OBSERVE,
    MAPPING_OK,
    DEGRADED_OBSERVE,
    LOST_OBSERVE,
    RECOVERY_SCAN_RECOMMENDED,
    WAITING_FOR_RECOVERY_SCAN,
    CANDIDATE_EVALUATION,
    CANDIDATE_READY,
    REJECTED,
    COOLDOWN,
};

inline const char *recovery_manager_state_name(RecoveryManagerState s) {
    switch (s) {
    case RecoveryManagerState::DISABLED: return "DISABLED";
    case RecoveryManagerState::IDLE: return "IDLE";
    case RecoveryManagerState::STARTUP_OBSERVE: return "STARTUP_OBSERVE";
    case RecoveryManagerState::MAPPING_OK: return "MAPPING_OK";
    case RecoveryManagerState::DEGRADED_OBSERVE: return "DEGRADED_OBSERVE";
    case RecoveryManagerState::LOST_OBSERVE: return "LOST_OBSERVE";
    case RecoveryManagerState::RECOVERY_SCAN_RECOMMENDED: return "RECOVERY_SCAN_RECOMMENDED";
    case RecoveryManagerState::WAITING_FOR_RECOVERY_SCAN: return "WAITING_FOR_RECOVERY_SCAN";
    case RecoveryManagerState::CANDIDATE_EVALUATION: return "CANDIDATE_EVALUATION";
    case RecoveryManagerState::CANDIDATE_READY: return "CANDIDATE_READY";
    case RecoveryManagerState::REJECTED: return "REJECTED";
    case RecoveryManagerState::COOLDOWN: return "COOLDOWN";
    }
    return "IDLE";
}

struct RecoveryManagerInput {
    double timestamp_s = 0.0;
    bool localizer_initialized = false;
    Pose current_pose;
    double linear_speed_mps = 0.0;
    double yaw_rate_radps = 0.0;
    MapQualitySnapshot map_quality;
    MappingSupervisorSnapshot supervisor;
    ActiveScanSnapshot active_scan;
    ActiveScanCommandSnapshot active_scan_command;
    SparseScanYawMatchSummary latest_yaw_match;
    YawCorrectionGateSnapshot latest_yaw_gate;
    YawCorrectionApplySnapshot latest_yaw_apply;
    YawCorrectionPostApplySnapshot latest_post_apply;
    bool tof_recent = false;
    double latest_tof_age_s = 0.0;
    uint64_t map_known_cells = 0;
    uint64_t map_occupied_cells = 0;
};

struct RecoveryManagerSnapshot {
    double timestamp_s = 0.0;
    std::string state = "IDLE";
    std::string previous_state = "IDLE";
    std::string reason = "idle";
    bool startup_recovery_observe = false;
    bool degraded_recovery_observe = false;
    bool lost_recovery_observe = false;
    bool recovery_scan_recommended = false;
    bool waiting_for_recovery_scan = false;
    bool candidate_seen = false;
    bool candidate_ready = false;
    bool rejected = false;
    std::string recovery_type = "none";
    std::string missing_evidence = "none";
    bool map_evidence_ok = false;
    bool tof_evidence_ok = false;
    bool localizer_evidence_ok = false;
    bool yaw_match_evidence_ok = false;
    bool yaw_correction_stable = false;
    uint64_t latest_match_scan_id = 0;
    double latest_match_timestamp_s = 0.0;
    double latest_yaw_candidate_deg = 0.0;
    double latest_yaw_match_score = 0.0;
    double latest_yaw_match_margin = 0.0;
    double latest_yaw_match_inlier_ratio = 0.0;
    std::string supervisor_state = "INIT";
    std::string map_quality_level = "UNKNOWN";
    double time_in_lost_s = 0.0;
    double time_in_degraded_s = 0.0;
    double time_since_last_recovery_scan_s = -1.0;
};

class RecoveryManager {
public:
    explicit RecoveryManager(const Config &cfg) : cfg_(cfg) {
        if (!enabled()) {
            state_ = RecoveryManagerState::DISABLED;
            latest_.state = "DISABLED";
            latest_.reason = "disabled";
        }
    }

    bool update(const RecoveryManagerInput &in) {
        update_supervisor_duration(in);
        if (in.latest_post_apply.state == "VALIDATED" &&
            std::isfinite(in.latest_post_apply.timestamp_s) &&
            std::fabs(in.latest_post_apply.timestamp_s - last_post_apply_validated_timestamp_s_) > 1e-9) {
            post_apply_validated_observed_count_++;
            last_post_apply_validated_timestamp_s_ = in.latest_post_apply.timestamp_s;
        }
        RecoveryManagerSnapshot s = base_snapshot(in);
        if (!enabled()) {
            return set_snapshot(s, RecoveryManagerState::DISABLED, "disabled", "none", false);
        }
        if (cfg_.recovery_mode != "observe_only") {
            return set_snapshot(s, RecoveryManagerState::DISABLED, "disabled", "none", false);
        }

        const bool startup_ctx = cfg_.recovery_startup_recovery_enabled && in.timestamp_s <= cfg_.recovery_startup_grace_s;
        const bool degraded_raw = cfg_.recovery_degraded_recovery_enabled && in.supervisor.state == "DEGRADED";
        const bool lost_raw = cfg_.recovery_lost_recovery_enabled && in.supervisor.state == "LOST";
        const bool degraded_ctx = degraded_raw && s.time_in_degraded_s >= cfg_.recovery_degraded_confirm_s;
        const bool lost_ctx = lost_raw && s.time_in_lost_s >= cfg_.recovery_lost_confirm_s;

        if (startup_ctx) {
            s.recovery_type = "startup";
            s.startup_recovery_observe = true;
            return evaluate_context(in, s, "startup");
        }
        if (lost_ctx) {
            s.recovery_type = "lost";
            s.lost_recovery_observe = true;
            return evaluate_context(in, s, "lost");
        }
        if (degraded_ctx) {
            s.recovery_type = "degraded";
            s.degraded_recovery_observe = true;
            return evaluate_context(in, s, "degraded");
        }
        if (lost_raw) {
            s.recovery_type = "lost";
            s.lost_recovery_observe = true;
            return set_snapshot(s, RecoveryManagerState::LOST_OBSERVE, "lost_confirm_wait", "none", false);
        }
        if (degraded_raw) {
            s.recovery_type = "degraded";
            s.degraded_recovery_observe = true;
            return set_snapshot(s, RecoveryManagerState::DEGRADED_OBSERVE, "degraded_confirm_wait", "none", false);
        }
        return set_snapshot(s, RecoveryManagerState::MAPPING_OK, "mapping_ok", "none", false);
    }

    bool should_log(double now_s) const {
        if (!enabled()) return false;
        if (state_changed_since_log_) return true;
        if (last_log_s_ < 0.0) return true;
        return now_s - last_log_s_ >= (1.0 / cfg_.recovery_log_hz);
    }

    void mark_logged(double now_s, const RecoveryManagerSnapshot &) {
        last_log_s_ = now_s;
        state_changed_since_log_ = false;
    }

    RecoveryManagerSnapshot snapshot() const { return latest_; }
    RecoveryManagerRunStats run_stats(double) const {
        RecoveryManagerRunStats out = stats_;
        out.state_last = recovery_manager_state_name(state_);
        return out;
    }

private:
    bool enabled() const { return cfg_.recovery_enabled && cfg_.recovery_mode != "disabled"; }

    void update_supervisor_duration(const RecoveryManagerInput &in) {
        if (!have_supervisor_state_) {
            supervisor_state_ = in.supervisor.state;
            supervisor_state_since_s_ = in.timestamp_s;
            have_supervisor_state_ = true;
            return;
        }
        if (in.supervisor.state != supervisor_state_) {
            supervisor_state_ = in.supervisor.state;
            supervisor_state_since_s_ = in.timestamp_s;
        }
    }

    RecoveryManagerSnapshot base_snapshot(const RecoveryManagerInput &in) const {
        RecoveryManagerSnapshot s;
        s.timestamp_s = in.timestamp_s;
        s.state = recovery_manager_state_name(state_);
        s.previous_state = recovery_manager_state_name(previous_state_);
        s.map_evidence_ok = map_evidence_ok(in);
        s.tof_evidence_ok = tof_evidence_ok(in);
        s.localizer_evidence_ok = localizer_evidence_ok(in);
        s.yaw_match_evidence_ok = yaw_match_evidence_ok(in);
        s.yaw_correction_stable = yaw_correction_stable(in);
        s.latest_match_scan_id = in.latest_yaw_match.scan_id;
        s.latest_match_timestamp_s = in.latest_yaw_match.timestamp_s;
        s.latest_yaw_candidate_deg = in.latest_yaw_match.best_yaw_delta_deg;
        s.latest_yaw_match_score = in.latest_yaw_match.best_score;
        s.latest_yaw_match_margin = in.latest_yaw_match.score_margin;
        s.latest_yaw_match_inlier_ratio = in.latest_yaw_match.inlier_ratio;
        s.supervisor_state = in.supervisor.state;
        s.map_quality_level = in.map_quality.quality_level.empty() ? "UNKNOWN" : in.map_quality.quality_level;
        s.time_in_degraded_s = in.supervisor.state == "DEGRADED" ? std::max(0.0, in.timestamp_s - supervisor_state_since_s_) : 0.0;
        s.time_in_lost_s = in.supervisor.state == "LOST" ? std::max(0.0, in.timestamp_s - supervisor_state_since_s_) : 0.0;
        s.time_since_last_recovery_scan_s = last_recovery_scan_s_ >= 0.0 ? in.timestamp_s - last_recovery_scan_s_ : -1.0;
        return s;
    }

    bool evaluate_context(const RecoveryManagerInput &in, RecoveryManagerSnapshot &s, const std::string &type) {
        const bool startup = type == "startup";
        const bool lost = type == "lost";
        const bool localizer_required = !startup || !cfg_.recovery_allow_uninitialized_startup_recovery_observe_only;
        s.candidate_seen = s.yaw_match_evidence_ok;

        if (s.candidate_seen) {
            if (lost && !s.localizer_evidence_ok) {
                s.candidate_ready = false;
                return set_snapshot(s, RecoveryManagerState::REJECTED, "pose_reference_untrusted", "localizer", true);
            }
            if (localizer_required && !s.localizer_evidence_ok) {
                return reject_missing(s, "localizer_evidence_missing", "localizer");
            }
            if (!s.map_evidence_ok) return reject_missing(s, "map_evidence_missing", "map");
            if (!s.tof_evidence_ok) return reject_missing(s, "tof_evidence_missing", "tof");
            if (!s.yaw_correction_stable) return reject_missing(s, "yaw_correction_unstable", "yaw_correction");
            if (startup && !in.localizer_initialized) {
                s.candidate_ready = false;
                return set_snapshot(s, RecoveryManagerState::CANDIDATE_EVALUATION, "startup_uninitialized_observe_only", "localizer", false);
            }
            s.candidate_ready = true;
            return set_snapshot(s, RecoveryManagerState::CANDIDATE_READY, "candidate_ready", "none", false);
        }

        if (!s.map_evidence_ok) return reject_missing(s, "map_evidence_missing", "map");
        if (!s.tof_evidence_ok) return reject_missing(s, "tof_evidence_missing", "tof");
        if (localizer_required && !s.localizer_evidence_ok) return reject_missing(s, "localizer_evidence_missing", "localizer");
        if (!s.yaw_correction_stable) return reject_missing(s, "yaw_correction_unstable", "yaw_correction");

        if (in.timestamp_s < cooldown_until_s_) {
            s.waiting_for_recovery_scan = true;
            return set_snapshot(s, RecoveryManagerState::WAITING_FOR_RECOVERY_SCAN, "waiting_for_recovery_scan", "none", false);
        }
        if (cfg_.recovery_request_recovery_scan) {
            s.recovery_scan_recommended = true;
            last_recovery_scan_s_ = in.timestamp_s;
            cooldown_until_s_ = in.timestamp_s + cfg_.recovery_recovery_scan_cooldown_s;
            return set_snapshot(s, RecoveryManagerState::RECOVERY_SCAN_RECOMMENDED, "recovery_scan_recommended", "none", false);
        }
        return set_snapshot(s, state_for_context(type), type + "_observe", "none", false);
    }

    RecoveryManagerState state_for_context(const std::string &type) const {
        if (type == "startup") return RecoveryManagerState::STARTUP_OBSERVE;
        if (type == "degraded") return RecoveryManagerState::DEGRADED_OBSERVE;
        if (type == "lost") return RecoveryManagerState::LOST_OBSERVE;
        return RecoveryManagerState::IDLE;
    }

    bool reject_missing(RecoveryManagerSnapshot &s, const std::string &reason, const std::string &missing) {
        return set_snapshot(s, RecoveryManagerState::REJECTED, reason, missing, true);
    }

    bool set_snapshot(RecoveryManagerSnapshot &s, RecoveryManagerState next, const std::string &reason,
                      const std::string &missing, bool rejected) {
        previous_state_ = state_;
        const bool changed = state_ != next;
        state_ = next;
        s.state = recovery_manager_state_name(state_);
        s.previous_state = recovery_manager_state_name(previous_state_);
        s.reason = reason;
        s.missing_evidence = missing;
        s.rejected = rejected;
        s.waiting_for_recovery_scan = state_ == RecoveryManagerState::WAITING_FOR_RECOVERY_SCAN;
        if (state_ == RecoveryManagerState::RECOVERY_SCAN_RECOMMENDED) s.recovery_scan_recommended = true;
        if (state_ == RecoveryManagerState::CANDIDATE_READY) s.candidate_ready = true;
        latest_ = s;
        if (changed) {
            stats_.state_changes++;
            state_changed_since_log_ = true;
        }
        update_stats(s);
        return changed;
    }

    void update_stats(const RecoveryManagerSnapshot &s) {
        stats_.state_last = s.state;
        stats_.last_reason = s.reason;
        stats_.last_type = s.recovery_type;
        stats_.last_missing_evidence = s.missing_evidence;
        stats_.last_match_scan_id = s.latest_match_scan_id;
        stats_.last_yaw_candidate_deg = s.latest_yaw_candidate_deg;
        if (s.startup_recovery_observe) stats_.startup_observe_count++;
        if (s.degraded_recovery_observe) stats_.degraded_observe_count++;
        if (s.lost_recovery_observe) stats_.lost_observe_count++;
        if (s.recovery_scan_recommended) stats_.scan_recommended_count++;
        if (s.candidate_seen) stats_.candidate_seen_count++;
        if (s.candidate_ready) stats_.candidate_ready_count++;
        if (s.rejected) {
            stats_.rejected_count++;
            if (s.missing_evidence == "map") stats_.map_evidence_reject_count++;
            else if (s.missing_evidence == "tof") stats_.tof_evidence_reject_count++;
            else if (s.missing_evidence == "localizer") stats_.localizer_evidence_reject_count++;
            else if (s.missing_evidence == "yaw_match") stats_.yaw_match_evidence_reject_count++;
            else if (s.missing_evidence == "yaw_correction") stats_.yaw_correction_unstable_reject_count++;
        }
    }

    bool map_evidence_ok(const RecoveryManagerInput &in) const {
        if (in.map_quality.quality_level.empty()) return false;
        if (in.map_quality.quality_level == "BAD") return false;
        if (cfg_.recovery_require_map_quality_not_invalid && in.map_quality.quality_level == "INVALID") return false;
        if (in.map_known_cells < static_cast<uint64_t>(cfg_.recovery_require_min_known_cells)) return false;
        if (in.map_occupied_cells < static_cast<uint64_t>(cfg_.recovery_require_min_occupied_cells)) return false;
        return true;
    }

    bool tof_evidence_ok(const RecoveryManagerInput &in) const {
        if (!std::isfinite(in.latest_tof_age_s)) return false;
        if (cfg_.recovery_require_tof_recent && !in.tof_recent) return false;
        if (cfg_.recovery_require_tof_recent && in.latest_tof_age_s > cfg_.recovery_max_tof_age_s) return false;
        return true;
    }

    bool localizer_evidence_ok(const RecoveryManagerInput &in) const {
        if (cfg_.recovery_require_localizer_initialized_for_local_recovery && !in.localizer_initialized) return false;
        if (!std::isfinite(in.current_pose.x) || !std::isfinite(in.current_pose.y) || !std::isfinite(in.current_pose.yaw)) return false;
        if (!std::isfinite(in.linear_speed_mps) || !std::isfinite(in.yaw_rate_radps)) return false;
        return true;
    }

    bool yaw_match_evidence_ok(const RecoveryManagerInput &in) const {
        const auto &m = in.latest_yaw_match;
        if (!m.attempted || !m.usable || m.reason != "ok") return false;
        if (!in.latest_yaw_gate.scan_evidence_ok || !in.latest_yaw_gate.yaw_match_evidence_ok) return false;
        if (in.latest_yaw_gate.match_scan_id != m.scan_id) return false;
        if (!std::isfinite(m.timestamp_s)) return false;
        return true;
    }

    bool yaw_correction_stable(const RecoveryManagerInput &in) const {
        if (!cfg_.recovery_require_yaw_correction_stable) return true;
        if (cfg_.recovery_block_if_post_apply_failed_recent && in.latest_post_apply.state == "FAILED") return false;
        if (in.latest_yaw_apply.localizer_yaw_correction_apply_count > static_cast<uint64_t>(cfg_.recovery_max_recent_yaw_apply_count)) return false;
        if (in.latest_yaw_apply.applied && in.latest_post_apply.state != "VALIDATED") return false;
        if (post_apply_validated_observed_count_ < static_cast<uint64_t>(cfg_.recovery_min_post_apply_validated_count)) return false;
        return true;
    }

    const Config &cfg_;
    RecoveryManagerState state_ = RecoveryManagerState::IDLE;
    RecoveryManagerState previous_state_ = RecoveryManagerState::IDLE;
    RecoveryManagerSnapshot latest_;
    RecoveryManagerRunStats stats_;
    std::string supervisor_state_ = "INIT";
    double supervisor_state_since_s_ = 0.0;
    bool have_supervisor_state_ = false;
    double last_log_s_ = -1.0;
    bool state_changed_since_log_ = false;
    double last_recovery_scan_s_ = -1.0;
    double cooldown_until_s_ = -1.0;
    uint64_t post_apply_validated_observed_count_ = 0;
    double last_post_apply_validated_timestamp_s_ = -1.0;
};

inline void write_recovery_manager_header(std::ostream &o) {
    o << "timestamp_s,state,previous_state,reason,recovery_type,startup_recovery_observe,degraded_recovery_observe,lost_recovery_observe,recovery_scan_recommended,waiting_for_recovery_scan,candidate_seen,candidate_ready,rejected,missing_evidence,map_evidence_ok,tof_evidence_ok,localizer_evidence_ok,yaw_match_evidence_ok,yaw_correction_stable,latest_match_scan_id,latest_match_timestamp_s,latest_yaw_candidate_deg,latest_yaw_match_score,latest_yaw_match_margin,latest_yaw_match_inlier_ratio,supervisor_state,map_quality_level,time_in_lost_s,time_in_degraded_s,time_since_last_recovery_scan_s\n";
}

inline void write_recovery_manager_row(std::ostream &o, const RecoveryManagerSnapshot &s) {
    o << std::fixed << std::setprecision(6)
      << s.timestamp_s << ',' << s.state << ',' << s.previous_state << ',' << s.reason << ',' << s.recovery_type << ','
      << (s.startup_recovery_observe ? 1 : 0) << ',' << (s.degraded_recovery_observe ? 1 : 0) << ','
      << (s.lost_recovery_observe ? 1 : 0) << ',' << (s.recovery_scan_recommended ? 1 : 0) << ','
      << (s.waiting_for_recovery_scan ? 1 : 0) << ',' << (s.candidate_seen ? 1 : 0) << ','
      << (s.candidate_ready ? 1 : 0) << ',' << (s.rejected ? 1 : 0) << ',' << s.missing_evidence << ','
      << (s.map_evidence_ok ? 1 : 0) << ',' << (s.tof_evidence_ok ? 1 : 0) << ','
      << (s.localizer_evidence_ok ? 1 : 0) << ',' << (s.yaw_match_evidence_ok ? 1 : 0) << ','
      << (s.yaw_correction_stable ? 1 : 0) << ',' << s.latest_match_scan_id << ',' << s.latest_match_timestamp_s << ','
      << s.latest_yaw_candidate_deg << ',' << s.latest_yaw_match_score << ',' << s.latest_yaw_match_margin << ','
      << s.latest_yaw_match_inlier_ratio << ',' << s.supervisor_state << ',' << s.map_quality_level << ','
      << s.time_in_lost_s << ',' << s.time_in_degraded_s << ',' << s.time_since_last_recovery_scan_s << "\n";
}

} // namespace robot_slamd
