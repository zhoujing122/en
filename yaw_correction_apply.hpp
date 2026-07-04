#pragma once
#include "common.hpp"
#include "localizer.hpp"
#include "mapping_supervisor.hpp"
#include "yaw_correction_gate.hpp"

namespace robot_slamd {

enum class YawCorrectionApplyState {
    DISABLED,
    IDLE,
    READY_TO_APPLY,
    APPLIED,
    REJECTED,
    COOLDOWN,
    FAULT,
};

inline const char *yaw_correction_apply_state_name(YawCorrectionApplyState s) {
    switch (s) {
    case YawCorrectionApplyState::DISABLED: return "DISABLED";
    case YawCorrectionApplyState::IDLE: return "IDLE";
    case YawCorrectionApplyState::READY_TO_APPLY: return "READY_TO_APPLY";
    case YawCorrectionApplyState::APPLIED: return "APPLIED";
    case YawCorrectionApplyState::REJECTED: return "REJECTED";
    case YawCorrectionApplyState::COOLDOWN: return "COOLDOWN";
    case YawCorrectionApplyState::FAULT: return "FAULT";
    }
    return "IDLE";
}

struct YawCorrectionApplySnapshot {
    double timestamp_s = 0.0;
    std::string state = "IDLE";
    std::string reason = "idle";
    bool attempted = false;
    bool applied = false;
    double old_yaw_rad = 0.0;
    double delta_yaw_deg = 0.0;
    double new_yaw_rad = 0.0;
    bool gate_would_apply = false;
    std::string gate_reason;
    double gate_candidate_yaw_delta_deg = 0.0;
    double gate_suggested_correction_deg = 0.0;
    double gate_best_score = 0.0;
    double gate_score_margin = 0.0;
    double gate_inlier_ratio = 0.0;
    bool gate_scan_evidence_ok = false;
    bool gate_yaw_match_evidence_ok = false;
    double linear_speed_mps = 0.0;
    double yaw_rate_dps = 0.0;
    std::string supervisor_state;
    uint64_t session_apply_count = 0;
    double session_total_abs_correction_deg = 0.0;
    double time_since_last_apply_s = -1.0;
    double old_yaw_correction_offset_rad = 0.0;
    double new_yaw_correction_offset_rad = 0.0;
    uint64_t localizer_yaw_correction_apply_count = 0;
    double localizer_yaw_correction_total_abs_deg = 0.0;
    uint64_t gate_match_scan_id = 0;
    double gate_match_timestamp_s = 0.0;
    bool duplicate_candidate_rejected = false;
};

inline bool compute_localization_valid_for_yaw_writeback(const Localizer &loc,
                                                         const MappingSupervisorSnapshot &supervisor,
                                                         const MapQualitySnapshot &map_quality,
                                                         const YawCorrectionGateSnapshot &gate) {
    if (!loc.initialized()) return false;
    if (supervisor.state == "LOST" || supervisor.state == "DEGRADED") return false;
    if (map_quality.quality_level == "BAD" || map_quality.quality_level == "INVALID") return false;
    if (!gate.scan_evidence_ok || !gate.yaw_match_evidence_ok) return false;
    const auto &p = loc.pose();
    return std::isfinite(p.yaw) && std::isfinite(loc.v()) && std::isfinite(loc.w());
}

class YawCorrectionApply {
public:
    explicit YawCorrectionApply(const Config &cfg) : cfg_(cfg) {}

    YawCorrectionApplySnapshot update(double timestamp_s,
                                      const YawCorrectionGateSnapshot &gate,
                                      Localizer &localizer,
                                      double current_yaw_rad,
                                      double linear_speed_mps,
                                      double yaw_rate_radps,
                                      const MappingSupervisorSnapshot &supervisor,
                                      bool localization_valid = true) {
        YawCorrectionApplySnapshot s = make_snapshot(timestamp_s, gate, current_yaw_rad,
                                                     linear_speed_mps, yaw_rate_radps, supervisor);
        s.old_yaw_correction_offset_rad = localizer.yaw_correction_offset_rad();
        s.new_yaw_correction_offset_rad = s.old_yaw_correction_offset_rad;
        s.localizer_yaw_correction_apply_count = localizer.yaw_correction_apply_count();
        s.localizer_yaw_correction_total_abs_deg = localizer.yaw_correction_total_abs_deg();
        if (!cfg_.yaw_correction_enabled || cfg_.yaw_correction_mode == "disabled") {
            state_ = YawCorrectionApplyState::DISABLED;
            s.state = yaw_correction_apply_state_name(state_);
            s.reason = "disabled";
            latest_ = s;
            return latest_;
        }
        if (cfg_.yaw_correction_mode != "writeback") {
            state_ = YawCorrectionApplyState::IDLE;
            s.state = yaw_correction_apply_state_name(state_);
            s.reason = "not_writeback_mode";
            latest_ = s;
            return latest_;
        }
        if (!cfg_.yaw_correction_writeback_enabled) {
            state_ = YawCorrectionApplyState::FAULT;
            s.state = yaw_correction_apply_state_name(state_);
            s.reason = "writeback_not_enabled";
            latest_ = s;
            return latest_;
        }
        if (cfg_.yaw_correction_writeback_acknowledgement != cfg_.yaw_correction_required_writeback_acknowledgement) {
            state_ = YawCorrectionApplyState::FAULT;
            s.state = yaw_correction_apply_state_name(state_);
            s.reason = "acknowledgement_mismatch";
            latest_ = s;
            return latest_;
        }

        const bool gate_candidate = gate.candidate_seen || gate.would_apply || gate.rejected;
        if (!gate_candidate) {
            state_ = YawCorrectionApplyState::IDLE;
            s.state = yaw_correction_apply_state_name(state_);
            s.reason = "idle";
            latest_ = s;
            return latest_;
        }

        s.attempted = true;
        stats_.attempts++;
        state_ = YawCorrectionApplyState::READY_TO_APPLY;
        s.state = yaw_correction_apply_state_name(state_);

        const double since_last = last_apply_s_ >= 0.0 ? timestamp_s - last_apply_s_ : -1.0;
        s.time_since_last_apply_s = since_last;
        const bool has_candidate_id = gate.match_scan_id != 0 || gate.match_timestamp_s > 0.0;
        if (has_candidate_id && gate.match_scan_id == last_applied_scan_id_ &&
            std::fabs(gate.match_timestamp_s - last_applied_match_timestamp_s_) < 1e-9) {
            s.duplicate_candidate_rejected = true;
            reject(s, "duplicate_candidate", RejectBucket::Safety);
            stats_.duplicate_candidate_reject_count++;
            latest_ = s;
            return latest_;
        }
        if (since_last >= 0.0 && since_last < cfg_.yaw_correction_min_seconds_between_writebacks) {
            reject(s, "cooldown", RejectBucket::Cooldown);
            state_ = YawCorrectionApplyState::COOLDOWN;
            s.state = yaw_correction_apply_state_name(state_);
            latest_ = s;
            return latest_;
        }
        if (gate.rejected) {
            reject(s, "gate_rejected", RejectBucket::Gate);
            latest_ = s;
            return latest_;
        }
        if (cfg_.yaw_correction_require_gate_would_apply && !gate.would_apply) {
            reject(s, "gate_not_would_apply", RejectBucket::Gate);
            latest_ = s;
            return latest_;
        }
        if (!cfg_.yaw_correction_require_gate_reason.empty() && gate.reason != cfg_.yaw_correction_require_gate_reason) {
            reject(s, "gate_not_would_apply", RejectBucket::Gate);
            latest_ = s;
            return latest_;
        }
        if (cfg_.yaw_correction_require_scan_evidence_ok && !gate.scan_evidence_ok) {
            reject(s, "gate_scan_evidence_not_ok", RejectBucket::Gate);
            latest_ = s;
            return latest_;
        }
        if (cfg_.yaw_correction_require_yaw_match_evidence_ok && !gate.yaw_match_evidence_ok) {
            reject(s, "gate_yaw_match_evidence_not_ok", RejectBucket::Gate);
            latest_ = s;
            return latest_;
        }
        if (!std::isfinite(gate.suggested_correction_deg)) {
            reject(s, "candidate_not_finite", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }
        const double abs_delta = std::fabs(gate.suggested_correction_deg);
        if (abs_delta > cfg_.yaw_correction_max_writeback_abs_deg) {
            reject(s, "correction_too_large", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }
        if (supervisor.state == "LOST") {
            reject(s, "supervisor_lost", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }
        if (!localization_valid) {
            reject(s, "localization_invalid", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }
        if (std::fabs(linear_speed_mps) > cfg_.yaw_correction_max_linear_speed_mps) {
            reject(s, "robot_moving", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }
        if (std::fabs(s.yaw_rate_dps) > cfg_.yaw_correction_max_abs_yaw_rate_dps) {
            reject(s, "yaw_rate_too_high", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }
        if (session_apply_count_ >= static_cast<uint64_t>(cfg_.yaw_correction_max_writeback_count_per_session)) {
            reject(s, "session_writeback_count_limit", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }
        if (session_total_abs_correction_deg_ + abs_delta > cfg_.yaw_correction_max_total_writeback_per_session_deg) {
            reject(s, "session_writeback_angle_limit", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }

        s.old_yaw_rad = localizer.pose().yaw;
        s.delta_yaw_deg = gate.suggested_correction_deg;
        const bool applied = localizer.apply_yaw_correction_only(deg2rad(gate.suggested_correction_deg), "yaw_correction_gate");
        s.new_yaw_rad = localizer.pose().yaw;
        s.new_yaw_correction_offset_rad = localizer.yaw_correction_offset_rad();
        s.localizer_yaw_correction_apply_count = localizer.yaw_correction_apply_count();
        s.localizer_yaw_correction_total_abs_deg = localizer.yaw_correction_total_abs_deg();
        if (!applied) {
            reject(s, "localizer_rejected", RejectBucket::Safety);
            latest_ = s;
            return latest_;
        }

        state_ = YawCorrectionApplyState::APPLIED;
        s.state = yaw_correction_apply_state_name(state_);
        s.reason = "applied";
        s.applied = true;
        last_apply_s_ = timestamp_s;
        session_apply_count_++;
        session_total_abs_correction_deg_ += abs_delta;
        s.session_apply_count = session_apply_count_;
        s.session_total_abs_correction_deg = session_total_abs_correction_deg_;

        stats_.success_count++;
        stats_.last_applied = true;
        stats_.last_reason = s.reason;
        stats_.last_delta_deg = s.delta_yaw_deg;
        stats_.last_old_yaw_rad = s.old_yaw_rad;
        stats_.last_new_yaw_rad = s.new_yaw_rad;
        stats_.session_count = session_apply_count_;
        stats_.session_total_abs_deg = session_total_abs_correction_deg_;
        last_applied_scan_id_ = gate.match_scan_id;
        last_applied_match_timestamp_s_ = gate.match_timestamp_s;
        stats_.last_match_scan_id = gate.match_scan_id;
        stats_.last_match_timestamp_s = gate.match_timestamp_s;
        latest_ = s;
        return latest_;
    }

    YawCorrectionApplySnapshot latest_snapshot() const { return latest_; }
    YawCorrectionApplyRunStats run_stats() const { return stats_; }

private:
    enum class RejectBucket { Gate, Safety, Cooldown };

    YawCorrectionApplySnapshot make_snapshot(double timestamp_s,
                                             const YawCorrectionGateSnapshot &gate,
                                             double current_yaw_rad,
                                             double linear_speed_mps,
                                             double yaw_rate_radps,
                                             const MappingSupervisorSnapshot &supervisor) const {
        YawCorrectionApplySnapshot s;
        s.timestamp_s = timestamp_s;
        s.old_yaw_rad = current_yaw_rad;
        s.new_yaw_rad = current_yaw_rad;
        s.gate_would_apply = gate.would_apply;
        s.gate_reason = gate.reason;
        s.gate_candidate_yaw_delta_deg = gate.candidate_yaw_delta_deg;
        s.gate_suggested_correction_deg = gate.suggested_correction_deg;
        s.gate_best_score = gate.best_score;
        s.gate_score_margin = gate.score_margin;
        s.gate_inlier_ratio = gate.inlier_ratio;
        s.gate_scan_evidence_ok = gate.scan_evidence_ok;
        s.gate_yaw_match_evidence_ok = gate.yaw_match_evidence_ok;
        s.gate_match_scan_id = gate.match_scan_id;
        s.gate_match_timestamp_s = gate.match_timestamp_s;
        s.linear_speed_mps = linear_speed_mps;
        s.yaw_rate_dps = yaw_rate_radps * 180.0 / kPi;
        s.supervisor_state = supervisor.state;
        s.session_apply_count = session_apply_count_;
        s.session_total_abs_correction_deg = session_total_abs_correction_deg_;
        s.time_since_last_apply_s = last_apply_s_ >= 0.0 ? timestamp_s - last_apply_s_ : -1.0;
        return s;
    }

    void reject(YawCorrectionApplySnapshot &s, const std::string &reason, RejectBucket bucket) {
        state_ = bucket == RejectBucket::Cooldown ? YawCorrectionApplyState::COOLDOWN : YawCorrectionApplyState::REJECTED;
        s.state = yaw_correction_apply_state_name(state_);
        s.reason = reason;
        s.applied = false;
        stats_.rejected_count++;
        stats_.last_applied = false;
        stats_.last_reason = reason;
        stats_.last_delta_deg = s.delta_yaw_deg;
        stats_.last_old_yaw_rad = s.old_yaw_rad;
        stats_.last_new_yaw_rad = s.new_yaw_rad;
        stats_.session_count = session_apply_count_;
        stats_.session_total_abs_deg = session_total_abs_correction_deg_;
        stats_.last_match_scan_id = s.gate_match_scan_id;
        stats_.last_match_timestamp_s = s.gate_match_timestamp_s;
        if (bucket == RejectBucket::Cooldown) stats_.cooldown_reject_count++;
        if (bucket == RejectBucket::Gate) stats_.gate_reject_count++;
        if (bucket == RejectBucket::Safety) stats_.safety_reject_count++;
    }

    const Config &cfg_;
    YawCorrectionApplyState state_ = YawCorrectionApplyState::IDLE;
    double last_apply_s_ = -1.0;
    uint64_t session_apply_count_ = 0;
    double session_total_abs_correction_deg_ = 0.0;
    uint64_t last_applied_scan_id_ = 0;
    double last_applied_match_timestamp_s_ = -1.0;
    YawCorrectionApplySnapshot latest_;
    YawCorrectionApplyRunStats stats_;
};

inline void write_yaw_correction_apply_header(std::ostream &o) {
    o << "timestamp_s,state,reason,attempted,applied,old_yaw_rad,delta_yaw_deg,new_yaw_rad,gate_would_apply,gate_reason,gate_candidate_yaw_delta_deg,gate_suggested_correction_deg,gate_best_score,gate_score_margin,gate_inlier_ratio,gate_scan_evidence_ok,gate_yaw_match_evidence_ok,linear_speed_mps,yaw_rate_dps,supervisor_state,session_apply_count,session_total_abs_correction_deg,time_since_last_apply_s,old_yaw_correction_offset_rad,new_yaw_correction_offset_rad,localizer_yaw_correction_apply_count,localizer_yaw_correction_total_abs_deg,gate_match_scan_id,gate_match_timestamp_s,duplicate_candidate_rejected\n";
}

inline void write_yaw_correction_apply_row(std::ostream &o, const YawCorrectionApplySnapshot &s) {
    o << std::fixed << std::setprecision(6)
      << s.timestamp_s << ',' << s.state << ',' << s.reason << ','
      << (s.attempted ? 1 : 0) << ',' << (s.applied ? 1 : 0) << ','
      << s.old_yaw_rad << ',' << s.delta_yaw_deg << ',' << s.new_yaw_rad << ','
      << (s.gate_would_apply ? 1 : 0) << ',' << s.gate_reason << ','
      << s.gate_candidate_yaw_delta_deg << ',' << s.gate_suggested_correction_deg << ','
      << s.gate_best_score << ',' << s.gate_score_margin << ',' << s.gate_inlier_ratio << ','
      << (s.gate_scan_evidence_ok ? 1 : 0) << ',' << (s.gate_yaw_match_evidence_ok ? 1 : 0) << ','
      << s.linear_speed_mps << ',' << s.yaw_rate_dps << ',' << s.supervisor_state << ','
      << s.session_apply_count << ',' << s.session_total_abs_correction_deg << ','
      << s.time_since_last_apply_s << ',' << s.old_yaw_correction_offset_rad << ','
      << s.new_yaw_correction_offset_rad << ',' << s.localizer_yaw_correction_apply_count << ','
      << s.localizer_yaw_correction_total_abs_deg << ',' << s.gate_match_scan_id << ','
      << s.gate_match_timestamp_s << ',' << (s.duplicate_candidate_rejected ? 1 : 0) << "\n";
}

} // namespace robot_slamd
