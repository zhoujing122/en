#pragma once
#include "robot_slamd/core/common.hpp"
#include "robot_slamd/mapping/mapping_supervisor.hpp"
#include "robot_slamd/active_scan/sparse_scan_yaw_matcher.hpp"
#include "robot_slamd/active_scan/yaw_correction_gate.hpp"

namespace robot_slamd {

enum class YawCorrectionPostApplyState {
    DISABLED,
    IDLE,
    WAITING_FOR_NEXT_MATCH,
    VALIDATED,
    SUSPECT,
    FAILED,
    TIMEOUT,
};

inline const char *yaw_correction_post_apply_state_name(YawCorrectionPostApplyState s) {
    switch (s) {
    case YawCorrectionPostApplyState::DISABLED: return "DISABLED";
    case YawCorrectionPostApplyState::IDLE: return "IDLE";
    case YawCorrectionPostApplyState::WAITING_FOR_NEXT_MATCH: return "WAITING_FOR_NEXT_MATCH";
    case YawCorrectionPostApplyState::VALIDATED: return "VALIDATED";
    case YawCorrectionPostApplyState::SUSPECT: return "SUSPECT";
    case YawCorrectionPostApplyState::FAILED: return "FAILED";
    case YawCorrectionPostApplyState::TIMEOUT: return "TIMEOUT";
    }
    return "IDLE";
}

struct YawCorrectionPostApplyInput {
    double timestamp_s = 0.0;
    bool apply_happened = false;
    uint64_t applied_scan_id = 0;
    double applied_match_timestamp_s = 0.0;
    double applied_delta_deg = 0.0;
    SparseScanYawMatchSummary latest_yaw_match;
    YawCorrectionGateSnapshot latest_gate;
    MappingSupervisorSnapshot supervisor;
};

struct YawCorrectionPostApplySnapshot {
    double timestamp_s = 0.0;
    std::string state = "IDLE";
    std::string reason = "idle";
    uint64_t applied_scan_id = 0;
    double applied_delta_deg = 0.0;
    double old_candidate_deg = 0.0;
    uint64_t new_scan_id = 0;
    double new_candidate_deg = 0.0;
    double improvement_deg = 0.0;
    bool validated = false;
    bool suspect = false;
    bool failed = false;
    bool timeout = false;
    std::string supervisor_state = "INIT";
    double expected_min_improvement_deg = 0.0;
    double applied_match_timestamp_s = 0.0;
    double new_match_timestamp_s = 0.0;
};

class YawCorrectionPostApplyValidator {
public:
    explicit YawCorrectionPostApplyValidator(const Config &cfg) : cfg_(cfg) {
        if (!enabled()) {
            state_ = YawCorrectionPostApplyState::DISABLED;
            latest_.state = "DISABLED";
            latest_.reason = "disabled";
        }
    }

    bool update(const YawCorrectionPostApplyInput &in) {
        if (!enabled()) {
            bool changed = state_ != YawCorrectionPostApplyState::DISABLED;
            state_ = YawCorrectionPostApplyState::DISABLED;
            latest_ = make_snapshot(in, "disabled");
            return changed;
        }
        if (in.apply_happened) {
            waiting_since_s_ = in.timestamp_s;
            applied_scan_id_ = in.applied_scan_id;
            applied_match_timestamp_s_ = in.applied_match_timestamp_s;
            applied_delta_deg_ = in.applied_delta_deg;
            old_candidate_abs_deg_ = std::fabs(in.latest_gate.candidate_yaw_delta_deg);
            state_ = YawCorrectionPostApplyState::WAITING_FOR_NEXT_MATCH;
            latest_ = make_snapshot(in, "waiting_for_next_match");
            latest_.old_candidate_deg = old_candidate_abs_deg_;
            return true;
        }
        if (state_ != YawCorrectionPostApplyState::WAITING_FOR_NEXT_MATCH) {
            latest_ = make_snapshot(in, state_ == YawCorrectionPostApplyState::IDLE ? "idle" : latest_.reason);
            return false;
        }
        if (in.timestamp_s - waiting_since_s_ > cfg_.yaw_correction_post_apply_timeout_s) {
            state_ = YawCorrectionPostApplyState::TIMEOUT;
            latest_ = make_snapshot(in, "timeout");
            stats_.timeout_count++;
            update_stats(latest_);
            return true;
        }
        const auto &m = in.latest_yaw_match;
        if (!m.attempted) {
            latest_ = make_snapshot(in, "waiting_for_next_match");
            return false;
        }
        if (cfg_.yaw_correction_post_apply_require_new_scan_id && m.scan_id == applied_scan_id_) {
            latest_ = make_snapshot(in, "waiting_for_new_scan_id");
            return false;
        }
        if (cfg_.yaw_correction_post_apply_require_newer_match_timestamp && m.timestamp_s <= applied_match_timestamp_s_) {
            latest_ = make_snapshot(in, "waiting_for_newer_match_timestamp");
            return false;
        }

        const double new_abs = std::fabs(m.best_yaw_delta_deg);
        const double improvement = old_candidate_abs_deg_ - new_abs;
        if (m.curve_flat) {
            state_ = YawCorrectionPostApplyState::FAILED;
            latest_ = make_snapshot(in, "post_match_curve_flat");
            stats_.failed_count++;
        } else if (m.multimodal) {
            state_ = YawCorrectionPostApplyState::FAILED;
            latest_ = make_snapshot(in, "post_match_multimodal");
            stats_.failed_count++;
        } else if (!m.usable) {
            state_ = YawCorrectionPostApplyState::SUSPECT;
            latest_ = make_snapshot(in, "post_match_not_usable");
            stats_.suspect_count++;
        } else if (new_abs > cfg_.yaw_correction_post_apply_max_post_apply_candidate_abs_deg) {
            state_ = YawCorrectionPostApplyState::FAILED;
            latest_ = make_snapshot(in, "post_candidate_too_large");
            stats_.failed_count++;
        } else if (improvement >= expected_min_improvement_deg()) {
            state_ = YawCorrectionPostApplyState::VALIDATED;
            latest_ = make_snapshot(in, "validated");
            stats_.validated_count++;
        } else if (-improvement > cfg_.yaw_correction_post_apply_max_allowed_worse_deg) {
            state_ = YawCorrectionPostApplyState::SUSPECT;
            latest_ = make_snapshot(in, "candidate_worse");
            stats_.suspect_count++;
        } else {
            state_ = YawCorrectionPostApplyState::SUSPECT;
            latest_ = make_snapshot(in, "insufficient_improvement");
            stats_.suspect_count++;
        }
        latest_.old_candidate_deg = old_candidate_abs_deg_;
        latest_.new_scan_id = m.scan_id;
        latest_.new_candidate_deg = new_abs;
        latest_.improvement_deg = improvement;
        update_stats(latest_);
        return true;
    }

    YawCorrectionPostApplySnapshot snapshot() const { return latest_; }
    YawCorrectionPostApplyRunStats run_stats() const { return stats_; }

private:
    bool enabled() const { return cfg_.yaw_correction_post_apply_enabled; }

    double expected_min_improvement_deg() const {
        return std::max(cfg_.yaw_correction_post_apply_min_absolute_improvement_deg,
                        std::fabs(applied_delta_deg_) * cfg_.yaw_correction_post_apply_min_improvement_fraction_of_applied_delta);
    }

    YawCorrectionPostApplySnapshot make_snapshot(const YawCorrectionPostApplyInput &in, const std::string &reason) const {
        YawCorrectionPostApplySnapshot s;
        s.timestamp_s = in.timestamp_s;
        s.state = yaw_correction_post_apply_state_name(state_);
        s.reason = reason;
        s.applied_scan_id = applied_scan_id_ ? applied_scan_id_ : in.applied_scan_id;
        s.applied_delta_deg = applied_delta_deg_ != 0.0 ? applied_delta_deg_ : in.applied_delta_deg;
        s.old_candidate_deg = old_candidate_abs_deg_;
        s.new_scan_id = in.latest_yaw_match.scan_id;
        s.new_candidate_deg = std::fabs(in.latest_yaw_match.best_yaw_delta_deg);
        s.improvement_deg = old_candidate_abs_deg_ - s.new_candidate_deg;
        s.supervisor_state = in.supervisor.state;
        s.validated = state_ == YawCorrectionPostApplyState::VALIDATED;
        s.suspect = state_ == YawCorrectionPostApplyState::SUSPECT;
        s.failed = state_ == YawCorrectionPostApplyState::FAILED;
        s.timeout = state_ == YawCorrectionPostApplyState::TIMEOUT;
        s.expected_min_improvement_deg = expected_min_improvement_deg();
        s.applied_match_timestamp_s = applied_match_timestamp_s_ != 0.0 ? applied_match_timestamp_s_ : in.applied_match_timestamp_s;
        s.new_match_timestamp_s = in.latest_yaw_match.timestamp_s;
        return s;
    }

    void update_stats(const YawCorrectionPostApplySnapshot &s) {
        stats_.last_state = s.state;
        stats_.last_reason = s.reason;
        stats_.last_improvement_deg = s.improvement_deg;
        stats_.last_expected_min_improvement_deg = s.expected_min_improvement_deg;
        stats_.last_new_match_timestamp_s = s.new_match_timestamp_s;
    }

    const Config &cfg_;
    YawCorrectionPostApplyState state_ = YawCorrectionPostApplyState::IDLE;
    double waiting_since_s_ = -1.0;
    uint64_t applied_scan_id_ = 0;
    double applied_match_timestamp_s_ = 0.0;
    double applied_delta_deg_ = 0.0;
    double old_candidate_abs_deg_ = 0.0;
    YawCorrectionPostApplySnapshot latest_;
    YawCorrectionPostApplyRunStats stats_;
};

inline void write_yaw_correction_post_apply_header(std::ostream &o) {
    o << "timestamp_s,state,reason,applied_scan_id,applied_delta_deg,old_candidate_deg,new_scan_id,new_candidate_deg,improvement_deg,validated,suspect,failed,timeout,supervisor_state,expected_min_improvement_deg,applied_match_timestamp_s,new_match_timestamp_s\n";
}

inline void write_yaw_correction_post_apply_row(std::ostream &o, const YawCorrectionPostApplySnapshot &s) {
    o << std::fixed << std::setprecision(6)
      << s.timestamp_s << ',' << s.state << ',' << s.reason << ','
      << s.applied_scan_id << ',' << s.applied_delta_deg << ',' << s.old_candidate_deg << ','
      << s.new_scan_id << ',' << s.new_candidate_deg << ',' << s.improvement_deg << ','
      << (s.validated ? 1 : 0) << ',' << (s.suspect ? 1 : 0) << ','
      << (s.failed ? 1 : 0) << ',' << (s.timeout ? 1 : 0) << ',' << s.supervisor_state << ','
      << s.expected_min_improvement_deg << ',' << s.applied_match_timestamp_s << ','
      << s.new_match_timestamp_s << "\n";
}

} // namespace robot_slamd
