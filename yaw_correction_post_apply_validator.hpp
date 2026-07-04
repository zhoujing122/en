#pragma once
#include "common.hpp"
#include "mapping_supervisor.hpp"
#include "sparse_scan_yaw_matcher.hpp"
#include "yaw_correction_gate.hpp"

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
            latest_.timeout = true;
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
        double new_abs = std::fabs(m.best_yaw_delta_deg);
        double improvement = old_candidate_abs_deg_ - new_abs;
        if (m.curve_flat || m.multimodal) {
            state_ = YawCorrectionPostApplyState::FAILED;
            latest_ = make_snapshot(in, m.curve_flat ? "post_match_curve_flat" : "post_match_multimodal");
            latest_.failed = true;
            stats_.failed_count++;
        } else if (!m.usable) {
            state_ = YawCorrectionPostApplyState::SUSPECT;
            latest_ = make_snapshot(in, "post_match_not_usable");
            latest_.suspect = true;
            stats_.suspect_count++;
        } else if (improvement >= cfg_.yaw_correction_post_apply_min_improvement_deg) {
            state_ = YawCorrectionPostApplyState::VALIDATED;
            latest_ = make_snapshot(in, "validated");
            latest_.validated = true;
            stats_.validated_count++;
        } else if (-improvement > cfg_.yaw_correction_post_apply_max_allowed_worse_deg) {
            state_ = YawCorrectionPostApplyState::SUSPECT;
            latest_ = make_snapshot(in, "candidate_worse");
            latest_.suspect = true;
            stats_.suspect_count++;
        } else {
            state_ = YawCorrectionPostApplyState::SUSPECT;
            latest_ = make_snapshot(in, "insufficient_improvement");
            latest_.suspect = true;
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
        return s;
    }

    void update_stats(const YawCorrectionPostApplySnapshot &s) {
        stats_.last_state = s.state;
        stats_.last_reason = s.reason;
        stats_.last_improvement_deg = s.improvement_deg;
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
    o << "timestamp_s,state,reason,applied_scan_id,applied_delta_deg,old_candidate_deg,new_scan_id,new_candidate_deg,improvement_deg,validated,suspect,failed,timeout,supervisor_state\n";
}

inline void write_yaw_correction_post_apply_row(std::ostream &o, const YawCorrectionPostApplySnapshot &s) {
    o << std::fixed << std::setprecision(6)
      << s.timestamp_s << ',' << s.state << ',' << s.reason << ','
      << s.applied_scan_id << ',' << s.applied_delta_deg << ',' << s.old_candidate_deg << ','
      << s.new_scan_id << ',' << s.new_candidate_deg << ',' << s.improvement_deg << ','
      << (s.validated ? 1 : 0) << ',' << (s.suspect ? 1 : 0) << ','
      << (s.failed ? 1 : 0) << ',' << (s.timeout ? 1 : 0) << ',' << s.supervisor_state << "\n";
}

} // namespace robot_slamd
