#pragma once
#include "active_scan_command_planner.hpp"
#include "active_scan_manager.hpp"
#include "common.hpp"
#include "map_quality.hpp"
#include "mapping_supervisor.hpp"
#include "sparse_scan_yaw_matcher.hpp"

namespace robot_slamd {

enum class YawCorrectionGateState {
    DISABLED,
    IDLE,
    CANDIDATE_SEEN,
    CONSISTENCY_CHECK,
    WOULD_APPLY,
    REJECTED,
    COOLDOWN,
};

inline const char *yaw_correction_gate_state_name(YawCorrectionGateState s) {
    switch (s) {
    case YawCorrectionGateState::DISABLED: return "DISABLED";
    case YawCorrectionGateState::IDLE: return "IDLE";
    case YawCorrectionGateState::CANDIDATE_SEEN: return "CANDIDATE_SEEN";
    case YawCorrectionGateState::CONSISTENCY_CHECK: return "CONSISTENCY_CHECK";
    case YawCorrectionGateState::WOULD_APPLY: return "WOULD_APPLY";
    case YawCorrectionGateState::REJECTED: return "REJECTED";
    case YawCorrectionGateState::COOLDOWN: return "COOLDOWN";
    }
    return "IDLE";
}

struct YawCorrectionGateInput {
    double timestamp_s = 0.0;
    SparseScanYawMatchSummary yaw_match;
    MapQualitySnapshot map_quality;
    MappingSupervisorSnapshot supervisor;
    ActiveScanSnapshot active_scan;
    ActiveScanCommandSnapshot active_scan_command;
    double current_yaw_rad = 0.0;
    double linear_speed_mps = 0.0;
    double yaw_rate_radps = 0.0;
    bool localization_valid = true;
};

struct YawCorrectionGateSnapshot {
    double timestamp_s = 0.0;
    std::string state = "IDLE";
    std::string previous_state = "IDLE";
    std::string reason = "idle";
    bool candidate_seen = false;
    bool would_apply = false;
    bool rejected = false;
    double candidate_yaw_delta_deg = 0.0;
    double suggested_correction_deg = 0.0;
    double suggested_new_yaw_rad = 0.0;
    double gain = 0.0;
    double max_step_deg = 0.0;
    double best_score = 0.0;
    double score_margin = 0.0;
    double inlier_ratio = 0.0;
    double curve_flatness = 0.0;
    bool multimodal = false;
    int consistency_count = 0;
    double consistency_spread_deg = 0.0;
    double linear_speed_mps = 0.0;
    double yaw_rate_dps = 0.0;
    std::string supervisor_state;
    std::string yaw_match_reason;
};

class YawCorrectionGate {
public:
    explicit YawCorrectionGate(const Config &cfg) : cfg_(cfg) {
        if (!enabled()) {
            state_ = YawCorrectionGateState::DISABLED;
            previous_state_ = YawCorrectionGateState::DISABLED;
            latest_.state = "DISABLED";
            latest_.previous_state = "DISABLED";
            latest_.reason = "disabled";
        }
    }

    bool update(const YawCorrectionGateInput &in) {
        double now_s = in.timestamp_s;
        bool changed = false;
        if (!enabled()) {
            changed = transition_to(YawCorrectionGateState::DISABLED, now_s);
            latest_ = make_base_snapshot(in, "disabled");
            return changed;
        }

        if (cooldown_until_s_ > now_s) {
            bool new_candidate = is_new_candidate(in.yaw_match);
            changed = transition_to(YawCorrectionGateState::COOLDOWN, now_s);
            latest_ = make_base_snapshot(in, "cooldown");
            latest_.rejected = new_candidate;
            if (new_candidate) {
                remember_candidate_id(in.yaw_match);
                stats_.candidates_seen++;
                stats_.rejected_count++;
                stats_.last_reason = "cooldown";
            }
            update_last_stats(latest_);
            return changed || new_candidate;
        }
        if (state_ == YawCorrectionGateState::COOLDOWN && cooldown_until_s_ <= now_s) {
            changed = transition_to(YawCorrectionGateState::IDLE, now_s) || changed;
        }

        if (!is_new_candidate(in.yaw_match)) {
            if (!in.yaw_match.attempted && state_ != YawCorrectionGateState::IDLE) {
                changed = transition_to(YawCorrectionGateState::IDLE, now_s) || changed;
            }
            latest_ = make_base_snapshot(in, state_ == YawCorrectionGateState::IDLE ? "idle" : latest_.reason);
            update_last_stats(latest_);
            return changed;
        }

        remember_candidate_id(in.yaw_match);
        stats_.candidates_seen++;
        latest_ = make_base_snapshot(in, "candidate_seen");
        latest_.candidate_seen = true;
        changed = transition_to(YawCorrectionGateState::CANDIDATE_SEEN, now_s) || changed;

        std::string reason = precheck_reject_reason(in);
        if (!reason.empty()) {
            reject(in, now_s, reason);
            return true;
        }

        add_consistency_candidate(in.yaw_match.best_yaw_delta_deg);
        latest_.consistency_count = static_cast<int>(candidate_window_.size());
        latest_.consistency_spread_deg = consistency_spread();

        if (static_cast<int>(candidate_window_.size()) < cfg_.yaw_correction_consistency_window) {
            latest_.reason = "consistency_not_enough";
            stats_.consistency_reject_count++;
            if (candidate_window_.size() <= 1) {
                changed = transition_to(YawCorrectionGateState::CANDIDATE_SEEN, now_s) || changed;
            } else {
                changed = transition_to(YawCorrectionGateState::CONSISTENCY_CHECK, now_s) || changed;
            }
            latest_.state = yaw_correction_gate_state_name(state_);
            latest_.previous_state = yaw_correction_gate_state_name(previous_state_);
            update_last_stats(latest_);
            return changed;
        }
        if (!consistency_sign_ok() || latest_.consistency_spread_deg > cfg_.yaw_correction_max_consistency_spread_deg) {
            stats_.consistency_reject_count++;
            reject(in, now_s, "consistency_spread_too_large");
            return true;
        }

        double suggested = clamp(in.yaw_match.best_yaw_delta_deg * cfg_.yaw_correction_gain,
                                 -cfg_.yaw_correction_max_step_deg,
                                  cfg_.yaw_correction_max_step_deg);
        if (std::fabs(suggested) < cfg_.yaw_correction_min_step_deg) {
            latest_.suggested_correction_deg = 0.0;
            latest_.suggested_new_yaw_rad = in.current_yaw_rad;
            reject(in, now_s, "candidate_too_small");
            return true;
        }

        latest_.state = "WOULD_APPLY";
        latest_.reason = "would_apply_dry_run";
        latest_.would_apply = true;
        latest_.rejected = false;
        latest_.suggested_correction_deg = suggested;
        latest_.suggested_new_yaw_rad = wrap_pi(in.current_yaw_rad + deg2rad(suggested));
        changed = transition_to(YawCorrectionGateState::WOULD_APPLY, now_s) || changed;
        latest_.state = yaw_correction_gate_state_name(state_);
        latest_.previous_state = yaw_correction_gate_state_name(previous_state_);
        stats_.would_apply_count++;
        cooldown_until_s_ = now_s + cfg_.yaw_correction_cooldown_s;
        update_last_stats(latest_);
        return true;
    }

    bool should_log(double now_s) const {
        if (!enabled()) return false;
        if (last_log_s_ < 0.0) return true;
        return now_s - last_log_s_ >= (1.0 / cfg_.yaw_correction_log_hz);
    }

    void mark_logged(double now_s, const YawCorrectionGateSnapshot &) { last_log_s_ = now_s; }
    YawCorrectionGateSnapshot snapshot() const { return latest_; }
    YawCorrectionGateRunStats run_stats(double) const {
        YawCorrectionGateRunStats out = stats_;
        out.state_last = yaw_correction_gate_state_name(state_);
        return out;
    }

private:
    bool enabled() const { return cfg_.yaw_correction_enabled && cfg_.yaw_correction_mode != "disabled"; }

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    bool transition_to(YawCorrectionGateState next, double now_s) {
        if (state_ == next) return false;
        previous_state_ = state_;
        state_ = next;
        state_enter_s_ = now_s;
        return true;
    }

    bool is_new_candidate(const SparseScanYawMatchSummary &s) const {
        if (!s.attempted) return false;
        return s.scan_id != last_scan_id_ || std::fabs(s.timestamp_s - last_match_timestamp_s_) > 1e-9;
    }

    void remember_candidate_id(const SparseScanYawMatchSummary &s) {
        last_scan_id_ = s.scan_id;
        last_match_timestamp_s_ = s.timestamp_s;
    }

    YawCorrectionGateSnapshot make_base_snapshot(const YawCorrectionGateInput &in, const std::string &reason) const {
        YawCorrectionGateSnapshot s;
        s.timestamp_s = in.timestamp_s;
        s.state = yaw_correction_gate_state_name(state_);
        s.previous_state = yaw_correction_gate_state_name(previous_state_);
        s.reason = reason;
        s.candidate_yaw_delta_deg = in.yaw_match.best_yaw_delta_deg;
        s.suggested_new_yaw_rad = in.current_yaw_rad;
        s.gain = cfg_.yaw_correction_gain;
        s.max_step_deg = cfg_.yaw_correction_max_step_deg;
        s.best_score = in.yaw_match.best_score;
        s.score_margin = in.yaw_match.score_margin;
        s.inlier_ratio = in.yaw_match.inlier_ratio;
        s.curve_flatness = in.yaw_match.curve_flatness;
        s.multimodal = in.yaw_match.multimodal;
        s.consistency_count = static_cast<int>(candidate_window_.size());
        s.consistency_spread_deg = consistency_spread();
        s.linear_speed_mps = in.linear_speed_mps;
        s.yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        s.supervisor_state = in.supervisor.state;
        s.yaw_match_reason = in.yaw_match.reason;
        return s;
    }

    std::string precheck_reject_reason(const YawCorrectionGateInput &in) const {
        const auto &m = in.yaw_match;
        if (cfg_.yaw_correction_require_yaw_match_usable && !m.usable) return "yaw_match_not_usable";
        if (m.reason != "ok") return "yaw_match_reason_not_ok";
        if (m.best_score < cfg_.yaw_correction_min_best_score) return "low_best_score";
        if (m.score_margin < cfg_.yaw_correction_min_score_margin) return "low_score_margin";
        if (m.inlier_ratio < cfg_.yaw_correction_min_inlier_ratio) return "low_inlier_ratio";
        if (m.curve_flat || m.curve_flatness > cfg_.yaw_correction_max_curve_flatness) return "curve_flat";
        if (m.multimodal) return "multimodal";
        double abs_candidate = std::fabs(m.best_yaw_delta_deg);
        if (abs_candidate > cfg_.yaw_correction_max_candidate_abs_deg) return "candidate_too_large";
        if (abs_candidate < cfg_.yaw_correction_min_candidate_abs_deg) return "candidate_too_small";
        if (cfg_.yaw_correction_require_low_speed_or_static && std::fabs(in.linear_speed_mps) > cfg_.yaw_correction_max_linear_speed_mps) return "robot_moving";
        if (cfg_.yaw_correction_require_low_speed_or_static && std::fabs(in.yaw_rate_radps) * 180.0 / kPi > cfg_.yaw_correction_max_abs_yaw_rate_dps) return "yaw_rate_too_high";
        if (!in.localization_valid) return "localization_invalid";
        if (cfg_.yaw_correction_require_mapping_state_ok && in.supervisor.state == "LOST") return "supervisor_lost";
        if (cfg_.yaw_correction_require_active_scan_complete && !active_scan_complete(in)) return "active_scan_not_complete";
        return "";
    }

    bool active_scan_complete(const YawCorrectionGateInput &in) const {
        return in.active_scan.completed ||
               in.active_scan.useful_scan_observed ||
               in.active_scan.state == "COMPLETED" ||
               in.active_scan_command.completed ||
               in.active_scan_command.state == "COMPLETED";
    }

    void add_consistency_candidate(double yaw_delta_deg) {
        candidate_window_.push_back(yaw_delta_deg);
        while (static_cast<int>(candidate_window_.size()) > cfg_.yaw_correction_consistency_window) candidate_window_.pop_front();
    }

    double consistency_spread() const {
        if (candidate_window_.empty()) return 0.0;
        auto mm = std::minmax_element(candidate_window_.begin(), candidate_window_.end());
        return *mm.second - *mm.first;
    }

    bool consistency_sign_ok() const {
        if (candidate_window_.empty()) return false;
        bool pos = candidate_window_.front() >= 0.0;
        for (double v : candidate_window_) {
            if ((v >= 0.0) != pos) return false;
        }
        return true;
    }

    void reject(const YawCorrectionGateInput &in, double now_s, const std::string &reason) {
        latest_ = make_base_snapshot(in, reason);
        latest_.candidate_seen = true;
        latest_.rejected = true;
        latest_.would_apply = false;
        latest_.state = "REJECTED";
        latest_.suggested_new_yaw_rad = in.current_yaw_rad;
        transition_to(YawCorrectionGateState::REJECTED, now_s);
        latest_.state = yaw_correction_gate_state_name(state_);
        latest_.previous_state = yaw_correction_gate_state_name(previous_state_);
        stats_.rejected_count++;
        if (reason == "robot_moving" || reason == "yaw_rate_too_high") stats_.robot_moving_reject_count++;
        if (reason == "yaw_match_not_usable" || reason == "yaw_match_reason_not_ok" || reason == "low_best_score" ||
            reason == "low_score_margin" || reason == "low_inlier_ratio" || reason == "curve_flat" || reason == "multimodal") {
            stats_.low_quality_reject_count++;
        }
        cooldown_until_s_ = now_s + cfg_.yaw_correction_cooldown_s;
        update_last_stats(latest_);
    }

    void update_last_stats(const YawCorrectionGateSnapshot &s) {
        stats_.last_candidate_deg = s.candidate_yaw_delta_deg;
        stats_.last_suggested_correction_deg = s.suggested_correction_deg;
        stats_.last_reason = s.reason;
        stats_.consistency_count_last = s.consistency_count;
        stats_.consistency_spread_deg_last = s.consistency_spread_deg;
    }

    const Config &cfg_;
    YawCorrectionGateState state_ = YawCorrectionGateState::IDLE;
    YawCorrectionGateState previous_state_ = YawCorrectionGateState::IDLE;
    double state_enter_s_ = 0.0;
    double last_log_s_ = -1.0;
    double cooldown_until_s_ = -1.0;
    uint64_t last_scan_id_ = 0;
    double last_match_timestamp_s_ = -1.0;
    std::deque<double> candidate_window_;
    YawCorrectionGateSnapshot latest_;
    YawCorrectionGateRunStats stats_;
};

inline void write_yaw_correction_gate_header(std::ofstream &o) {
    o << "timestamp_s,state,previous_state,reason,candidate_seen,would_apply,rejected,candidate_yaw_delta_deg,suggested_correction_deg,suggested_new_yaw_rad,best_score,score_margin,inlier_ratio,curve_flatness,multimodal,consistency_count,consistency_spread_deg,linear_speed_mps,yaw_rate_dps,supervisor_state,yaw_match_reason\n";
}

inline void write_yaw_correction_gate_row(std::ofstream &o, const YawCorrectionGateSnapshot &s) {
    o << s.timestamp_s << "," << s.state << "," << s.previous_state << "," << s.reason << ","
      << (s.candidate_seen ? 1 : 0) << "," << (s.would_apply ? 1 : 0) << "," << (s.rejected ? 1 : 0) << ","
      << s.candidate_yaw_delta_deg << "," << s.suggested_correction_deg << "," << s.suggested_new_yaw_rad << ","
      << s.best_score << "," << s.score_margin << "," << s.inlier_ratio << "," << s.curve_flatness << ","
      << (s.multimodal ? 1 : 0) << "," << s.consistency_count << "," << s.consistency_spread_deg << ","
      << s.linear_speed_mps << "," << s.yaw_rate_dps << "," << s.supervisor_state << "," << s.yaw_match_reason << "\n";
}

} // namespace robot_slamd
