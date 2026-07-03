#pragma once
#include "active_scan_command_planner.hpp"
#include "active_scan_manager.hpp"
#include "common.hpp"
#include "map_quality.hpp"
#include "mapping_supervisor.hpp"
#include "spin_scan_localization.hpp"

namespace robot_slamd {

enum class SparseScanState {
    DISABLED,
    IDLE,
    COLLECTING,
    FINALIZING,
    COMPLETED,
    ABORTED,
    COOLDOWN,
};

inline const char *sparse_scan_state_name(SparseScanState s) {
    switch (s) {
    case SparseScanState::DISABLED: return "DISABLED";
    case SparseScanState::IDLE: return "IDLE";
    case SparseScanState::COLLECTING: return "COLLECTING";
    case SparseScanState::FINALIZING: return "FINALIZING";
    case SparseScanState::COMPLETED: return "COMPLETED";
    case SparseScanState::ABORTED: return "ABORTED";
    case SparseScanState::COOLDOWN: return "COOLDOWN";
    }
    return "IDLE";
}

struct SparseScanInput {
    double timestamp_s = 0.0;
    uint64_t timestamp_us = 0;
    Pose pose;
    double linear_speed_mps = 0.0;
    double yaw_rate_radps = 0.0;
    bool localization_valid = true;
    MapQualitySnapshot map_quality;
    MappingSupervisorSnapshot supervisor;
    ActiveScanSnapshot active_scan;
    ActiveScanCommandSnapshot active_scan_command;
    std::vector<FilteredTofSample> tof_samples;
};

struct SparseScanSample {
    uint64_t scan_id = 0;
    uint64_t timestamp_us = 0;
    double timestamp_s = 0.0;
    std::string sensor_id;
    double base_x_m = 0.0;
    double base_y_m = 0.0;
    double base_yaw_rad = 0.0;
    double sensor_x_m = 0.0;
    double sensor_y_m = 0.0;
    double sensor_yaw_rad = 0.0;
    double global_bearing_rad = 0.0;
    double global_bearing_deg = 0.0;
    double range_m = 0.0;
    int confidence = 0;
    int range_status = 0;
    bool valid = false;
    bool hit = false;
    double hit_x_m = 0.0;
    double hit_y_m = 0.0;
    double observed_yaw_delta_deg = 0.0;
    std::string reason;
};

struct SparseScanBin {
    uint64_t scan_id = 0;
    int bin_index = 0;
    double bin_center_deg = 0.0;
    double selected_range_m = 0.0;
    int selected_confidence = 0;
    int sample_count = 0;
    int valid_count = 0;
    int sensor_mask = 0;
    bool valid = false;
};

struct SparseScanSessionSummary {
    uint64_t scan_id = 0;
    double start_s = 0.0;
    double end_s = 0.0;
    double duration_s = 0.0;
    std::string state;
    std::string reason;
    int total_samples = 0;
    int valid_samples = 0;
    int invalid_samples = 0;
    int front_valid = 0;
    int left_valid = 0;
    int right_valid = 0;
    double observed_yaw_delta_deg = 0.0;
    double valid_bin_ratio = 0.0;
    int total_bins = 0;
    int valid_bins = 0;
    bool useful = false;
    bool complete = false;
    double quality_score = 0.0;
    std::string active_scan_state;
    std::string active_scan_command_state;
    std::string supervisor_state;
};

struct SparseScanForMatching {
    uint64_t scan_id = 0;
    SparseScanSessionSummary summary;
    std::vector<SparseScanSample> samples;
    std::vector<SparseScanBin> bins;
};

class SparseScanCollector {
public:
    explicit SparseScanCollector(const Config &cfg) : cfg_(cfg) {
        reset_bins();
        if (!cfg_.sparse_scan_enabled) {
            state_ = SparseScanState::DISABLED;
            previous_state_ = SparseScanState::DISABLED;
        }
    }

    bool update(const SparseScanInput &input) {
        double now_s = input.timestamp_s;
        if (last_update_s_ < 0.0) {
            last_update_s_ = now_s;
            state_enter_s_ = now_s;
        } else {
            last_update_s_ = now_s;
        }

        if (!cfg_.sparse_scan_enabled) {
            bool changed = state_ != SparseScanState::DISABLED;
            if (changed) transition_to(SparseScanState::DISABLED, now_s);
            return changed;
        }

        bool changed = false;
        bool trigger = collection_trigger(input);
        bool gates = gates_ok(input);
        std::string gate_reason = blocked_reason(input);

        if (state_ == SparseScanState::COMPLETED || state_ == SparseScanState::ABORTED) {
            transition_to(SparseScanState::COOLDOWN, now_s);
            return true;
        }
        if (state_ == SparseScanState::COOLDOWN) {
            if (now_s >= cooldown_until_s_) {
                transition_to(SparseScanState::IDLE, now_s);
                reset_session_runtime(input.pose.yaw);
                return true;
            }
            return false;
        }

        if (state_ == SparseScanState::IDLE) {
            if (trigger && gates) {
                start_session(input);
                changed = true;
                collect_samples(input, "collecting");
                maybe_finalize_after_collect(input, changed);
            }
            return changed;
        }

        if (state_ != SparseScanState::COLLECTING) return changed;

        accumulate_yaw(input.pose.yaw);
        double session_duration_s = now_s - session_start_s_;
        if (!input.localization_valid) {
            finalize_session(input, false, "localization_invalid");
            return true;
        }
        if (std::fabs(input.linear_speed_mps) > cfg_.sparse_scan_max_linear_speed_mps) {
            finalize_session(input, false, "linear_speed_too_high");
            return true;
        }
        double yaw_rate_dps = std::fabs(input.yaw_rate_radps) * 180.0 / kPi;
        if (yaw_rate_dps > cfg_.sparse_scan_max_yaw_rate_dps) {
            finalize_session(input, false, "yaw_rate_too_high");
            return true;
        }
        if (session_duration_s > cfg_.sparse_scan_session_timeout_s) {
            finalize_session(input, false, "session_timeout");
            return true;
        }
        if (last_valid_sample_s_ >= 0.0 && now_s - last_valid_sample_s_ > cfg_.sparse_scan_max_gap_s) {
            finalize_session(input, false, "max_gap_timeout");
            return true;
        }
        if (!trigger || !gates) {
            if (observed_yaw_delta_deg_ >= cfg_.sparse_scan_useful_session_angle_deg) {
                finalize_session(input, true, "request_cleared_useful");
            } else {
                finalize_session(input, false, trigger ? gate_reason : "request_cleared");
            }
            return true;
        }

        collect_samples(input, "collecting");
        maybe_finalize_after_collect(input, changed);
        return changed;
    }

    bool should_log(double now_s) const {
        if (!cfg_.sparse_scan_enabled) return false;
        if (last_log_s_ < 0.0) return true;
        return now_s - last_log_s_ >= (1.0 / cfg_.sparse_scan_log_hz);
    }

    void mark_logged(double now_s) { last_log_s_ = now_s; }

    SparseScanState state() const { return state_; }
    SparseScanSessionSummary latest_summary() const { return latest_summary_; }

    std::vector<SparseScanSample> drain_samples() {
        std::vector<SparseScanSample> out;
        out.swap(pending_samples_);
        return out;
    }

    std::vector<SparseScanBin> drain_bins() {
        std::vector<SparseScanBin> out;
        out.swap(pending_bins_);
        return out;
    }

    std::vector<SparseScanSessionSummary> drain_summaries() {
        std::vector<SparseScanSessionSummary> out;
        out.swap(pending_summaries_);
        return out;
    }

    std::vector<SparseScanForMatching> drain_completed_sessions_for_matching() {
        std::vector<SparseScanForMatching> out;
        out.swap(pending_match_scans_);
        return out;
    }

    SparseScanRunStats run_stats(double) const {
        SparseScanRunStats out = stats_;
        out.state_last = sparse_scan_state_name(state_);
        return out;
    }

private:
    static double rad2deg(double r) { return r * 180.0 / kPi; }

    static double norm_deg_360(double d) {
        while (d < 0.0) d += 360.0;
        while (d >= 360.0) d -= 360.0;
        return d;
    }

    static int sensor_bit(const std::string &id) {
        if (id == "front") return 1;
        if (id == "left") return 2;
        if (id == "right") return 4;
        return 0;
    }

    bool valid_hit(const FilteredTofSample &t) const {
        return t.map_update && t.hit && t.confidence >= cfg_.sparse_scan_min_confidence &&
               t.range_m >= cfg_.sparse_scan_min_range_m && t.range_m <= cfg_.sparse_scan_max_range_m;
    }

    std::string invalid_reason(const FilteredTofSample &t) const {
        if (!t.map_update) return t.decision.empty() ? "filtered_rejected" : t.decision;
        if (!t.hit) return "not_hit";
        if (t.confidence < cfg_.sparse_scan_min_confidence) return "low_confidence";
        if (t.range_m < cfg_.sparse_scan_min_range_m || t.range_m > cfg_.sparse_scan_max_range_m) return "range_out_of_bounds";
        return "invalid";
    }

    bool collection_trigger(const SparseScanInput &in) const {
        bool active = cfg_.sparse_scan_collect_on_active_scan &&
                      (in.active_scan.state == "OBSERVING_ROTATION" ||
                       in.active_scan.state == "WOULD_SCAN" ||
                       in.active_scan.scan_recommended);
        bool command = cfg_.sparse_scan_collect_on_command_verifying &&
                       (in.active_scan_command.state == "VERIFYING_ROTATION" ||
                        in.active_scan_command.command_active);
        double yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        bool natural = cfg_.sparse_scan_collect_on_natural_rotation &&
                       yaw_rate_dps >= cfg_.sparse_scan_min_yaw_rate_dps &&
                       yaw_rate_dps <= cfg_.sparse_scan_max_yaw_rate_dps;
        return active || command || natural;
    }

    bool gates_ok(const SparseScanInput &in) const {
        double yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        if (!in.localization_valid) return false;
        if (std::fabs(in.linear_speed_mps) > cfg_.sparse_scan_max_linear_speed_mps) return false;
        if (yaw_rate_dps < cfg_.sparse_scan_min_yaw_rate_dps) return false;
        if (yaw_rate_dps > cfg_.sparse_scan_max_yaw_rate_dps) return false;
        if (cfg_.sparse_scan_require_active_scan_recommended && !in.active_scan.scan_recommended) return false;
        if (cfg_.sparse_scan_require_mapping_allowed && !in.supervisor.mapping_allowed) return false;
        if (state_ == SparseScanState::IDLE && cooldown_until_s_ >= 0.0 && in.timestamp_s < cooldown_until_s_) return false;
        return true;
    }

    std::string blocked_reason(const SparseScanInput &in) const {
        double yaw_rate_dps = std::fabs(in.yaw_rate_radps) * 180.0 / kPi;
        if (!in.localization_valid) return "localization_invalid";
        if (std::fabs(in.linear_speed_mps) > cfg_.sparse_scan_max_linear_speed_mps) return "linear_speed_too_high";
        if (yaw_rate_dps < cfg_.sparse_scan_min_yaw_rate_dps) return "yaw_rate_too_low";
        if (yaw_rate_dps > cfg_.sparse_scan_max_yaw_rate_dps) return "yaw_rate_too_high";
        if (cfg_.sparse_scan_require_active_scan_recommended && !in.active_scan.scan_recommended) return "active_scan_not_recommended";
        if (cfg_.sparse_scan_require_mapping_allowed && !in.supervisor.mapping_allowed) return "mapping_not_allowed";
        if (state_ == SparseScanState::IDLE && cooldown_until_s_ >= 0.0 && in.timestamp_s < cooldown_until_s_) return "cooldown";
        return "not_collecting";
    }

    void reset_bins() {
        int n = std::max(1, static_cast<int>(std::ceil(360.0 / cfg_.sparse_scan_bin_angle_deg)));
        bins_.clear();
        bins_.resize(n);
        for (int i = 0; i < n; ++i) {
            bins_[i].bin_index = i;
            bins_[i].bin_center_deg = i * cfg_.sparse_scan_bin_angle_deg + cfg_.sparse_scan_bin_angle_deg * 0.5;
        }
    }

    void reset_session_runtime(double yaw_rad) {
        have_last_yaw_ = true;
        last_yaw_rad_ = yaw_rad;
        observed_yaw_delta_deg_ = 0.0;
        session_total_samples_ = 0;
        session_valid_samples_ = 0;
        session_invalid_samples_ = 0;
        session_front_valid_ = 0;
        session_left_valid_ = 0;
        session_right_valid_ = 0;
        last_valid_sample_s_ = -1.0;
        max_samples_reached_ = false;
        current_session_samples_.clear();
    }

    void start_session(const SparseScanInput &in) {
        scan_id_++;
        reset_bins();
        reset_session_runtime(in.pose.yaw);
        session_start_s_ = in.timestamp_s;
        last_valid_sample_s_ = in.timestamp_s;
        latest_input_ = in;
        transition_to(SparseScanState::COLLECTING, in.timestamp_s);
        stats_.sessions_started++;
    }

    void transition_to(SparseScanState next, double now_s) {
        if (state_ == next) return;
        previous_state_ = state_;
        state_ = next;
        state_enter_s_ = now_s;
    }

    void accumulate_yaw(double yaw_rad) {
        if (have_last_yaw_) {
            observed_yaw_delta_deg_ += std::fabs(wrap_pi(yaw_rad - last_yaw_rad_)) * 180.0 / kPi;
        }
        last_yaw_rad_ = yaw_rad;
        have_last_yaw_ = true;
    }

    SparseScanSample make_sample(const SparseScanInput &in, const FilteredTofSample &t, bool valid, const std::string &reason) const {
        SparseScanSample s;
        s.scan_id = scan_id_;
        s.timestamp_us = t.timestamp_us ? t.timestamp_us : in.timestamp_us;
        s.timestamp_s = in.timestamp_s;
        s.sensor_id = t.id;
        s.base_x_m = in.pose.x;
        s.base_y_m = in.pose.y;
        s.base_yaw_rad = in.pose.yaw;
        auto it = cfg_.tof_extrinsics.find(t.id);
        Config::TofExtrinsic ex;
        if (it != cfg_.tof_extrinsics.end()) ex = it->second;
        double cy = std::cos(in.pose.yaw);
        double sy = std::sin(in.pose.yaw);
        s.sensor_x_m = in.pose.x + cy * ex.x_m - sy * ex.y_m;
        s.sensor_y_m = in.pose.y + sy * ex.x_m + cy * ex.y_m;
        s.sensor_yaw_rad = wrap_pi(in.pose.yaw + deg2rad(ex.yaw_deg));
        s.global_bearing_rad = s.sensor_yaw_rad;
        s.global_bearing_deg = norm_deg_360(rad2deg(s.global_bearing_rad));
        s.range_m = t.range_m;
        s.confidence = t.confidence;
        s.range_status = t.range_status;
        s.valid = valid;
        s.hit = t.hit;
        if (cfg_.sparse_scan_compute_hit_points && valid && t.hit) {
            s.hit_x_m = s.sensor_x_m + t.range_m * std::cos(s.global_bearing_rad);
            s.hit_y_m = s.sensor_y_m + t.range_m * std::sin(s.global_bearing_rad);
        }
        s.observed_yaw_delta_deg = observed_yaw_delta_deg_;
        s.reason = reason;
        return s;
    }

    void update_bin(const SparseScanSample &s) {
        if (bins_.empty()) return;
        double norm = norm_deg_360(s.global_bearing_deg);
        int idx = static_cast<int>(std::floor(norm / cfg_.sparse_scan_bin_angle_deg));
        if (idx < 0) idx = 0;
        if (idx >= static_cast<int>(bins_.size())) idx = static_cast<int>(bins_.size()) - 1;
        SparseScanBin &b = bins_[idx];
        b.scan_id = scan_id_;
        b.bin_index = idx;
        b.bin_center_deg = idx * cfg_.sparse_scan_bin_angle_deg + cfg_.sparse_scan_bin_angle_deg * 0.5;
        b.sample_count++;
        b.sensor_mask |= sensor_bit(s.sensor_id);
        if (!s.valid) return;
        b.valid_count++;
        if (!b.valid || s.confidence > b.selected_confidence ||
            (s.confidence == b.selected_confidence && s.range_m < b.selected_range_m)) {
            b.valid = true;
            b.selected_range_m = s.range_m;
            b.selected_confidence = s.confidence;
        }
    }

    void collect_samples(const SparseScanInput &in, const std::string &base_reason) {
        latest_input_ = in;
        for (const auto &t : in.tof_samples) {
            if (session_total_samples_ >= cfg_.sparse_scan_max_samples_per_session) {
                max_samples_reached_ = true;
                break;
            }
            bool valid = valid_hit(t);
            std::string reason = valid ? base_reason : invalid_reason(t);
            if (!valid && !cfg_.sparse_scan_keep_invalid_samples) continue;
            SparseScanSample s = make_sample(in, t, valid, reason);
            pending_samples_.push_back(s);
            current_session_samples_.push_back(s);
            update_bin(s);
            session_total_samples_++;
            stats_.total_samples++;
            if (valid) {
                session_valid_samples_++;
                stats_.valid_samples++;
                last_valid_sample_s_ = in.timestamp_s;
                if (t.id == "front") { session_front_valid_++; stats_.front_valid++; }
                else if (t.id == "left") { session_left_valid_++; stats_.left_valid++; }
                else if (t.id == "right") { session_right_valid_++; stats_.right_valid++; }
            } else {
                session_invalid_samples_++;
                stats_.invalid_samples++;
            }
        }
    }

    void maybe_finalize_after_collect(const SparseScanInput &in, bool &changed) {
        if (max_samples_reached_) {
            finalize_session(in, observed_yaw_delta_deg_ >= cfg_.sparse_scan_useful_session_angle_deg, "max_samples_reached");
            changed = true;
        } else if (observed_yaw_delta_deg_ >= cfg_.sparse_scan_complete_session_angle_deg) {
            finalize_session(in, true, "complete");
            changed = true;
        }
    }

    int valid_bin_count() const {
        int n = 0;
        for (const auto &b : bins_) if (b.valid) n++;
        return n;
    }

    void finalize_session(const SparseScanInput &in, bool completed, const std::string &reason) {
        latest_input_ = in;
        transition_to(SparseScanState::FINALIZING, in.timestamp_s);
        for (const auto &b : bins_) pending_bins_.push_back(b);
        SparseScanSessionSummary summary;
        summary.scan_id = scan_id_;
        summary.start_s = session_start_s_;
        summary.end_s = in.timestamp_s;
        summary.duration_s = std::max(0.0, in.timestamp_s - session_start_s_);
        summary.state = completed ? "COMPLETED" : "ABORTED";
        summary.reason = reason;
        summary.total_samples = session_total_samples_;
        summary.valid_samples = session_valid_samples_;
        summary.invalid_samples = session_invalid_samples_;
        summary.front_valid = session_front_valid_;
        summary.left_valid = session_left_valid_;
        summary.right_valid = session_right_valid_;
        summary.observed_yaw_delta_deg = observed_yaw_delta_deg_;
        summary.total_bins = static_cast<int>(bins_.size());
        summary.valid_bins = valid_bin_count();
        summary.valid_bin_ratio = summary.total_bins ? static_cast<double>(summary.valid_bins) / summary.total_bins : 0.0;
        summary.useful = observed_yaw_delta_deg_ >= cfg_.sparse_scan_useful_session_angle_deg;
        summary.complete = observed_yaw_delta_deg_ >= cfg_.sparse_scan_complete_session_angle_deg;
        summary.quality_score = in.map_quality.quality_score;
        summary.supervisor_state = in.supervisor.state;
        summary.active_scan_state = in.active_scan.state;
        summary.active_scan_command_state = in.active_scan_command.state;
        latest_summary_ = summary;
        pending_summaries_.push_back(summary);
        SparseScanForMatching match_scan;
        match_scan.scan_id = scan_id_;
        match_scan.summary = summary;
        match_scan.samples = current_session_samples_;
        match_scan.bins = bins_;
        pending_match_scans_.push_back(match_scan);

        if (completed) stats_.sessions_completed++;
        else stats_.sessions_aborted++;
        if (summary.useful) stats_.useful_sessions++;
        if (summary.complete) stats_.complete_sessions++;
        stats_.last_observed_yaw_delta_deg = observed_yaw_delta_deg_;
        stats_.max_observed_yaw_delta_deg = std::max(stats_.max_observed_yaw_delta_deg, observed_yaw_delta_deg_);
        stats_.last_valid_bin_ratio = summary.valid_bin_ratio;
        stats_.valid_bin_ratio_sum += summary.valid_bin_ratio;
        stats_.valid_bin_ratio_count++;

        transition_to(completed ? SparseScanState::COMPLETED : SparseScanState::ABORTED, in.timestamp_s);
        cooldown_until_s_ = in.timestamp_s + cfg_.sparse_scan_cooldown_s;
    }

    const Config &cfg_;
    SparseScanState state_ = SparseScanState::IDLE;
    SparseScanState previous_state_ = SparseScanState::IDLE;
    double state_enter_s_ = 0.0;
    double last_update_s_ = -1.0;
    double last_log_s_ = -1.0;
    double cooldown_until_s_ = -1.0;
    uint64_t scan_id_ = 0;
    double session_start_s_ = 0.0;
    double last_valid_sample_s_ = -1.0;
    bool have_last_yaw_ = false;
    double last_yaw_rad_ = 0.0;
    double observed_yaw_delta_deg_ = 0.0;
    int session_total_samples_ = 0;
    int session_valid_samples_ = 0;
    int session_invalid_samples_ = 0;
    int session_front_valid_ = 0;
    int session_left_valid_ = 0;
    int session_right_valid_ = 0;
    bool max_samples_reached_ = false;
    SparseScanInput latest_input_;
    SparseScanSessionSummary latest_summary_;
    SparseScanRunStats stats_;
    std::vector<SparseScanBin> bins_;
    std::vector<SparseScanSample> current_session_samples_;
    std::vector<SparseScanSample> pending_samples_;
    std::vector<SparseScanBin> pending_bins_;
    std::vector<SparseScanSessionSummary> pending_summaries_;
    std::vector<SparseScanForMatching> pending_match_scans_;
};

inline void write_sparse_scan_samples_header(std::ofstream &o) {
    o << "timestamp_us,scan_id,state,sensor_id,base_x_m,base_y_m,base_yaw_rad,sensor_x_m,sensor_y_m,sensor_yaw_rad,global_bearing_deg,range_m,confidence,range_status,valid,hit,hit_x_m,hit_y_m,observed_yaw_delta_deg,reason\n";
}

inline void write_sparse_scan_bins_header(std::ofstream &o) {
    o << "scan_id,bin_index,bin_center_deg,selected_range_m,selected_confidence,sample_count,valid_count,sensor_mask,valid\n";
}

inline void write_sparse_scan_sessions_header(std::ofstream &o) {
    o << "scan_id,start_s,end_s,duration_s,state,reason,total_samples,valid_samples,invalid_samples,front_valid,left_valid,right_valid,observed_yaw_delta_deg,total_bins,valid_bins,valid_bin_ratio,useful,complete,quality_score,supervisor_state,active_scan_state,active_scan_command_state\n";
}

inline void write_sparse_scan_sample_row(std::ofstream &o, const SparseScanSample &s, const std::string &state) {
    o << s.timestamp_us << "," << s.scan_id << "," << state << "," << s.sensor_id << ","
      << s.base_x_m << "," << s.base_y_m << "," << s.base_yaw_rad << ","
      << s.sensor_x_m << "," << s.sensor_y_m << "," << s.sensor_yaw_rad << ","
      << s.global_bearing_deg << "," << s.range_m << "," << s.confidence << "," << s.range_status << ","
      << (s.valid ? 1 : 0) << "," << (s.hit ? 1 : 0) << "," << s.hit_x_m << "," << s.hit_y_m << ","
      << s.observed_yaw_delta_deg << "," << s.reason << "\n";
}

inline void write_sparse_scan_bin_row(std::ofstream &o, const SparseScanBin &b) {
    o << b.scan_id << "," << b.bin_index << "," << b.bin_center_deg << ","
      << b.selected_range_m << "," << b.selected_confidence << "," << b.sample_count << ","
      << b.valid_count << "," << b.sensor_mask << "," << (b.valid ? 1 : 0) << "\n";
}

inline void write_sparse_scan_session_row(std::ofstream &o, const SparseScanSessionSummary &s) {
    o << s.scan_id << "," << s.start_s << "," << s.end_s << "," << s.duration_s << ","
      << s.state << "," << s.reason << "," << s.total_samples << "," << s.valid_samples << ","
      << s.invalid_samples << "," << s.front_valid << "," << s.left_valid << "," << s.right_valid << ","
      << s.observed_yaw_delta_deg << "," << s.total_bins << "," << s.valid_bins << ","
      << s.valid_bin_ratio << "," << (s.useful ? 1 : 0) << "," << (s.complete ? 1 : 0) << ","
      << s.quality_score << "," << s.supervisor_state << "," << s.active_scan_state << ","
      << s.active_scan_command_state << "\n";
}

} // namespace robot_slamd
