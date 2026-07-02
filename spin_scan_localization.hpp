#pragma once
#include "common.hpp"
#include "grid.hpp"

namespace robot_slamd {

struct FilteredTofSample {
    uint64_t timestamp_us = 0;
    std::string id;
    double range_m = 0.0;
    int confidence = 0;
    int range_status = 7;
    bool map_update = false;
    bool hit = false;
    std::string decision;
};

struct SpinScanSample {
    uint64_t timestamp_us = 0;
    int scan_id = 0;
    std::string state;
    double base_x = 0.0;
    double base_y = 0.0;
    double base_yaw = 0.0;
    double linear_speed_mps = 0.0;
    double angular_speed_radps = 0.0;
    double accumulated_rotation_deg = 0.0;
    std::string sensor_id;
    double sensor_yaw = 0.0;
    double global_bearing = 0.0;
    double range_m = 0.0;
    int confidence = 0;
    int range_status = 7;
    bool valid = false;
    std::string reason;
};

struct SpinScanBin {
    int scan_id = 0;
    int bin_index = 0;
    double bin_center_deg = 0.0;
    double selected_range_m = 0.0;
    int selected_confidence = 0;
    int sample_count = 0;
    int valid_count = 0;
    int sensor_mask = 0;
    bool valid = false;
};

struct SpinMatchCurvePoint {
    uint64_t timestamp_us = 0;
    int scan_id = 0;
    double candidate_yaw = 0.0;
    double yaw_offset_deg = 0.0;
    double total_residual = 0.0;
    int valid_bins = 0;
    int front_like_bins = 0;
    int left_like_bins = 0;
    int right_like_bins = 0;
    std::string reason;
};

struct SpinScanMatchResult {
    int scan_id = 0;
    uint64_t timestamp_us = 0;
    bool complete = false;
    bool matched = false;
    double candidate_yaw = 0.0;
    double delta_yaw = 0.0;
    double best_residual = 0.0;
    double second_residual = 0.0;
    double third_residual = 0.0;
    double best_second_margin = 0.0;
    double best_second_yaw_gap_deg = 0.0;
    double curve_min_residual = 0.0;
    double curve_mean_residual = 0.0;
    double curve_p25_residual = 0.0;
    double curve_p50_residual = 0.0;
    double curve_p75_residual = 0.0;
    double curve_sharpness = 0.0;
    bool flat_curve = false;
    bool multimodal = false;
    bool best_at_search_edge = false;
    int num_local_minima = 0;
    int valid_bins = 0;
    int total_bins = 0;
    double valid_bin_ratio = 0.0;
    double reliability = 0.0;
    bool usable = false;
    std::string reason;
};

class SpinScanLocalizer {
public:
    explicit SpinScanLocalizer(const Config &cfg) : cfg_(cfg) {
        reset_bins(cfg);
    }

    void update(uint64_t timestamp_us, const Pose &pose, const std::vector<FilteredTofSample> &filtered_tof, const ChunkedGrid &grid, const Config &cfg) {
        if (!cfg.spin_scan_enabled || cfg.spin_scan_mode != "observe_only") return;
        double linear_speed = 0.0;
        double angular_speed = 0.0;
        if (have_pose_) {
            double dt = std::max(1e-6, static_cast<double>(timestamp_us - last_pose_time_us_) / 1000000.0);
            linear_speed = std::hypot(pose.x - last_pose_.x, pose.y - last_pose_.y) / dt;
            angular_speed = wrap_pi(pose.yaw - last_pose_.yaw) / dt;
        }
        last_pose_ = pose;
        last_pose_time_us_ = timestamp_us;
        have_pose_ = true;

        bool any_valid_tof = false;
        for (const auto &t : filtered_tof) {
            if (valid_hit(t, cfg)) any_valid_tof = true;
        }
        bool linear_ok = std::fabs(linear_speed) <= cfg.spin_scan_max_linear_speed_mps;
        bool angular_ok = std::fabs(angular_speed) >= cfg.spin_scan_angular_speed_min_radps &&
                          std::fabs(angular_speed) <= cfg.spin_scan_angular_speed_max_radps;

        if (state_ == "IDLE") {
            if (linear_ok && angular_ok && any_valid_tof) start_scan(timestamp_us, pose);
            else if (!filtered_tof.empty()) {
                std::string reason = !linear_ok ? "linear_speed_too_high" :
                                     (!angular_ok && std::fabs(angular_speed) > cfg.spin_scan_angular_speed_max_radps) ? "angular_speed_out_of_range" :
                                     (!angular_ok ? "not_spinning" : "no_valid_tof");
                emit_samples(timestamp_us, pose, filtered_tof, linear_speed, angular_speed, 0.0, reason);
            }
        } else if (state_ == "DONE" || state_ == "FAILED") {
            if (std::fabs(angular_speed) < cfg.spin_scan_angular_speed_min_radps) {
                state_ = "IDLE";
                reset_bins(cfg);
            }
            return;
        }

        if (state_ != "COLLECTING") return;

        double delta_yaw = have_collect_pose_ ? wrap_pi(pose.yaw - last_collect_pose_.yaw) : 0.0;
        accumulated_rotation_rad_ += std::fabs(delta_yaw);
        last_collect_pose_ = pose;
        have_collect_pose_ = true;
        double accumulated_deg = rad2deg(accumulated_rotation_rad_);

        if (!linear_ok) {
            fail_scan(timestamp_us, pose, "linear_speed_too_high");
            return;
        }
        if (std::fabs(angular_speed) > cfg.spin_scan_angular_speed_max_radps) {
            fail_scan(timestamp_us, pose, "angular_speed_out_of_range");
            return;
        }

        collect_samples(timestamp_us, pose, filtered_tof, linear_speed, angular_speed, accumulated_deg, cfg);

        double elapsed_s = static_cast<double>(timestamp_us - scan_start_time_us_) / 1000000.0;
        if (accumulated_deg >= cfg.spin_scan_min_rotation_deg) {
            match_scan(timestamp_us, pose, grid, cfg, true, "complete");
            state_ = "DONE";
        } else if (elapsed_s >= cfg.spin_scan_max_duration_s) {
            fail_scan(timestamp_us, pose, "insufficient_rotation");
        } else if (std::fabs(angular_speed) < cfg.spin_scan_angular_speed_min_radps && elapsed_s > 0.5) {
            fail_scan(timestamp_us, pose, "insufficient_rotation");
        }
    }

    bool has_active_scan() const { return state_ == "COLLECTING"; }
    int current_scan_id() const { return scan_id_; }

    std::vector<SpinScanSample> drain_samples() {
        std::vector<SpinScanSample> out;
        out.swap(pending_samples_);
        return out;
    }

    std::vector<SpinScanBin> drain_bins() {
        std::vector<SpinScanBin> out;
        out.swap(pending_bins_);
        return out;
    }

    std::vector<SpinMatchCurvePoint> drain_curve() {
        std::vector<SpinMatchCurvePoint> out;
        out.swap(pending_curve_);
        return out;
    }

    std::vector<SpinScanMatchResult> drain_summary() {
        std::vector<SpinScanMatchResult> out;
        out.swap(pending_summary_);
        return out;
    }

private:
    static double rad2deg(double r) { return r * 180.0 / kPi; }

    static double deg_norm_360(double d) {
        while (d < 0.0) d += 360.0;
        while (d >= 360.0) d -= 360.0;
        return d;
    }

    static double percentile_sorted(const std::vector<double> &sorted, double q) {
        if (sorted.empty()) return 0.0;
        double pos = q * static_cast<double>(sorted.size() - 1);
        size_t lo = static_cast<size_t>(std::floor(pos));
        size_t hi = static_cast<size_t>(std::ceil(pos));
        if (lo == hi) return sorted[lo];
        double t = pos - static_cast<double>(lo);
        return sorted[lo] * (1.0 - t) + sorted[hi] * t;
    }

    static int sensor_bit(const std::string &id) {
        if (id == "front") return 1;
        if (id == "left") return 2;
        if (id == "right") return 4;
        return 0;
    }

    static bool valid_hit(const FilteredTofSample &t, const Config &cfg) {
        return t.map_update && t.hit && t.confidence >= cfg.confidence_min &&
               t.range_m >= cfg.tof_min_range_m && t.range_m <= cfg.tof_max_range_m;
    }

    void reset_bins(const Config &cfg) {
        int n = std::max(1, static_cast<int>(std::round(360.0 / std::max(0.1, cfg.spin_scan_angle_bin_deg))));
        bins_.clear();
        bins_.resize(n);
        for (int i = 0; i < n; ++i) {
            bins_[i].scan_id = scan_id_;
            bins_[i].bin_index = i;
            bins_[i].bin_center_deg = i * (360.0 / n);
        }
    }

    void start_scan(uint64_t timestamp_us, const Pose &pose) {
        scan_id_++;
        reset_bins(cfg_);
        state_ = "COLLECTING";
        scan_start_time_us_ = timestamp_us;
        scan_start_pose_ = pose;
        last_collect_pose_ = pose;
        have_collect_pose_ = true;
        accumulated_rotation_rad_ = 0.0;
    }

    void emit_samples(uint64_t timestamp_us, const Pose &pose, const std::vector<FilteredTofSample> &filtered_tof,
                      double linear_speed, double angular_speed, double accumulated_deg, const std::string &reason) {
        for (const auto &t : filtered_tof) {
            auto ext_it = cfg_.tof_extrinsics.find(t.id);
            double sensor_yaw = ext_it == cfg_.tof_extrinsics.end() ? 0.0 : deg2rad(ext_it->second.yaw_deg);
            double bearing = wrap_pi(pose.yaw + sensor_yaw);
            SpinScanSample s;
            s.timestamp_us = timestamp_us;
            s.scan_id = scan_id_;
            s.state = state_;
            s.base_x = pose.x;
            s.base_y = pose.y;
            s.base_yaw = pose.yaw;
            s.linear_speed_mps = linear_speed;
            s.angular_speed_radps = angular_speed;
            s.accumulated_rotation_deg = accumulated_deg;
            s.sensor_id = t.id;
            s.sensor_yaw = sensor_yaw;
            s.global_bearing = bearing;
            s.range_m = t.range_m;
            s.confidence = t.confidence;
            s.range_status = t.range_status;
            s.valid = valid_hit(t, cfg_);
            s.reason = reason;
            pending_samples_.push_back(s);
        }
    }

    void collect_samples(uint64_t timestamp_us, const Pose &pose, const std::vector<FilteredTofSample> &filtered_tof,
                         double linear_speed, double angular_speed, double accumulated_deg, const Config &cfg) {
        emit_samples(timestamp_us, pose, filtered_tof, linear_speed, angular_speed, accumulated_deg, "collecting");
        int n = std::max(1, static_cast<int>(bins_.size()));
        double bin_deg = 360.0 / n;
        for (const auto &t : filtered_tof) {
            auto ext_it = cfg.tof_extrinsics.find(t.id);
            if (ext_it == cfg.tof_extrinsics.end()) continue;
            double bearing_deg = deg_norm_360(rad2deg(pose.yaw + deg2rad(ext_it->second.yaw_deg)));
            int idx = static_cast<int>(std::floor((bearing_deg + bin_deg * 0.5) / bin_deg)) % n;
            SpinScanBin &b = bins_[idx];
            b.scan_id = scan_id_;
            b.bin_index = idx;
            b.bin_center_deg = idx * bin_deg;
            b.sample_count++;
            b.sensor_mask |= sensor_bit(t.id);
            if (!valid_hit(t, cfg)) continue;
            b.valid_count++;
            if (!b.valid || t.confidence > b.selected_confidence) {
                b.valid = true;
                b.selected_range_m = t.range_m;
                b.selected_confidence = t.confidence;
            }
        }
    }

    void fail_scan(uint64_t timestamp_us, const Pose &pose, const std::string &reason) {
        SpinScanMatchResult r;
        r.scan_id = scan_id_;
        r.timestamp_us = timestamp_us;
        r.complete = false;
        r.matched = false;
        r.candidate_yaw = pose.yaw;
        r.delta_yaw = 0.0;
        r.total_bins = static_cast<int>(bins_.size());
        r.valid_bins = count_valid_bins();
        r.valid_bin_ratio = r.total_bins ? static_cast<double>(r.valid_bins) / r.total_bins : 0.0;
        r.reason = reason;
        pending_summary_.push_back(r);
        dump_bins();
        state_ = "FAILED";
    }

    int count_valid_bins() const {
        int n = 0;
        for (const auto &b : bins_) if (b.valid) n++;
        return n;
    }

    double expected_range_for_bearing(const Pose &base, double bearing_rad, const ChunkedGrid &grid, const Config &cfg) const {
        Config::TofExtrinsic ext;
        ext.x_m = 0.0;
        ext.y_m = 0.0;
        ext.yaw_deg = rad2deg(bearing_rad);
        ext.fov_deg = 1.0;
        Pose p = base;
        p.yaw = 0.0;
        return grid.ray_cast_expected_range(p, ext, cfg.tof_max_range_m, std::max(0.005, cfg.resolution_m * 0.5), cfg);
    }

    double score_offset(const Pose &pose, double yaw_offset_deg, const ChunkedGrid &grid, const Config &cfg,
                        int &valid_bins, int &front_like_bins, int &left_like_bins, int &right_like_bins) const {
        double sum = 0.0;
        valid_bins = front_like_bins = left_like_bins = right_like_bins = 0;
        for (const auto &b : bins_) {
            if (!b.valid) continue;
            double bearing_rad = deg2rad(b.bin_center_deg + yaw_offset_deg);
            double expected = expected_range_for_bearing(pose, bearing_rad, grid, cfg);
            sum += std::fabs(b.selected_range_m - expected);
            valid_bins++;
            if (b.sensor_mask & 1) front_like_bins++;
            if (b.sensor_mask & 2) left_like_bins++;
            if (b.sensor_mask & 4) right_like_bins++;
        }
        return valid_bins ? sum / static_cast<double>(valid_bins) : 1e9;
    }

    void match_scan(uint64_t timestamp_us, const Pose &pose, const ChunkedGrid &grid, const Config &cfg, bool complete, const std::string &base_reason) {
        int total_bins = static_cast<int>(bins_.size());
        int valid_bins_total = count_valid_bins();
        double valid_ratio = total_bins ? static_cast<double>(valid_bins_total) / total_bins : 0.0;
        if (valid_bins_total < cfg.spin_scan_min_valid_bins || valid_ratio < cfg.spin_scan_min_valid_bin_ratio) {
            SpinScanMatchResult r;
            r.scan_id = scan_id_;
            r.timestamp_us = timestamp_us;
            r.complete = complete;
            r.matched = false;
            r.candidate_yaw = pose.yaw;
            r.total_bins = total_bins;
            r.valid_bins = valid_bins_total;
            r.valid_bin_ratio = valid_ratio;
            r.reason = "not_enough_bins";
            pending_summary_.push_back(r);
            dump_bins();
            return;
        }

        double best = 1e9, second = 1e9, third = 1e9;
        double best_offset = 0.0, second_offset = 0.0, third_offset = 0.0;
        int steps = static_cast<int>(std::floor(cfg.spin_scan_match_search_yaw_deg / cfg.spin_scan_match_search_yaw_step_deg + 1e-9));
        std::vector<std::pair<double, double>> curve;
        for (int i = -steps; i <= steps; ++i) {
            double offset = i * cfg.spin_scan_match_search_yaw_step_deg;
            int vb = 0, fb = 0, lb = 0, rb = 0;
            double residual = score_offset(pose, offset, grid, cfg, vb, fb, lb, rb);
            curve.push_back({offset, residual});
            if (cfg.spin_scan_debug_curve) {
                pending_curve_.push_back({timestamp_us, scan_id_, wrap_pi(pose.yaw + deg2rad(offset)), offset, residual, vb, fb, lb, rb, "candidate"});
            }
            if (residual < best) {
                third = second; third_offset = second_offset;
                second = best; second_offset = best_offset;
                best = residual; best_offset = offset;
            } else if (residual < second) {
                third = second; third_offset = second_offset;
                second = residual; second_offset = offset;
            } else if (residual < third) {
                third = residual; third_offset = offset;
            }
        }

        SpinScanMatchResult r;
        r.scan_id = scan_id_;
        r.timestamp_us = timestamp_us;
        r.complete = complete;
        r.matched = best < 1e8;
        r.candidate_yaw = wrap_pi(pose.yaw + deg2rad(best_offset));
        r.delta_yaw = deg2rad(best_offset);
        r.best_residual = best;
        r.second_residual = second < 1e8 ? second : best;
        r.third_residual = third < 1e8 ? third : r.second_residual;
        r.best_second_margin = r.second_residual - r.best_residual;
        r.best_second_yaw_gap_deg = std::fabs(best_offset - second_offset);
        r.valid_bins = valid_bins_total;
        r.total_bins = total_bins;
        r.valid_bin_ratio = valid_ratio;
        r.reason = base_reason;
        summarize_curve(curve, best, best_offset, r, cfg);
        r.reliability = compute_reliability(r, cfg);
        r.usable = r.complete && r.matched &&
                   r.valid_bins >= cfg.spin_scan_min_valid_bins &&
                   r.valid_bin_ratio >= cfg.spin_scan_min_valid_bin_ratio &&
                   r.reliability >= cfg.spin_scan_min_reliability &&
                   !r.flat_curve &&
                   !r.multimodal &&
                   !r.best_at_search_edge &&
                   r.best_second_margin >= cfg.spin_scan_min_best_second_margin;
        if (!r.usable && r.reason == "complete") {
            if (r.flat_curve) r.reason = "flat_curve";
            else if (r.multimodal) r.reason = "multimodal";
            else if (r.best_at_search_edge) r.reason = "best_at_search_edge";
            else if (r.best_second_margin < cfg.spin_scan_min_best_second_margin) r.reason = "low_margin";
        }
        pending_summary_.push_back(r);
        dump_bins();
    }

    static double compute_reliability(const SpinScanMatchResult &r, const Config &cfg) {
        if (!r.complete || !r.matched) return 0.0;
        if (r.valid_bins < cfg.spin_scan_min_valid_bins || r.valid_bin_ratio < cfg.spin_scan_min_valid_bin_ratio) return 0.0;
        double q = 1.0;
        if (r.flat_curve) q -= 0.4;
        if (r.multimodal) q -= 0.4;
        if (r.best_at_search_edge) q -= 0.3;
        if (r.best_second_margin < cfg.spin_scan_min_best_second_margin) q -= 0.3;
        return clamp01(q);
    }

    void summarize_curve(const std::vector<std::pair<double, double>> &curve, double best, double best_offset, SpinScanMatchResult &r, const Config &cfg) const {
        if (curve.empty()) return;
        std::vector<double> residuals;
        residuals.reserve(curve.size());
        double sum = 0.0;
        for (const auto &p : curve) {
            residuals.push_back(p.second);
            sum += p.second;
        }
        std::sort(residuals.begin(), residuals.end());
        r.curve_min_residual = residuals.front();
        r.curve_mean_residual = sum / static_cast<double>(residuals.size());
        r.curve_p25_residual = percentile_sorted(residuals, 0.25);
        r.curve_p50_residual = percentile_sorted(residuals, 0.50);
        r.curve_p75_residual = percentile_sorted(residuals, 0.75);
        r.curve_sharpness = r.curve_p50_residual - r.curve_min_residual;
        r.flat_curve = r.curve_sharpness < cfg.spin_scan_flat_curve_sharpness_thresh;
        r.best_at_search_edge = std::fabs(std::fabs(best_offset) - cfg.spin_scan_match_search_yaw_deg) <= (cfg.spin_scan_match_search_yaw_step_deg * 0.5 + 1e-9);

        std::vector<double> minima;
        for (size_t i = 0; i < curve.size(); ++i) {
            bool lower_prev = i == 0 || curve[i].second <= curve[i - 1].second;
            bool lower_next = i + 1 == curve.size() || curve[i].second <= curve[i + 1].second;
            if (lower_prev && lower_next && curve[i].second <= best + cfg.spin_scan_multimodal_residual_window) {
                bool separated = true;
                for (double prev : minima) {
                    if (std::fabs(curve[i].first - prev) <= cfg.spin_scan_multimodal_yaw_gap_deg) separated = false;
                }
                if (separated) minima.push_back(curve[i].first);
            }
        }
        r.num_local_minima = static_cast<int>(minima.size());
        r.multimodal = r.num_local_minima >= 2;
    }

    void dump_bins() {
        for (const auto &b : bins_) pending_bins_.push_back(b);
    }

    const Config &cfg_;
    std::string state_ = "IDLE";
    int scan_id_ = 0;
    bool have_pose_ = false;
    Pose last_pose_;
    uint64_t last_pose_time_us_ = 0;
    bool have_collect_pose_ = false;
    Pose last_collect_pose_;
    Pose scan_start_pose_;
    uint64_t scan_start_time_us_ = 0;
    double accumulated_rotation_rad_ = 0.0;
    std::vector<SpinScanBin> bins_;
    std::vector<SpinScanSample> pending_samples_;
    std::vector<SpinScanBin> pending_bins_;
    std::vector<SpinMatchCurvePoint> pending_curve_;
    std::vector<SpinScanMatchResult> pending_summary_;
};

} // namespace robot_slamd
