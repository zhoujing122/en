#pragma once
#include "common.hpp"
#include "grid.hpp"
#include "map_quality.hpp"
#include "mapping_supervisor.hpp"
#include "sparse_scan_collector.hpp"

namespace robot_slamd {

struct YawMatchCandidate {
    uint64_t scan_id = 0;
    double timestamp_s = 0.0;
    double yaw_delta_deg = 0.0;
    double score = 0.0;
    int inliers = 0;
    int tested_samples = 0;
    double inlier_ratio = 0.0;
    double mean_distance_m = 0.0;
};

struct SparseScanYawMatchSummary {
    uint64_t scan_id = 0;
    double timestamp_s = 0.0;
    bool attempted = false;
    bool usable = false;
    std::string reason = "no_scan";
    double best_yaw_delta_deg = 0.0;
    double best_score = 0.0;
    double second_yaw_delta_deg = 0.0;
    double second_score = 0.0;
    double score_margin = 0.0;
    double inlier_ratio = 0.0;
    int inliers = 0;
    int tested_samples = 0;
    bool curve_flat = false;
    double curve_flatness = 0.0;
    bool multimodal = false;
    int peak_count = 0;
    double observed_yaw_delta_deg = 0.0;
    int valid_samples = 0;
    int valid_bins = 0;
    double valid_bin_ratio = 0.0;
    std::string quality_level;
    double map_quality_score = 0.0;
};

class SparseScanYawMatcher {
public:
    explicit SparseScanYawMatcher(const Config &cfg) : cfg_(cfg) {}

    bool update(double now_s,
                const std::vector<SparseScanForMatching> &completed_scans,
                const ChunkedGrid &grid,
                const MapQualitySnapshot &map_quality,
                const MappingSupervisorSnapshot &) {
        if (!cfg_.sparse_scan_yaw_match_enabled) return false;
        bool produced = false;
        for (const auto &scan : completed_scans) {
            SparseScanYawMatchSummary summary = match_one(now_s, scan, grid, map_quality);
            latest_summary_ = summary;
            pending_summaries_.push_back(summary);
            update_stats(summary);
            produced = true;
        }
        return produced;
    }

    bool should_log(double now_s) const {
        if (!cfg_.sparse_scan_yaw_match_enabled) return false;
        if (last_log_s_ < 0.0) return true;
        return now_s - last_log_s_ >= (1.0 / cfg_.sparse_scan_yaw_match_log_hz);
    }

    void mark_logged(double now_s) { last_log_s_ = now_s; }

    std::vector<YawMatchCandidate> drain_curve_rows() {
        std::vector<YawMatchCandidate> out;
        out.swap(pending_curve_);
        return out;
    }

    std::vector<SparseScanYawMatchSummary> drain_summaries() {
        std::vector<SparseScanYawMatchSummary> out;
        out.swap(pending_summaries_);
        return out;
    }

    SparseScanYawMatchSummary latest_summary() const { return latest_summary_; }
    SparseScanYawMatchRunStats run_stats(double) const { return stats_; }

private:
    struct HitPoint {
        double x = 0.0;
        double y = 0.0;
    };

    static double norm_score_margin(double best, double second) {
        return best - second;
    }

    HitPoint recompute_hit(const SparseScanSample &s, double yaw_delta_deg) const {
        auto it = cfg_.tof_extrinsics.find(s.sensor_id);
        Config::TofExtrinsic ex;
        if (it != cfg_.tof_extrinsics.end()) ex = it->second;
        double yaw = wrap_pi(s.base_yaw_rad + deg2rad(yaw_delta_deg));
        double cy = std::cos(yaw);
        double sy = std::sin(yaw);
        double sx = s.base_x_m + cy * ex.x_m - sy * ex.y_m;
        double syw = s.base_y_m + sy * ex.x_m + cy * ex.y_m;
        double bearing = wrap_pi(yaw + deg2rad(ex.yaw_deg));
        return {sx + s.range_m * std::cos(bearing), syw + s.range_m * std::sin(bearing)};
    }

    std::vector<SparseScanSample> select_valid_samples(const SparseScanForMatching &scan) const {
        std::vector<SparseScanSample> valid;
        for (const auto &s : scan.samples) {
            if (s.valid && s.hit) valid.push_back(s);
        }
        if (static_cast<int>(valid.size()) <= cfg_.sparse_scan_yaw_match_max_samples_per_match) return valid;
        std::vector<SparseScanSample> down;
        int step = std::max(1, static_cast<int>(std::ceil(static_cast<double>(valid.size()) / cfg_.sparse_scan_yaw_match_max_samples_per_match)));
        for (size_t i = 0; i < valid.size() && static_cast<int>(down.size()) < cfg_.sparse_scan_yaw_match_max_samples_per_match; i += static_cast<size_t>(step)) {
            down.push_back(valid[i]);
        }
        return down;
    }

    YawMatchCandidate score_candidate(uint64_t scan_id, double timestamp_s, const std::vector<SparseScanSample> &samples,
                                      double yaw_delta_deg, const ChunkedGrid &grid) const {
        YawMatchCandidate c;
        c.scan_id = scan_id;
        c.timestamp_s = timestamp_s;
        c.yaw_delta_deg = yaw_delta_deg;
        double score = 0.0;
        double dist_sum = 0.0;
        for (const auto &s : samples) {
            HitPoint hit = recompute_hit(s, yaw_delta_deg);
            GridQueryResult q = grid.query_world(hit.x, hit.y, cfg_);
            NearestOccupiedResult near = grid.nearest_occupied_distance(hit.x, hit.y, cfg_.sparse_scan_yaw_match_occupied_search_radius_m, cfg_);
            if (near.found) {
                double weight = std::max(0.0, 1.0 - near.distance_m / cfg_.sparse_scan_yaw_match_distance_penalty_scale_m);
                score += cfg_.sparse_scan_yaw_match_occupied_hit_reward * weight;
                c.inliers++;
                dist_sum += near.distance_m;
            } else if (q.free) {
                score -= cfg_.sparse_scan_yaw_match_free_hit_penalty;
            } else {
                score -= cfg_.sparse_scan_yaw_match_unknown_hit_penalty;
            }
            c.tested_samples++;
        }
        c.score = c.tested_samples ? score / static_cast<double>(c.tested_samples) : 0.0;
        c.inlier_ratio = c.tested_samples ? static_cast<double>(c.inliers) / c.tested_samples : 0.0;
        c.mean_distance_m = c.inliers ? dist_sum / static_cast<double>(c.inliers) : cfg_.sparse_scan_yaw_match_occupied_search_radius_m;
        return c;
    }

    void add_candidate(std::vector<YawMatchCandidate> &curve, const YawMatchCandidate &candidate) const {
        for (auto &c : curve) {
            if (std::fabs(c.yaw_delta_deg - candidate.yaw_delta_deg) < 1e-9) {
                if (candidate.score > c.score) c = candidate;
                return;
            }
        }
        curve.push_back(candidate);
    }

    void evaluate_range(std::vector<YawMatchCandidate> &curve, uint64_t scan_id, double timestamp_s,
                        const std::vector<SparseScanSample> &samples, const ChunkedGrid &grid,
                        double lo_deg, double hi_deg, double step_deg) const {
        if (step_deg <= 0.0) return;
        int steps = static_cast<int>(std::floor((hi_deg - lo_deg) / step_deg + 0.5));
        for (int i = 0; i <= steps; ++i) {
            double yaw = lo_deg + static_cast<double>(i) * step_deg;
            if (yaw < -cfg_.sparse_scan_yaw_match_max_yaw_search_deg - 1e-9 ||
                yaw > cfg_.sparse_scan_yaw_match_max_yaw_search_deg + 1e-9) continue;
            add_candidate(curve, score_candidate(scan_id, timestamp_s, samples, yaw, grid));
        }
    }

    static double median_score(std::vector<YawMatchCandidate> curve) {
        if (curve.empty()) return 0.0;
        std::sort(curve.begin(), curve.end(), [](const auto &a, const auto &b) { return a.score < b.score; });
        size_t n = curve.size();
        if (n % 2) return curve[n / 2].score;
        return 0.5 * (curve[n / 2 - 1].score + curve[n / 2].score);
    }

    int count_peaks(const std::vector<YawMatchCandidate> &curve) const {
        if (curve.empty()) return 0;
        std::vector<YawMatchCandidate> sorted = curve;
        std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) { return a.yaw_delta_deg < b.yaw_delta_deg; });
        int peaks = 0;
        for (size_t i = 0; i < sorted.size(); ++i) {
            bool ge_prev = i == 0 || sorted[i].score >= sorted[i - 1].score;
            bool ge_next = i + 1 == sorted.size() || sorted[i].score >= sorted[i + 1].score;
            if (ge_prev && ge_next) peaks++;
        }
        return peaks;
    }

    bool is_multimodal(const std::vector<YawMatchCandidate> &curve, const YawMatchCandidate &best) const {
        if (!cfg_.sparse_scan_yaw_match_multimodal_check_enabled || best.score <= 0.0) return false;
        std::vector<YawMatchCandidate> sorted = curve;
        std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) { return a.yaw_delta_deg < b.yaw_delta_deg; });
        for (size_t i = 0; i < sorted.size(); ++i) {
            bool ge_prev = i == 0 || sorted[i].score >= sorted[i - 1].score;
            bool ge_next = i + 1 == sorted.size() || sorted[i].score >= sorted[i + 1].score;
            if (!ge_prev || !ge_next) continue;
            if (std::fabs(sorted[i].yaw_delta_deg - best.yaw_delta_deg) < cfg_.sparse_scan_yaw_match_multimodal_peak_separation_deg) continue;
            if (sorted[i].score / best.score >= cfg_.sparse_scan_yaw_match_max_second_peak_ratio) return true;
        }
        return false;
    }

    SparseScanYawMatchSummary make_reject(double now_s, const SparseScanForMatching &scan, const MapQualitySnapshot &map_quality,
                                          const std::string &reason, bool attempted) const {
        SparseScanYawMatchSummary s;
        s.scan_id = scan.scan_id;
        s.timestamp_s = now_s;
        s.attempted = attempted;
        s.usable = false;
        s.reason = reason;
        s.observed_yaw_delta_deg = scan.summary.observed_yaw_delta_deg;
        s.valid_samples = scan.summary.valid_samples;
        s.valid_bins = scan.summary.valid_bins;
        s.valid_bin_ratio = scan.summary.valid_bin_ratio;
        s.quality_level = map_quality.quality_level;
        s.map_quality_score = map_quality.quality_score;
        return s;
    }

    SparseScanYawMatchSummary match_one(double now_s, const SparseScanForMatching &scan, const ChunkedGrid &grid,
                                        const MapQualitySnapshot &map_quality) {
        if (cfg_.sparse_scan_yaw_match_require_complete_session && !scan.summary.complete) {
            return make_reject(now_s, scan, map_quality, "require_complete_session", false);
        }
        if (scan.summary.complete && !cfg_.sparse_scan_yaw_match_run_on_completed_sparse_scan) {
            return make_reject(now_s, scan, map_quality, "completed_scan_disabled", false);
        }
        if (!scan.summary.complete && scan.summary.useful && !cfg_.sparse_scan_yaw_match_run_on_useful_sparse_scan) {
            return make_reject(now_s, scan, map_quality, "useful_scan_disabled", false);
        }
        if (!scan.summary.complete && !scan.summary.useful) {
            return make_reject(now_s, scan, map_quality, "scan_not_useful", false);
        }
        if (scan.summary.valid_samples < cfg_.sparse_scan_yaw_match_min_valid_samples) {
            return make_reject(now_s, scan, map_quality, "insufficient_samples", false);
        }
        if (scan.summary.valid_bins < cfg_.sparse_scan_yaw_match_min_valid_bins) {
            return make_reject(now_s, scan, map_quality, "insufficient_bins", false);
        }
        if (scan.summary.observed_yaw_delta_deg < cfg_.sparse_scan_yaw_match_min_observed_yaw_delta_deg) {
            return make_reject(now_s, scan, map_quality, "insufficient_yaw_coverage", false);
        }
        if (scan.summary.valid_bin_ratio < cfg_.sparse_scan_yaw_match_min_valid_bin_ratio) {
            return make_reject(now_s, scan, map_quality, "low_valid_bin_ratio", false);
        }
        GridStats gst = grid.stats(cfg_);
        if (gst.occupied_cells == 0 || gst.known_cells == 0) {
            return make_reject(now_s, scan, map_quality, "no_map_support", false);
        }

        std::vector<SparseScanSample> samples = select_valid_samples(scan);
        if (static_cast<int>(samples.size()) < cfg_.sparse_scan_yaw_match_min_valid_samples) {
            return make_reject(now_s, scan, map_quality, "insufficient_samples", false);
        }

        std::vector<YawMatchCandidate> curve;
        evaluate_range(curve, scan.scan_id, now_s, samples, grid,
                       -cfg_.sparse_scan_yaw_match_max_yaw_search_deg,
                       cfg_.sparse_scan_yaw_match_max_yaw_search_deg,
                       cfg_.sparse_scan_yaw_match_coarse_step_deg);
        if (curve.empty()) return make_reject(now_s, scan, map_quality, "no_candidates", false);
        auto best_it = std::max_element(curve.begin(), curve.end(), [](const auto &a, const auto &b) { return a.score < b.score; });
        YawMatchCandidate coarse_best = *best_it;
        if (cfg_.sparse_scan_yaw_match_refine_enabled) {
            evaluate_range(curve, scan.scan_id, now_s, samples, grid,
                           coarse_best.yaw_delta_deg - cfg_.sparse_scan_yaw_match_refine_window_deg,
                           coarse_best.yaw_delta_deg + cfg_.sparse_scan_yaw_match_refine_window_deg,
                           cfg_.sparse_scan_yaw_match_refine_step_deg);
        }
        std::sort(curve.begin(), curve.end(), [](const auto &a, const auto &b) { return a.yaw_delta_deg < b.yaw_delta_deg; });
        for (const auto &c : curve) pending_curve_.push_back(c);

        YawMatchCandidate best;
        YawMatchCandidate second;
        bool have_best = false;
        bool have_second = false;
        for (const auto &c : curve) {
            if (!have_best || c.score > best.score) {
                if (have_best) { second = best; have_second = true; }
                best = c;
                have_best = true;
            } else if (std::fabs(c.yaw_delta_deg - best.yaw_delta_deg) > 1e-9 && (!have_second || c.score > second.score)) {
                second = c;
                have_second = true;
            }
        }
        if (!have_second) second = best;

        double median = median_score(curve);
        double sharpness = (best.score - median) / std::max(std::fabs(best.score), 1e-9);
        double curve_flatness = 1.0 - clamp01(sharpness);
        bool curve_flat = curve_flatness > cfg_.sparse_scan_yaw_match_max_curve_flatness;
        int peaks = count_peaks(curve);
        bool multimodal = is_multimodal(curve, best);

        SparseScanYawMatchSummary summary;
        summary.scan_id = scan.scan_id;
        summary.timestamp_s = now_s;
        summary.attempted = true;
        summary.best_yaw_delta_deg = best.yaw_delta_deg;
        summary.best_score = best.score;
        summary.second_yaw_delta_deg = second.yaw_delta_deg;
        summary.second_score = second.score;
        summary.score_margin = norm_score_margin(best.score, second.score);
        summary.inlier_ratio = best.inlier_ratio;
        summary.inliers = best.inliers;
        summary.tested_samples = best.tested_samples;
        summary.curve_flat = curve_flat;
        summary.curve_flatness = curve_flatness;
        summary.multimodal = multimodal;
        summary.peak_count = peaks;
        summary.observed_yaw_delta_deg = scan.summary.observed_yaw_delta_deg;
        summary.valid_samples = scan.summary.valid_samples;
        summary.valid_bins = scan.summary.valid_bins;
        summary.valid_bin_ratio = scan.summary.valid_bin_ratio;
        summary.quality_level = map_quality.quality_level;
        summary.map_quality_score = map_quality.quality_score;

        if (best.score < cfg_.sparse_scan_yaw_match_min_best_score) summary.reason = "low_best_score";
        else if (summary.score_margin < cfg_.sparse_scan_yaw_match_min_score_margin) summary.reason = "low_score_margin";
        else if (summary.inlier_ratio < cfg_.sparse_scan_yaw_match_min_inlier_ratio) summary.reason = "low_inlier_ratio";
        else if (curve_flat) summary.reason = "curve_flat";
        else if (multimodal) summary.reason = "multimodal";
        else if (std::fabs(best.yaw_delta_deg) > cfg_.sparse_scan_yaw_match_max_yaw_search_deg + 1e-9) summary.reason = "outside_search_range";
        else {
            summary.reason = "ok";
            summary.usable = true;
        }
        return summary;
    }

    void update_stats(const SparseScanYawMatchSummary &s) {
        stats_.attempts++;
        stats_.last_best_yaw_delta_deg = s.best_yaw_delta_deg;
        stats_.last_best_score = s.best_score;
        stats_.last_score_margin = s.score_margin;
        stats_.last_inlier_ratio = s.inlier_ratio;
        stats_.last_usable = s.usable;
        stats_.last_reason = s.reason;
        if (s.usable) stats_.usable_count++;
        else stats_.rejected_count++;
        if (s.curve_flat) stats_.curve_flat_count++;
        if (s.multimodal) stats_.multimodal_count++;
        if (s.reason == "insufficient_samples" || s.reason == "insufficient_bins" ||
            s.reason == "insufficient_yaw_coverage" || s.reason == "low_valid_bin_ratio" ||
            s.reason == "no_map_support" || s.reason == "scan_not_useful") {
            stats_.insufficient_data_count++;
        }
    }

    const Config &cfg_;
    double last_log_s_ = -1.0;
    SparseScanYawMatchSummary latest_summary_;
    SparseScanYawMatchRunStats stats_;
    std::vector<YawMatchCandidate> pending_curve_;
    std::vector<SparseScanYawMatchSummary> pending_summaries_;
};

inline void write_yaw_match_curve_header(std::ofstream &o) {
    o << "timestamp_s,scan_id,yaw_delta_deg,score,inliers,tested_samples,inlier_ratio,mean_distance_m\n";
}

inline void write_yaw_match_summary_header(std::ofstream &o) {
    o << "timestamp_s,scan_id,attempted,usable,reason,best_yaw_delta_deg,best_score,second_yaw_delta_deg,second_score,score_margin,inlier_ratio,inliers,tested_samples,curve_flat,curve_flatness,multimodal,peak_count,observed_yaw_delta_deg,valid_samples,valid_bins,valid_bin_ratio,map_quality_score,quality_level\n";
}

inline void write_yaw_match_curve_row(std::ofstream &o, const YawMatchCandidate &c) {
    o << c.timestamp_s << "," << c.scan_id << "," << c.yaw_delta_deg << "," << c.score << ","
      << c.inliers << "," << c.tested_samples << "," << c.inlier_ratio << "," << c.mean_distance_m << "\n";
}

inline void write_yaw_match_summary_row(std::ofstream &o, const SparseScanYawMatchSummary &s) {
    o << s.timestamp_s << "," << s.scan_id << "," << (s.attempted ? 1 : 0) << "," << (s.usable ? 1 : 0) << ","
      << s.reason << "," << s.best_yaw_delta_deg << "," << s.best_score << ","
      << s.second_yaw_delta_deg << "," << s.second_score << "," << s.score_margin << ","
      << s.inlier_ratio << "," << s.inliers << "," << s.tested_samples << ","
      << (s.curve_flat ? 1 : 0) << "," << s.curve_flatness << "," << (s.multimodal ? 1 : 0) << ","
      << s.peak_count << "," << s.observed_yaw_delta_deg << "," << s.valid_samples << ","
      << s.valid_bins << "," << s.valid_bin_ratio << "," << s.map_quality_score << "," << s.quality_level << "\n";
}

} // namespace robot_slamd
