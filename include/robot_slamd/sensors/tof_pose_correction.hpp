#pragma once
#include "robot_slamd/core/common.hpp"
#include "robot_slamd/core/grid.hpp"

namespace robot_slamd {

struct TofObsForCorrection {
    std::string id;
    double range_m = 0.0;
    bool hit = false;
    bool free_only = false;
    int confidence = 0;
};

struct YawResidualCurvePoint {
    double candidate_yaw = 0.0;
    double yaw_offset_deg = 0.0;
    double total_residual = 0.0;
    double front_obs_m = 0.0;
    double front_expected_m = 0.0;
    double front_residual_m = 0.0;
    double left_obs_m = 0.0;
    double left_expected_m = 0.0;
    double left_residual_m = 0.0;
    double right_obs_m = 0.0;
    double right_expected_m = 0.0;
    double right_residual_m = 0.0;
    int used_tof = 0;
    bool valid_candidate = false;
    std::string reason;
};

struct PoseCorrectionResult {
    bool accepted = false;
    Pose corrected_pose;
    double dx = 0.0;
    double dy = 0.0;
    double dyaw = 0.0;
    double best_residual = 0.0;
    double second_residual = 0.0;
    double third_residual = 0.0;
    double odom_residual = 0.0;
    int used_tof = 0;
    int candidate_count = 0;
    double odom_front_expected_m = 0.0;
    double odom_left_expected_m = 0.0;
    double odom_right_expected_m = 0.0;
    double best_front_expected_m = 0.0;
    double best_left_expected_m = 0.0;
    double best_right_expected_m = 0.0;
    double best_yaw = 0.0;
    double second_yaw = 0.0;
    double third_yaw = 0.0;
    double best_second_margin = 0.0;
    double second_third_margin = 0.0;
    double best_second_yaw_gap_deg = 0.0;
    double best_third_yaw_gap_deg = 0.0;
    double curve_min_residual = 0.0;
    double curve_mean_residual = 0.0;
    double curve_p25_residual = 0.0;
    double curve_p50_residual = 0.0;
    double curve_p75_residual = 0.0;
    double curve_sharpness = 0.0;
    bool best_at_search_edge = false;
    int num_local_minima = 0;
    bool multimodal = false;
    bool flat_curve = false;
    std::vector<YawResidualCurvePoint> yaw_curve;
    std::string reason;
};

class TofPoseCorrector {
public:
    explicit TofPoseCorrector(const Config &cfg) : cfg_(cfg) {}

    PoseCorrectionResult correct(const Pose &odom_pose, const std::vector<TofObsForCorrection> &obs, const ChunkedGrid &grid, const Config &cfg) const {
        PoseCorrectionResult result;
        result.corrected_pose = odom_pose;
        if (!cfg.tof_pose_correction_enabled || cfg.tof_pose_correction_mode == "off") {
            result.reason = "disabled";
            return result;
        }
        if (cfg.tof_pose_correction_mode != "score_only" && cfg.tof_pose_correction_mode != "yaw_candidate") {
            result.reason = "mode_not_implemented";
            return result;
        }

        int usable = usable_obs(obs, cfg);
        result.used_tof = usable;
        if (usable < cfg.tof_pose_correction_min_valid_tof) {
            result.reason = "insufficient_tof";
            return result;
        }

        DetailedScore odom_score = score_pose_detail(odom_pose, obs, grid, cfg);
        result.odom_residual = odom_score.total_residual;
        result.odom_front_expected_m = odom_score.front_expected_m;
        result.odom_left_expected_m = odom_score.left_expected_m;
        result.odom_right_expected_m = odom_score.right_expected_m;

        double best = 1e9;
        double second = 1e9;
        double third = 1e9;
        Pose best_pose = odom_pose;
        Pose second_pose = odom_pose;
        Pose third_pose = odom_pose;
        double best_front = 0.0, best_left = 0.0, best_right = 0.0;
        int xy_steps = cfg.tof_pose_correction_mode == "yaw_candidate" ? 0 : static_cast<int>(std::floor(cfg.tof_pose_correction_search_xy_m / cfg.tof_pose_correction_search_xy_step_m + 1e-9));
        int yaw_steps = static_cast<int>(std::floor(cfg.tof_pose_correction_search_yaw_deg / cfg.tof_pose_correction_search_yaw_step_deg + 1e-9));
        bool collect_yaw_curve = cfg.tof_pose_correction_yaw_residual_curve_debug && cfg.tof_pose_correction_mode == "yaw_candidate";
        for (int ix = -xy_steps; ix <= xy_steps; ++ix) {
            for (int iy = -xy_steps; iy <= xy_steps; ++iy) {
                for (int ia = -yaw_steps; ia <= yaw_steps; ++ia) {
                    Pose candidate = odom_pose;
                    candidate.x += ix * cfg.tof_pose_correction_search_xy_step_m;
                    candidate.y += iy * cfg.tof_pose_correction_search_xy_step_m;
                    candidate.yaw = wrap_pi(candidate.yaw + deg2rad(ia * cfg.tof_pose_correction_search_yaw_step_deg));
                    DetailedScore detail = score_pose_detail(candidate, obs, grid, cfg);
                    double score = detail.total_residual;
                    result.candidate_count++;
                    if (collect_yaw_curve) {
                        YawResidualCurvePoint point;
                        point.candidate_yaw = candidate.yaw;
                        point.yaw_offset_deg = ia * cfg.tof_pose_correction_search_yaw_step_deg;
                        point.total_residual = score;
                        point.front_obs_m = detail.front_obs_m;
                        point.front_expected_m = detail.front_expected_m;
                        point.front_residual_m = detail.front_residual_m;
                        point.left_obs_m = detail.left_obs_m;
                        point.left_expected_m = detail.left_expected_m;
                        point.left_residual_m = detail.left_residual_m;
                        point.right_obs_m = detail.right_obs_m;
                        point.right_expected_m = detail.right_expected_m;
                        point.right_residual_m = detail.right_residual_m;
                        point.used_tof = detail.used_tof;
                        point.valid_candidate = detail.used_tof >= cfg.tof_pose_correction_min_valid_tof && score < 1e8;
                        point.reason = point.valid_candidate ? (score <= cfg.tof_pose_correction_max_residual_m ? "candidate" : "residual_high") : "insufficient_tof";
                        result.yaw_curve.push_back(point);
                    }
                    if (score < best) {
                        third = second;
                        third_pose = second_pose;
                        second = best;
                        second_pose = best_pose;
                        best = score;
                        best_pose = candidate;
                        best_front = detail.front_expected_m;
                        best_left = detail.left_expected_m;
                        best_right = detail.right_expected_m;
                    } else if (score < second) {
                        third = second;
                        third_pose = second_pose;
                        second = score;
                        second_pose = candidate;
                    } else if (score < third) {
                        third = score;
                        third_pose = candidate;
                    }
                }
            }
        }

        result.best_residual = best;
        result.second_residual = second < 1e8 ? second : best;
        result.third_residual = third < 1e8 ? third : result.second_residual;
        result.corrected_pose = best_pose;
        result.dx = best_pose.x - odom_pose.x;
        result.dy = best_pose.y - odom_pose.y;
        result.dyaw = wrap_pi(best_pose.yaw - odom_pose.yaw);
        result.best_front_expected_m = best_front;
        result.best_left_expected_m = best_left;
        result.best_right_expected_m = best_right;
        result.best_yaw = best_pose.yaw;
        result.second_yaw = second_pose.yaw;
        result.third_yaw = third_pose.yaw;
        result.best_second_margin = result.second_residual - result.best_residual;
        result.second_third_margin = result.third_residual - result.second_residual;
        result.best_second_yaw_gap_deg = rad2deg(std::fabs(wrap_pi(result.best_yaw - result.second_yaw)));
        result.best_third_yaw_gap_deg = rad2deg(std::fabs(wrap_pi(result.best_yaw - result.third_yaw)));
        summarize_curve(result, cfg);
        result.accepted = false;
        if (best > cfg.tof_pose_correction_max_residual_m) result.reason = cfg.tof_pose_correction_mode + "_residual_high";
        else if ((result.second_residual - result.best_residual) < cfg.tof_pose_correction_min_margin) {
            if (cfg.tof_pose_correction_mode == "yaw_candidate") {
                result.reason = result.best_second_yaw_gap_deg <= 2.0 ? "yaw_candidate_low_margin_nearby" : "yaw_candidate_low_margin_ambiguous";
            } else {
                result.reason = cfg.tof_pose_correction_mode + "_low_margin";
            }
        } else result.reason = cfg.tof_pose_correction_mode;
        return result;
    }

private:
    struct DetailedScore {
        double total_residual = 1e9;
        double front_obs_m = 0.0;
        double front_expected_m = 0.0;
        double front_residual_m = 0.0;
        double left_obs_m = 0.0;
        double left_expected_m = 0.0;
        double left_residual_m = 0.0;
        double right_obs_m = 0.0;
        double right_expected_m = 0.0;
        double right_residual_m = 0.0;
        int used_tof = 0;
    };

    static int usable_obs(const std::vector<TofObsForCorrection> &obs, const Config &cfg) {
        int n = 0;
        for (const auto &o : obs) {
            if (!o.hit && !o.free_only) continue;
            if (cfg.tof_extrinsics.find(o.id) == cfg.tof_extrinsics.end()) continue;
            n++;
        }
        return n;
    }

    static double rad2deg(double r) { return r * 180.0 / kPi; }

    static double huber(double e) {
        constexpr double k = 0.10;
        return e <= k ? e : k + std::sqrt(e - k) * 0.05;
    }

    static void set_detail(const std::string &id, double obs_m, double expected_m, double residual_m, DetailedScore &detail) {
        if (id == "front") {
            detail.front_obs_m = obs_m;
            detail.front_expected_m = expected_m;
            detail.front_residual_m = residual_m;
        } else if (id == "left") {
            detail.left_obs_m = obs_m;
            detail.left_expected_m = expected_m;
            detail.left_residual_m = residual_m;
        } else if (id == "right") {
            detail.right_obs_m = obs_m;
            detail.right_expected_m = expected_m;
            detail.right_residual_m = residual_m;
        }
    }

    static DetailedScore score_pose_detail(const Pose &pose, const std::vector<TofObsForCorrection> &obs, const ChunkedGrid &grid, const Config &cfg) {
        DetailedScore detail;
        double sum = 0.0;
        double wsum = 0.0;
        double step_m = std::max(0.005, cfg.resolution_m * 0.5);
        for (const auto &o : obs) {
            if (!o.hit && !o.free_only) continue;
            auto ext_it = cfg.tof_extrinsics.find(o.id);
            if (ext_it == cfg.tof_extrinsics.end()) continue;
            double expected = grid.ray_cast_expected_range(pose, ext_it->second, cfg.tof_max_range_m, step_m, cfg);
            double obs_m = o.hit ? o.range_m : cfg.tof_max_range_m;
            double residual = 0.0;
            double weight = 1.0;
            if (o.hit) {
                residual = std::fabs(o.range_m - expected);
                weight = std::max(0.05, confidence_weight(o.confidence));
            } else {
                residual = expected >= (cfg.tof_max_range_m - cfg.resolution_m) ? 0.0 : (cfg.tof_max_range_m - expected);
                weight = 0.5;
            }
            set_detail(o.id, obs_m, expected, residual, detail);
            sum += weight * huber(residual);
            wsum += weight;
            detail.used_tof++;
        }
        detail.total_residual = wsum > 0.0 ? sum / wsum : 1e9;
        return detail;
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

    static void summarize_curve(PoseCorrectionResult &result, const Config &cfg) {
        if (result.yaw_curve.empty()) return;
        std::vector<double> residuals;
        residuals.reserve(result.yaw_curve.size());
        double sum = 0.0;
        for (const auto &p : result.yaw_curve) {
            residuals.push_back(p.total_residual);
            sum += p.total_residual;
        }
        std::sort(residuals.begin(), residuals.end());
        result.curve_min_residual = residuals.front();
        result.curve_mean_residual = sum / static_cast<double>(residuals.size());
        result.curve_p25_residual = percentile_sorted(residuals, 0.25);
        result.curve_p50_residual = percentile_sorted(residuals, 0.50);
        result.curve_p75_residual = percentile_sorted(residuals, 0.75);
        result.curve_sharpness = result.curve_p50_residual - result.curve_min_residual;
        result.flat_curve = result.curve_sharpness < 0.02;
        double best_offset_deg = rad2deg(result.dyaw);
        result.best_at_search_edge = std::fabs(std::fabs(best_offset_deg) - cfg.tof_pose_correction_search_yaw_deg) <= (cfg.tof_pose_correction_search_yaw_step_deg * 0.5 + 1e-9);

        std::vector<double> local_minima_yaw;
        for (size_t i = 0; i < result.yaw_curve.size(); ++i) {
            const auto &cur = result.yaw_curve[i];
            bool lower_prev = i == 0 || cur.total_residual <= result.yaw_curve[i - 1].total_residual;
            bool lower_next = i + 1 == result.yaw_curve.size() || cur.total_residual <= result.yaw_curve[i + 1].total_residual;
            if (lower_prev && lower_next && cur.total_residual <= result.best_residual + 0.02) {
                if (local_minima_yaw.empty() || std::fabs(cur.yaw_offset_deg - local_minima_yaw.back()) > 2.0) {
                    local_minima_yaw.push_back(cur.yaw_offset_deg);
                }
            }
        }
        result.num_local_minima = static_cast<int>(local_minima_yaw.size());
        result.multimodal = result.num_local_minima >= 2;
    }

    const Config &cfg_;
};

} // namespace robot_slamd
