#pragma once

#include "robot_slamd/localization/sparse_tof/se2_pose_transform.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_grid_ray_traversal.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace robot_slamd {

enum class SparseTofCorrectionMode { Rejected, YawOnly, FullSE2 };

enum class SparseTofMatcherFault {
    None,
    InvalidConfig,
    ReferenceMapUnavailable,
    BundleEmpty,
    BundleTimestampInvalid,
    BundleNotMonotonic,
    InsufficientValidRays,
    InsufficientHitRays,
    InsufficientKnownCells,
    KnownRatioTooLow,
    CandidateBudgetExceeded,
    CandidateEvaluationFailed,
    BestScoreTooLow,
    ScoreMarginTooSmall,
    AmbiguousMatch,
    YawUnobservable,
    CorrectionOutOfBounds,
    NonFiniteResult
};

inline std::string to_string(SparseTofCorrectionMode mode) {
    switch (mode) {
    case SparseTofCorrectionMode::Rejected: return "rejected";
    case SparseTofCorrectionMode::YawOnly: return "yaw_only";
    case SparseTofCorrectionMode::FullSE2: return "full_se2";
    }
    return "unknown";
}

inline std::string to_string(SparseTofMatcherFault fault) {
    switch (fault) {
    case SparseTofMatcherFault::None: return "none";
    case SparseTofMatcherFault::InvalidConfig: return "invalid_config";
    case SparseTofMatcherFault::ReferenceMapUnavailable: return "reference_map_unavailable";
    case SparseTofMatcherFault::BundleEmpty: return "bundle_empty";
    case SparseTofMatcherFault::BundleTimestampInvalid: return "bundle_timestamp_invalid";
    case SparseTofMatcherFault::BundleNotMonotonic: return "bundle_not_monotonic";
    case SparseTofMatcherFault::InsufficientValidRays: return "insufficient_valid_rays";
    case SparseTofMatcherFault::InsufficientHitRays: return "insufficient_hit_rays";
    case SparseTofMatcherFault::InsufficientKnownCells: return "insufficient_known_cells";
    case SparseTofMatcherFault::KnownRatioTooLow: return "known_ratio_too_low";
    case SparseTofMatcherFault::CandidateBudgetExceeded: return "candidate_budget_exceeded";
    case SparseTofMatcherFault::CandidateEvaluationFailed: return "candidate_evaluation_failed";
    case SparseTofMatcherFault::BestScoreTooLow: return "best_score_too_low";
    case SparseTofMatcherFault::ScoreMarginTooSmall: return "score_margin_too_small";
    case SparseTofMatcherFault::AmbiguousMatch: return "ambiguous_match";
    case SparseTofMatcherFault::YawUnobservable: return "yaw_unobservable";
    case SparseTofMatcherFault::CorrectionOutOfBounds: return "correction_out_of_bounds";
    case SparseTofMatcherFault::NonFiniteResult: return "non_finite_result";
    }
    return "unknown";
}

struct SparseTofMatchObservation {
    double timestamp_s = 0.0;
    RobotPose2D odom_pose;
    std::array<SparseTofRayObservation, 3> rays;
    std::size_t ray_count = 0;
};

struct SparseTofObservationBundle {
    std::vector<SparseTofMatchObservation> observations;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    RobotPose2D first_odom_pose;
    RobotPose2D last_odom_pose;
    std::size_t valid_ray_count = 0;
    std::size_t hit_ray_count = 0;
    std::size_t no_return_ray_count = 0;

    void add(const SparseTofMatchObservation &observation) {
        if (observations.empty()) {
            start_time_s = observation.timestamp_s;
            first_odom_pose = observation.odom_pose;
        }
        observations.push_back(observation);
        end_time_s = observation.timestamp_s;
        last_odom_pose = observation.odom_pose;
        for (std::size_t i = 0; i < observation.ray_count && i < 3U; ++i) {
            const auto &ray = observation.rays[i];
            if (ray.return_kind == ScalarTofReturnKind::Hit) {
                valid_ray_count++;
                hit_ray_count++;
            } else if (ray.return_kind == ScalarTofReturnKind::NoReturn) {
                valid_ray_count++;
                no_return_ray_count++;
            }
        }
    }
};

struct SparseTofMatchScoreConfig {
    std::int32_t free_path_matches_free_reward = 4;
    std::int32_t free_path_hits_occupied_penalty = -20;
    std::int32_t hit_endpoint_matches_occupied_reward = 30;
    std::int32_t hit_endpoint_matches_free_penalty = -30;
    std::int32_t uncertain_cell_weight = 0;
    std::int32_t unknown_cell_weight = -1;
    std::size_t path_sampling_stride_cells = 1;
    std::size_t minimum_known_cell_count = 2;
    double minimum_known_cell_ratio = 0.10;
    std::size_t minimum_valid_ray_count = 2;
    std::size_t minimum_hit_ray_count = 1;
    double minimum_normalized_score = 0.10;
    double minimum_score_margin = 0.02;
    double minimum_observability_contrast = 0.01;
};

struct SparseTofLocalSearchConfig {
    double coarse_translation_window_m = 0.15;
    double coarse_translation_step_m = 0.05;
    double coarse_yaw_window_rad = 0.20;
    double coarse_yaw_step_rad = 0.05;
    std::size_t coarse_top_k = 5;
    double fine_translation_window_m = 0.04;
    double fine_translation_step_m = 0.02;
    double fine_yaw_window_rad = 0.04;
    double fine_yaw_step_rad = 0.01;
    std::size_t maximum_candidate_count = 20000;
};

struct SparseTofLocalScanMatcherConfig {
    SparseOccupancyGridConfig grid;
    SparseTofMatchScoreConfig score;
    SparseTofLocalSearchConfig search;
    double maximum_correction_translation_m = 0.35;
    double maximum_correction_yaw_rad = 0.35;
};

struct SparseTofCandidateScore {
    RobotPose2D map_from_odom;
    RobotPose2D correction;
    std::int64_t raw_score = 0;
    double normalized_score = 0.0;
    std::size_t known_cell_count = 0;
    std::size_t unknown_cell_count = 0;
    std::size_t evaluated_cell_count = 0;
    std::size_t evaluated_ray_count = 0;
    std::size_t hit_endpoint_agreement_count = 0;
    std::size_t free_path_contradiction_count = 0;
    std::size_t generation_index = 0;
};

struct SparseTofMatchResult {
    bool accepted = false;
    SparseTofCorrectionMode correction_mode = SparseTofCorrectionMode::Rejected;
    SparseTofMatcherFault fault = SparseTofMatcherFault::None;
    std::string message;
    SparseTofCandidateScore best;
    SparseTofCandidateScore runner_up;
    double score_margin = 0.0;
    bool x_observable = false;
    bool y_observable = false;
    bool yaw_observable = false;
    double x_score_contrast = 0.0;
    double y_score_contrast = 0.0;
    double yaw_score_contrast = 0.0;
    std::size_t candidate_count = 0;
    bool bounded_search_verified = false;
    bool reference_map_frozen = true;
};

class SparseTofLocalScanMatcher {
public:
    explicit SparseTofLocalScanMatcher(SparseTofLocalScanMatcherConfig config = {})
        : config_(config) {}

    bool valid() const {
        return sparse_grid_config_valid(config_.grid) && valid_score_config() &&
               valid_search_config() &&
               std::isfinite(config_.maximum_correction_translation_m) &&
               config_.maximum_correction_translation_m >= 0.0 &&
               std::isfinite(config_.maximum_correction_yaw_rad) &&
               config_.maximum_correction_yaw_rad >= 0.0;
    }

    SparseTofMatchResult match(const SparseOccupancyGridSnapshot &reference_map,
                               const SparseTofObservationBundle &bundle,
                               const RobotPose2D &current_map_from_odom) const {
        SparseTofMatchResult result;
        if (!valid() || !se2_pose_finite(current_map_from_odom)) {
            return reject(result, SparseTofMatcherFault::InvalidConfig,
                          "sparse_tof_match_invalid_config");
        }
        if (reference_map.cells.empty()) {
            return reject(result, SparseTofMatcherFault::ReferenceMapUnavailable,
                          "sparse_tof_match_reference_unavailable");
        }
        const auto bundle_fault = validate_bundle(bundle);
        if (bundle_fault != SparseTofMatcherFault::None) {
            return reject(result, bundle_fault, "sparse_tof_match_bundle_rejected");
        }
        std::vector<SparseTofCandidateScore> candidates;
        if (!generate_candidates(current_map_from_odom, candidates)) {
            return reject(result, SparseTofMatcherFault::CandidateBudgetExceeded,
                          "sparse_tof_match_candidate_budget");
        }
        result.candidate_count = candidates.size();
        result.bounded_search_verified = candidates.size() <= config_.search.maximum_candidate_count;
        for (auto &candidate : candidates) {
            if (!evaluate(reference_map, bundle, candidate)) {
                return reject(result, SparseTofMatcherFault::CandidateEvaluationFailed,
                              "sparse_tof_match_candidate_eval_failed");
            }
        }
        std::sort(candidates.begin(), candidates.end(), better_candidate);
        result.best = candidates.front();
        result.runner_up = find_runner_up(candidates, result.best);
        result.score_margin = result.best.normalized_score - result.runner_up.normalized_score;
        if (result.best.known_cell_count < config_.score.minimum_known_cell_count) {
            return reject(result, SparseTofMatcherFault::InsufficientKnownCells,
                          "sparse_tof_match_insufficient_known_cells");
        }
        const double known_ratio = result.best.evaluated_cell_count == 0
            ? 0.0
            : static_cast<double>(result.best.known_cell_count) /
                  static_cast<double>(result.best.evaluated_cell_count);
        if (known_ratio < config_.score.minimum_known_cell_ratio) {
            return reject(result, SparseTofMatcherFault::KnownRatioTooLow,
                          "sparse_tof_match_known_ratio_low");
        }
        if (result.best.normalized_score < config_.score.minimum_normalized_score) {
            return reject(result, SparseTofMatcherFault::BestScoreTooLow,
                          "sparse_tof_match_best_score_low");
        }
        if (result.score_margin < config_.score.minimum_score_margin) {
            return reject(result, SparseTofMatcherFault::ScoreMarginTooSmall,
                          "sparse_tof_match_margin_small");
        }
        if (correction_too_large(result.best.correction)) {
            return reject(result, SparseTofMatcherFault::CorrectionOutOfBounds,
                          "sparse_tof_match_correction_out_of_bounds");
        }
        compute_observability(reference_map, bundle, result);
        if (!result.yaw_observable) {
            return reject(result, SparseTofMatcherFault::YawUnobservable,
                          "sparse_tof_match_yaw_unobservable");
        }
        result.correction_mode = (result.x_observable && result.y_observable)
            ? SparseTofCorrectionMode::FullSE2
            : SparseTofCorrectionMode::YawOnly;
        if (result.correction_mode == SparseTofCorrectionMode::YawOnly) {
            RobotPose2D yaw_only = current_map_from_odom;
            yaw_only.yaw_rad = result.best.map_from_odom.yaw_rad;
            result.best.map_from_odom = yaw_only;
            result.best.correction.x_m = 0.0;
            result.best.correction.y_m = 0.0;
        }
        result.accepted = true;
        result.fault = SparseTofMatcherFault::None;
        result.message = "sparse_tof_match_accepted";
        return result;
    }

    SparseTofCandidateScore score_candidate(const SparseOccupancyGridSnapshot &reference_map,
                                            const SparseTofObservationBundle &bundle,
                                            const RobotPose2D &candidate_map_from_odom) const {
        SparseTofCandidateScore score;
        score.map_from_odom = candidate_map_from_odom;
        evaluate(reference_map, bundle, score);
        return score;
    }

private:
    SparseTofLocalScanMatcherConfig config_;

    bool valid_score_config() const {
        return config_.score.path_sampling_stride_cells > 0 &&
               std::isfinite(config_.score.minimum_known_cell_ratio) &&
               config_.score.minimum_known_cell_ratio >= 0.0 &&
               config_.score.minimum_known_cell_ratio <= 1.0 &&
               std::isfinite(config_.score.minimum_normalized_score) &&
               std::isfinite(config_.score.minimum_score_margin) &&
               std::isfinite(config_.score.minimum_observability_contrast);
    }

    bool valid_search_config() const {
        const auto &s = config_.search;
        return std::isfinite(s.coarse_translation_window_m) && s.coarse_translation_window_m >= 0.0 &&
               std::isfinite(s.coarse_translation_step_m) && s.coarse_translation_step_m > 0.0 &&
               std::isfinite(s.coarse_yaw_window_rad) && s.coarse_yaw_window_rad >= 0.0 &&
               std::isfinite(s.coarse_yaw_step_rad) && s.coarse_yaw_step_rad > 0.0 &&
               s.coarse_top_k > 0 &&
               std::isfinite(s.fine_translation_window_m) && s.fine_translation_window_m >= 0.0 &&
               std::isfinite(s.fine_translation_step_m) && s.fine_translation_step_m > 0.0 &&
               std::isfinite(s.fine_yaw_window_rad) && s.fine_yaw_window_rad >= 0.0 &&
               std::isfinite(s.fine_yaw_step_rad) && s.fine_yaw_step_rad > 0.0 &&
               s.maximum_candidate_count > 0;
    }

    SparseTofMatcherFault validate_bundle(const SparseTofObservationBundle &bundle) const {
        if (bundle.observations.empty()) return SparseTofMatcherFault::BundleEmpty;
        if (bundle.valid_ray_count < config_.score.minimum_valid_ray_count) return SparseTofMatcherFault::InsufficientValidRays;
        if (bundle.hit_ray_count < config_.score.minimum_hit_ray_count) return SparseTofMatcherFault::InsufficientHitRays;
        double prev = -std::numeric_limits<double>::infinity();
        for (const auto &obs : bundle.observations) {
            if (!std::isfinite(obs.timestamp_s) || !se2_pose_finite(obs.odom_pose)) return SparseTofMatcherFault::BundleTimestampInvalid;
            if (obs.timestamp_s <= prev) return SparseTofMatcherFault::BundleNotMonotonic;
            prev = obs.timestamp_s;
        }
        return SparseTofMatcherFault::None;
    }

    bool generate_candidates(const RobotPose2D &base, std::vector<SparseTofCandidateScore> &out) const {
        std::vector<SparseTofCandidateScore> coarse;
        if (!generate_grid(base, config_.search.coarse_translation_window_m,
                           config_.search.coarse_translation_step_m,
                           config_.search.coarse_yaw_window_rad,
                           config_.search.coarse_yaw_step_rad, 0, coarse)) return false;
        std::sort(coarse.begin(), coarse.end(), [](const SparseTofCandidateScore &a, const SparseTofCandidateScore &b) {
            const double at = se2_squared_translation_norm(a.correction);
            const double bt = se2_squared_translation_norm(b.correction);
            if (at != bt) return at < bt;
            if (se2_abs_yaw(a.correction.yaw_rad) != se2_abs_yaw(b.correction.yaw_rad)) return se2_abs_yaw(a.correction.yaw_rad) < se2_abs_yaw(b.correction.yaw_rad);
            return a.generation_index < b.generation_index;
        });
        const std::size_t top = std::min(config_.search.coarse_top_k, coarse.size());
        out = coarse;
        for (std::size_t i = 0; i < top; ++i) {
            std::vector<SparseTofCandidateScore> fine;
            if (!generate_grid(coarse[i].map_from_odom,
                               config_.search.fine_translation_window_m,
                               config_.search.fine_translation_step_m,
                               config_.search.fine_yaw_window_rad,
                               config_.search.fine_yaw_step_rad,
                               out.size(), fine)) return false;
            out.insert(out.end(), fine.begin(), fine.end());
            if (out.size() > config_.search.maximum_candidate_count) return false;
        }
        return !out.empty();
    }

    bool generate_grid(const RobotPose2D &base, double t_window, double t_step,
                       double yaw_window, double yaw_step, std::size_t start_index,
                       std::vector<SparseTofCandidateScore> &out) const {
        const int tx_steps = static_cast<int>(std::floor(t_window / t_step));
        const int yaw_steps = static_cast<int>(std::floor(yaw_window / yaw_step));
        for (int ix = -tx_steps; ix <= tx_steps; ++ix) {
            for (int iy = -tx_steps; iy <= tx_steps; ++iy) {
                for (int it = -yaw_steps; it <= yaw_steps; ++it) {
                    if (out.size() + start_index >= config_.search.maximum_candidate_count) return false;
                    SparseTofCandidateScore c;
                    c.correction.x_m = static_cast<double>(ix) * t_step;
                    c.correction.y_m = static_cast<double>(iy) * t_step;
                    c.correction.yaw_rad = static_cast<double>(it) * yaw_step;
                    c.map_from_odom = se2_compose(c.correction, base);
                    c.generation_index = start_index + out.size();
                    out.push_back(c);
                }
            }
        }
        return true;
    }

    bool evaluate(const SparseOccupancyGridSnapshot &reference_map,
                  const SparseTofObservationBundle &bundle,
                  SparseTofCandidateScore &score) const {
        score.raw_score = 0;
        score.known_cell_count = 0;
        score.unknown_cell_count = 0;
        score.evaluated_cell_count = 0;
        score.evaluated_ray_count = 0;
        score.hit_endpoint_agreement_count = 0;
        score.free_path_contradiction_count = 0;
        std::int64_t max_abs = 0;
        for (const auto &obs : bundle.observations) {
            const RobotPose2D map_pose = se2_compose(score.map_from_odom, obs.odom_pose);
            for (std::size_t i = 0; i < obs.ray_count && i < 3U; ++i) {
                SparseTofRayObservation ray = transform_ray(obs, map_pose, obs.rays[i]);
                if (ray.return_kind != ScalarTofReturnKind::Hit && ray.return_kind != ScalarTofReturnKind::NoReturn) continue;
                const bool hit = ray.return_kind == ScalarTofReturnKind::Hit;
                const double range = hit ? ray.measured_range_m : ray.free_space_range_m;
                auto traversal = sparse_grid_traverse_ray(ray.sensor_origin_x_m, ray.sensor_origin_y_m, ray.ray_yaw_rad,
                                                          std::min(range, config_.grid.maximum_mapping_range_m),
                                                          config_.grid, true);
                if (!traversal.ok || traversal.cells.empty()) return false;
                score.evaluated_ray_count++;
                const std::size_t endpoint_index = hit ? traversal.cells.size() - 1U : traversal.cells.size();
                for (std::size_t j = 0; j < traversal.cells.size(); ++j) {
                    if (config_.score.path_sampling_stride_cells > 1 && j + 1U < traversal.cells.size() &&
                        (j % config_.score.path_sampling_stride_cells) != 0U) continue;
                    const bool endpoint = hit && j == endpoint_index;
                    const auto cls = sparse_map_cell_class(reference_map, config_.grid, traversal.cells[j]);
                    score_cell(cls, endpoint, score, max_abs);
                }
            }
        }
        score.normalized_score = max_abs == 0 ? 0.0 : static_cast<double>(score.raw_score) / static_cast<double>(max_abs);
        return std::isfinite(score.normalized_score);
    }

    SparseTofRayObservation transform_ray(const SparseTofMatchObservation &obs,
                                          const RobotPose2D &map_pose,
                                          const SparseTofRayObservation &ray) const {
        SparseTofRayObservation out = ray;
        RobotPose2D ray_pose;
        ray_pose.x_m = ray.sensor_origin_x_m;
        ray_pose.y_m = ray.sensor_origin_y_m;
        ray_pose.yaw_rad = ray.ray_yaw_rad;
        const RobotPose2D local_ray = se2_between(obs.odom_pose, ray_pose);
        const RobotPose2D mapped_ray = se2_compose(map_pose, local_ray);
        out.sensor_origin_x_m = mapped_ray.x_m;
        out.sensor_origin_y_m = mapped_ray.y_m;
        out.ray_yaw_rad = mapped_ray.yaw_rad;
        return out;
    }

    void score_cell(SparseMapCellClass cls, bool endpoint,
                    SparseTofCandidateScore &score, std::int64_t &max_abs) const {
        std::int32_t delta = 0;
        if (endpoint) {
            if (cls == SparseMapCellClass::Occupied) { delta = config_.score.hit_endpoint_matches_occupied_reward; score.hit_endpoint_agreement_count++; }
            else if (cls == SparseMapCellClass::Free) delta = config_.score.hit_endpoint_matches_free_penalty;
            else if (cls == SparseMapCellClass::Unknown) delta = config_.score.unknown_cell_weight;
            else delta = config_.score.uncertain_cell_weight;
        } else {
            if (cls == SparseMapCellClass::Free) delta = config_.score.free_path_matches_free_reward;
            else if (cls == SparseMapCellClass::Occupied) { delta = config_.score.free_path_hits_occupied_penalty; score.free_path_contradiction_count++; }
            else if (cls == SparseMapCellClass::Unknown) delta = config_.score.unknown_cell_weight;
            else delta = config_.score.uncertain_cell_weight;
        }
        if (cls == SparseMapCellClass::Unknown) score.unknown_cell_count++; else score.known_cell_count++;
        score.evaluated_cell_count++;
        score.raw_score += delta;
        max_abs += std::max<std::int32_t>(1, std::abs(delta));
    }

    static bool better_candidate(const SparseTofCandidateScore &a, const SparseTofCandidateScore &b) {
        if (a.normalized_score != b.normalized_score) return a.normalized_score > b.normalized_score;
        if (a.raw_score != b.raw_score) return a.raw_score > b.raw_score;
        if (a.known_cell_count != b.known_cell_count) return a.known_cell_count > b.known_cell_count;
        if (a.free_path_contradiction_count != b.free_path_contradiction_count) return a.free_path_contradiction_count < b.free_path_contradiction_count;
        const double amag = se2_squared_translation_norm(a.correction);
        const double bmag = se2_squared_translation_norm(b.correction);
        if (amag != bmag) return amag < bmag;
        if (se2_abs_yaw(a.correction.yaw_rad) != se2_abs_yaw(b.correction.yaw_rad)) return se2_abs_yaw(a.correction.yaw_rad) < se2_abs_yaw(b.correction.yaw_rad);
        if (a.correction.x_m != b.correction.x_m) return a.correction.x_m < b.correction.x_m;
        if (a.correction.y_m != b.correction.y_m) return a.correction.y_m < b.correction.y_m;
        if (a.correction.yaw_rad != b.correction.yaw_rad) return a.correction.yaw_rad < b.correction.yaw_rad;
        return a.generation_index < b.generation_index;
    }

    SparseTofCandidateScore find_runner_up(const std::vector<SparseTofCandidateScore> &candidates,
                                           const SparseTofCandidateScore &best) const {
        const double exclude_t = std::max(config_.search.fine_translation_step_m, 1e-9);
        const double exclude_yaw = std::max(config_.search.fine_yaw_step_rad, 1e-9);
        for (const auto &candidate : candidates) {
            const RobotPose2D diff = se2_between(best.map_from_odom, candidate.map_from_odom);
            if (std::sqrt(se2_squared_translation_norm(diff)) > exclude_t || se2_abs_yaw(diff.yaw_rad) > exclude_yaw) return candidate;
        }
        return candidates.size() > 1 ? candidates[1] : candidates.front();
    }

    void compute_observability(const SparseOccupancyGridSnapshot &reference_map,
                               const SparseTofObservationBundle &bundle,
                               SparseTofMatchResult &result) const {
        const double step_t = config_.search.fine_translation_step_m;
        const double step_yaw = config_.search.fine_yaw_step_rad;
        result.x_score_contrast = axis_contrast(reference_map, bundle, result.best.map_from_odom, step_t, 0.0, 0.0);
        result.y_score_contrast = axis_contrast(reference_map, bundle, result.best.map_from_odom, 0.0, step_t, 0.0);
        result.yaw_score_contrast = axis_contrast(reference_map, bundle, result.best.map_from_odom, 0.0, 0.0, step_yaw);
        const double threshold = config_.score.minimum_observability_contrast;
        result.x_observable = result.x_score_contrast >= threshold;
        result.y_observable = result.y_score_contrast >= threshold;
        result.yaw_observable = result.yaw_score_contrast >= threshold;
    }

    double axis_contrast(const SparseOccupancyGridSnapshot &reference_map,
                         const SparseTofObservationBundle &bundle,
                         const RobotPose2D &pose, double dx, double dy, double dyaw) const {
        SparseTofCandidateScore base;
        SparseTofCandidateScore plus;
        SparseTofCandidateScore minus;
        base.map_from_odom = pose;
        RobotPose2D delta_plus; delta_plus.x_m = dx; delta_plus.y_m = dy; delta_plus.yaw_rad = dyaw;
        RobotPose2D delta_minus; delta_minus.x_m = -dx; delta_minus.y_m = -dy; delta_minus.yaw_rad = -dyaw;
        plus.map_from_odom = se2_compose(delta_plus, pose);
        minus.map_from_odom = se2_compose(delta_minus, pose);
        evaluate(reference_map, bundle, base);
        evaluate(reference_map, bundle, plus);
        evaluate(reference_map, bundle, minus);
        return std::max(0.0, base.normalized_score - std::max(plus.normalized_score, minus.normalized_score));
    }

    bool correction_too_large(const RobotPose2D &correction) const {
        return std::sqrt(se2_squared_translation_norm(correction)) > config_.maximum_correction_translation_m ||
               se2_abs_yaw(correction.yaw_rad) > config_.maximum_correction_yaw_rad;
    }

    SparseTofMatchResult reject(SparseTofMatchResult result, SparseTofMatcherFault fault,
                                const std::string &message) const {
        result.accepted = false;
        result.correction_mode = SparseTofCorrectionMode::Rejected;
        result.fault = fault;
        result.message = message;
        return result;
    }
};

} // namespace robot_slamd
