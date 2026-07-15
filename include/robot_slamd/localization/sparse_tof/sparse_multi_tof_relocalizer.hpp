#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace robot_slamd {

enum class SparseRelocalizationSearchMode {
    Global,
    LocalRecovery
};

enum class SparseRelocalizationStatus {
    NotRun,
    Accepted,
    RejectedInvalidInput,
    RejectedInsufficientInformation,
    RejectedUnknownDominated,
    RejectedContradictory,
    RejectedLowScore,
    RejectedLowMargin,
    RejectedFlatLandscape,
    RejectedMultimodal,
    RejectedCandidateBudget,
    RejectedRevisionChanged,
    Fault
};

inline const char *to_string(SparseRelocalizationStatus status) {
    switch (status) {
    case SparseRelocalizationStatus::NotRun: return "not_run";
    case SparseRelocalizationStatus::Accepted: return "accepted";
    case SparseRelocalizationStatus::RejectedInvalidInput: return "rejected_invalid_input";
    case SparseRelocalizationStatus::RejectedInsufficientInformation: return "rejected_insufficient_information";
    case SparseRelocalizationStatus::RejectedUnknownDominated: return "rejected_unknown_dominated";
    case SparseRelocalizationStatus::RejectedContradictory: return "rejected_contradictory";
    case SparseRelocalizationStatus::RejectedLowScore: return "rejected_low_score";
    case SparseRelocalizationStatus::RejectedLowMargin: return "rejected_low_margin";
    case SparseRelocalizationStatus::RejectedFlatLandscape: return "rejected_flat_landscape";
    case SparseRelocalizationStatus::RejectedMultimodal: return "rejected_multimodal";
    case SparseRelocalizationStatus::RejectedCandidateBudget: return "rejected_candidate_budget";
    case SparseRelocalizationStatus::RejectedRevisionChanged: return "rejected_revision_changed";
    case SparseRelocalizationStatus::Fault: return "fault";
    }
    return "unknown";
}

struct SparseMultiTofRelocalizerConfig {
    std::size_t coarse_max_scored_rays = 32;
    std::size_t refine_max_scored_rays = 96;
    std::size_t max_cells_per_ray = 256;
    std::size_t top_k = 8;
    std::size_t max_total_candidates = 15000;
    std::size_t coarse_free_cell_stride = 4;
    double coarse_yaw_step_rad = 0.2617993877991494;
    double refine_xy_step_m = 0.10;
    double refine_yaw_step_rad = 0.05235987755982989;
    double final_xy_step_m = 0.05;
    double final_yaw_step_rad = 0.017453292519943295;
    double local_xy_half_range_m = 0.40;
    double local_yaw_half_range_rad = 0.5235987755982988;
    double exclusion_xy_m = 0.25;
    double exclusion_yaw_rad = 0.2617993877991494;
    double minimum_normalized_score = 0.02;
    double minimum_score_margin = 0.02;
    double minimum_score_range = 0.015;
    double multimodal_max_score_drop = 0.03;
};

inline bool sparse_relocalizer_config_valid(
    const SparseMultiTofRelocalizerConfig &config) {
    return config.coarse_max_scored_rays > 0 &&
           config.refine_max_scored_rays >= config.coarse_max_scored_rays &&
           config.max_cells_per_ray > 0 &&
           config.top_k > 0 && config.top_k <= 32 &&
           config.max_total_candidates > 0 &&
           config.max_total_candidates <= 15000 &&
           config.coarse_free_cell_stride > 0 &&
           std::isfinite(config.coarse_yaw_step_rad) &&
           config.coarse_yaw_step_rad > 0.0 &&
           config.coarse_yaw_step_rad <= 3.14159265358979323846 &&
           std::isfinite(config.refine_xy_step_m) &&
           config.refine_xy_step_m > 0.0 &&
           std::isfinite(config.refine_yaw_step_rad) &&
           config.refine_yaw_step_rad > 0.0 &&
           config.refine_yaw_step_rad <= config.coarse_yaw_step_rad &&
           std::isfinite(config.final_xy_step_m) &&
           config.final_xy_step_m > 0.0 &&
           config.final_xy_step_m <= config.refine_xy_step_m &&
           std::isfinite(config.final_yaw_step_rad) &&
           config.final_yaw_step_rad > 0.0 &&
           config.final_yaw_step_rad <= config.refine_yaw_step_rad &&
           std::isfinite(config.local_xy_half_range_m) &&
           config.local_xy_half_range_m >= 0.0 &&
           std::isfinite(config.local_yaw_half_range_rad) &&
           config.local_yaw_half_range_rad >= 0.0 &&
           std::isfinite(config.exclusion_xy_m) &&
           config.exclusion_xy_m > 0.0 &&
           std::isfinite(config.exclusion_yaw_rad) &&
           config.exclusion_yaw_rad > 0.0 &&
           std::isfinite(config.minimum_normalized_score) &&
           std::isfinite(config.minimum_score_margin) &&
           config.minimum_score_margin >= 0.0 &&
           std::isfinite(config.minimum_score_range) &&
           config.minimum_score_range >= 0.0 &&
           std::isfinite(config.multimodal_max_score_drop) &&
           config.multimodal_max_score_drop >= 0.0;
}

struct SparseRelocalizationCandidate {
    MapFromOdom2D map_from_odom = identity_map_from_odom();
    SparseTofLocalMatchCandidateMetrics metrics;
};

struct SparseMultiTofRelocalizationResult {
    SparseRelocalizationStatus status = SparseRelocalizationStatus::NotRun;
    SparseRelocalizationSearchMode mode = SparseRelocalizationSearchMode::Global;
    bool executed = false;
    bool accepted = false;
    std::uint64_t bundle_id = 0;
    std::uint64_t reference_revision = 0;
    std::size_t evaluated_candidate_count = 0;
    std::size_t coarse_candidate_count = 0;
    std::size_t refine_candidate_count = 0;
    std::size_t final_candidate_count = 0;
    std::optional<SparseRelocalizationCandidate> best;
    std::optional<SparseRelocalizationCandidate> runner_up;
    double score_margin = 0.0;
    double runner_up_xy_separation_m = 0.0;
    double runner_up_yaw_separation_rad = 0.0;
    double score_min = 0.0;
    double score_max = 0.0;
    double score_range = 0.0;
    std::string reason = "relocalization_not_run";
};

class SparseMultiTofRelocalizer {
public:
    SparseMultiTofRelocalizationResult search_global(
        const SparseTofLocalMatchInput &input,
        const SparseMultiTofRelocalizerConfig &config) const {
        return search(input, config, SparseRelocalizationSearchMode::Global,
                      std::nullopt);
    }

    SparseMultiTofRelocalizationResult search_local(
        const SparseTofLocalMatchInput &input,
        const MapFromOdom2D &center,
        const SparseMultiTofRelocalizerConfig &config) const {
        return search(input, config,
                      SparseRelocalizationSearchMode::LocalRecovery, center);
    }

private:
    SparseTofLocalMatcher shared_scorer_;

    static constexpr double kPi = 3.14159265358979323846;

    static bool same_pose(const MapFromOdom2D &a, const MapFromOdom2D &b) {
        return std::fabs(a.map_T_odom.x_m - b.map_T_odom.x_m) <= 1e-12 &&
               std::fabs(a.map_T_odom.y_m - b.map_T_odom.y_m) <= 1e-12 &&
               std::fabs(sparse_slam_shortest_yaw_delta(
                   a.map_T_odom.yaw_rad, b.map_T_odom.yaw_rad)) <= 1e-12;
    }

    static bool better(const SparseRelocalizationCandidate &a,
                       const SparseRelocalizationCandidate &b) {
        if (a.metrics.normalized_score != b.metrics.normalized_score) {
            return a.metrics.normalized_score > b.metrics.normalized_score;
        }
        if (a.map_from_odom.map_T_odom.x_m != b.map_from_odom.map_T_odom.x_m) {
            return a.map_from_odom.map_T_odom.x_m <
                   b.map_from_odom.map_T_odom.x_m;
        }
        if (a.map_from_odom.map_T_odom.y_m != b.map_from_odom.map_T_odom.y_m) {
            return a.map_from_odom.map_T_odom.y_m <
                   b.map_from_odom.map_T_odom.y_m;
        }
        return a.map_from_odom.map_T_odom.yaw_rad <
               b.map_from_odom.map_T_odom.yaw_rad;
    }

    static bool independent(const SparseRelocalizationCandidate &a,
                            const SparseRelocalizationCandidate &b,
                            const SparseMultiTofRelocalizerConfig &config) {
        const double dx = a.map_from_odom.map_T_odom.x_m -
                          b.map_from_odom.map_T_odom.x_m;
        const double dy = a.map_from_odom.map_T_odom.y_m -
                          b.map_from_odom.map_T_odom.y_m;
        const double xy = std::hypot(dx, dy);
        const double yaw = std::fabs(sparse_slam_shortest_yaw_delta(
            a.map_from_odom.map_T_odom.yaw_rad,
            b.map_from_odom.map_T_odom.yaw_rad));
        return xy + 1e-12 >= config.exclusion_xy_m ||
               yaw + 1e-12 >= config.exclusion_yaw_rad;
    }

    static bool push_unique(std::vector<MapFromOdom2D> &poses,
                            const MapFromOdom2D &pose,
                            std::size_t budget) {
        if (std::any_of(poses.begin(), poses.end(),
                        [&pose](const auto &p) { return same_pose(p, pose); })) {
            return true;
        }
        if (poses.size() >= budget) return false;
        poses.push_back(pose);
        return true;
    }

    static std::vector<SparseRelocalizationCandidate> top_modes(
        std::vector<SparseRelocalizationCandidate> candidates,
        const SparseMultiTofRelocalizerConfig &config) {
        std::sort(candidates.begin(), candidates.end(), better);
        std::vector<SparseRelocalizationCandidate> out;
        for (const auto &candidate : candidates) {
            if (!candidate.metrics.valid) continue;
            const bool distinct = std::all_of(
                out.begin(), out.end(), [&candidate, &config](const auto &saved) {
                    return independent(candidate, saved, config);
                });
            if (distinct) out.push_back(candidate);
            if (out.size() >= config.top_k) break;
        }
        return out;
    }

    static bool append_global_coarse(
        const SparseTofLocalMatchInput &input,
        const SparseMultiTofRelocalizerConfig &config,
        std::vector<MapFromOdom2D> &poses) {
        std::size_t free_index = 0;
        for (const auto &cell : input.reference_map->cells()) {
            const auto query = input.reference_map->query_cell(cell.key);
            if (query.classification != SparseReferenceCellClass::Free) continue;
            if ((free_index++ % config.coarse_free_cell_stride) != 0U) continue;
            const double x = (static_cast<double>(cell.key.x) + 0.5) *
                             input.reference_map->resolution_m();
            const double y = (static_cast<double>(cell.key.y) + 0.5) *
                             input.reference_map->resolution_m();
            const std::size_t yaw_count = static_cast<std::size_t>(
                std::ceil((2.0 * kPi) / config.coarse_yaw_step_rad));
            for (std::size_t i = 0; i < yaw_count; ++i) {
                const double yaw = -kPi +
                    static_cast<double>(i) * (2.0 * kPi /
                                              static_cast<double>(yaw_count));
                if (!push_unique(poses, make_map_from_odom({x, y, yaw}),
                                 config.max_total_candidates)) {
                    return false;
                }
            }
        }
        return !poses.empty();
    }

    static bool append_local_coarse(
        const MapFromOdom2D &center,
        const SparseMultiTofRelocalizerConfig &config,
        std::vector<MapFromOdom2D> &poses) {
        const double xy_step = std::max(config.refine_xy_step_m * 2.0,
                                        config.final_xy_step_m);
        const double yaw_step = config.coarse_yaw_step_rad;
        for (double dx = -config.local_xy_half_range_m;
             dx <= config.local_xy_half_range_m + 1e-12; dx += xy_step) {
            for (double dy = -config.local_xy_half_range_m;
                 dy <= config.local_xy_half_range_m + 1e-12; dy += xy_step) {
                for (double dyaw = -config.local_yaw_half_range_rad;
                     dyaw <= config.local_yaw_half_range_rad + 1e-12;
                     dyaw += yaw_step) {
                    const auto pose = make_map_from_odom({
                        center.map_T_odom.x_m + dx,
                        center.map_T_odom.y_m + dy,
                        sparse_slam_normalize_yaw(
                            center.map_T_odom.yaw_rad + dyaw)});
                    if (!push_unique(poses, pose, config.max_total_candidates)) {
                        return false;
                    }
                }
            }
        }
        return !poses.empty();
    }

    static bool append_neighborhood(
        const std::vector<SparseRelocalizationCandidate> &centers,
        double xy_step, double yaw_step,
        std::size_t budget, std::vector<MapFromOdom2D> &poses) {
        for (const auto &center : centers) {
            for (int ix = -1; ix <= 1; ++ix) {
                for (int iy = -1; iy <= 1; ++iy) {
                    for (int iyaw = -1; iyaw <= 1; ++iyaw) {
                        const auto pose = make_map_from_odom({
                            center.map_from_odom.map_T_odom.x_m +
                                static_cast<double>(ix) * xy_step,
                            center.map_from_odom.map_T_odom.y_m +
                                static_cast<double>(iy) * xy_step,
                            sparse_slam_normalize_yaw(
                                center.map_from_odom.map_T_odom.yaw_rad +
                                static_cast<double>(iyaw) * yaw_step)});
                        if (!push_unique(poses, pose, budget)) return false;
                    }
                }
            }
        }
        return true;
    }

    bool evaluate(
        const SparseTofLocalMatchInput &input,
        const std::vector<MapFromOdom2D> &poses,
        std::size_t max_rays,
        std::vector<SparseRelocalizationCandidate> &all,
        SparseMultiTofRelocalizationResult &result) const {
        for (const auto &pose : poses) {
            SparseTofLocalMatchCandidateMetrics metrics;
            if (!shared_scorer_.score_absolute_candidate(
                    input, pose, max_rays, metrics)) {
                return false;
            }
            all.push_back({pose, metrics});
            result.evaluated_candidate_count++;
        }
        return true;
    }

    static SparseMultiTofRelocalizationResult reject(
        SparseMultiTofRelocalizationResult result,
        SparseRelocalizationStatus status, const std::string &reason) {
        result.status = status;
        result.accepted = false;
        result.reason = reason;
        return result;
    }

    SparseMultiTofRelocalizationResult search(
        const SparseTofLocalMatchInput &input,
        const SparseMultiTofRelocalizerConfig &config,
        SparseRelocalizationSearchMode mode,
        const std::optional<MapFromOdom2D> &center) const {
        SparseMultiTofRelocalizationResult result;
        result.mode = mode;
        result.bundle_id = input.bundle_id;
        result.reference_revision = input.expected_reference_map_revision;
        if (!sparse_relocalizer_config_valid(config) ||
            !input.frozen_bundle || !input.frozen_bundle->available() ||
            !input.reference_map || !input.reference_map->valid()) {
            return reject(result, SparseRelocalizationStatus::RejectedInvalidInput,
                          "relocalization_input_invalid");
        }
        if (input.reference_map->revision() !=
                input.expected_reference_map_revision ||
            input.current_runtime_map_revision !=
                input.expected_reference_map_revision) {
            return reject(result,
                          SparseRelocalizationStatus::RejectedRevisionChanged,
                          "relocalization_reference_revision_changed");
        }
        if (mode == SparseRelocalizationSearchMode::LocalRecovery &&
            (!center || !sparse_slam_frame_pose_valid(*center))) {
            return reject(result, SparseRelocalizationStatus::RejectedInvalidInput,
                          "relocalization_local_center_invalid");
        }
        result.executed = true;
        SparseTofLocalMatchInput scoring_input = input;
        scoring_input.config.max_cells_per_ray = config.max_cells_per_ray;

        std::vector<MapFromOdom2D> coarse_poses;
        const bool generated = mode == SparseRelocalizationSearchMode::Global
            ? append_global_coarse(input, config, coarse_poses)
            : append_local_coarse(*center, config, coarse_poses);
        if (!generated) {
            return reject(result,
                          coarse_poses.size() >= config.max_total_candidates
                              ? SparseRelocalizationStatus::RejectedCandidateBudget
                              : SparseRelocalizationStatus::RejectedInvalidInput,
                          coarse_poses.size() >= config.max_total_candidates
                              ? "relocalization_coarse_candidate_budget"
                              : "relocalization_no_coarse_candidates");
        }
        result.coarse_candidate_count = coarse_poses.size();

        std::vector<SparseRelocalizationCandidate> all;
        all.reserve(config.max_total_candidates);
        if (!evaluate(scoring_input, coarse_poses,
                      config.coarse_max_scored_rays, all, result)) {
            return reject(result,
                          SparseRelocalizationStatus::RejectedCandidateBudget,
                          "relocalization_ray_traversal_budget");
        }
        auto modes = top_modes(all, config);
        if (modes.empty()) return reject_invalid_candidates(result, all);

        std::vector<MapFromOdom2D> refine_poses;
        if (!append_neighborhood(modes, config.refine_xy_step_m,
                                 config.refine_yaw_step_rad,
                                 config.max_total_candidates -
                                     result.evaluated_candidate_count,
                                 refine_poses)) {
            return reject(result,
                          SparseRelocalizationStatus::RejectedCandidateBudget,
                          "relocalization_refine_candidate_budget");
        }
        result.refine_candidate_count = refine_poses.size();
        if (!evaluate(scoring_input, refine_poses,
                      config.refine_max_scored_rays, all, result)) {
            return reject(result,
                          SparseRelocalizationStatus::RejectedCandidateBudget,
                          "relocalization_refine_ray_budget");
        }

        modes = top_modes(all, config);
        std::vector<MapFromOdom2D> final_poses;
        if (!append_neighborhood(modes, config.final_xy_step_m,
                                 config.final_yaw_step_rad,
                                 config.max_total_candidates -
                                     result.evaluated_candidate_count,
                                 final_poses)) {
            return reject(result,
                          SparseRelocalizationStatus::RejectedCandidateBudget,
                          "relocalization_final_candidate_budget");
        }
        result.final_candidate_count = final_poses.size();
        if (!evaluate(scoring_input, final_poses,
                      config.refine_max_scored_rays, all, result)) {
            return reject(result,
                          SparseRelocalizationStatus::RejectedCandidateBudget,
                          "relocalization_final_ray_budget");
        }
        return select_and_gate(std::move(result), all, config);
    }

    static SparseMultiTofRelocalizationResult reject_invalid_candidates(
        SparseMultiTofRelocalizationResult result,
        const std::vector<SparseRelocalizationCandidate> &all) {
        bool enough_information = false;
        bool unknown_ok = false;
        bool contradiction_ok = false;
        for (const auto &candidate : all) {
            enough_information = enough_information ||
                (candidate.metrics.used_ray_count > 0 &&
                 candidate.metrics.known_cell_count > 0);
            unknown_ok = unknown_ok || candidate.metrics.unknown_ratio < 1.0;
            contradiction_ok = contradiction_ok ||
                candidate.metrics.contradiction_ratio < 1.0;
        }
        if (!enough_information) {
            return reject(result,
                SparseRelocalizationStatus::RejectedInsufficientInformation,
                "relocalization_insufficient_information");
        }
        if (!unknown_ok) {
            return reject(result,
                SparseRelocalizationStatus::RejectedUnknownDominated,
                "relocalization_unknown_dominated");
        }
        if (!contradiction_ok) {
            return reject(result,
                SparseRelocalizationStatus::RejectedContradictory,
                "relocalization_contradictory");
        }
        return reject(result,
            SparseRelocalizationStatus::RejectedInsufficientInformation,
            "relocalization_no_valid_candidate");
    }

    static SparseMultiTofRelocalizationResult select_and_gate(
        SparseMultiTofRelocalizationResult result,
        const std::vector<SparseRelocalizationCandidate> &all,
        const SparseMultiTofRelocalizerConfig &config) {
        std::vector<SparseRelocalizationCandidate> valid;
        for (const auto &candidate : all) {
            if (candidate.metrics.valid) valid.push_back(candidate);
        }
        if (valid.empty()) return reject_invalid_candidates(result, all);
        std::sort(valid.begin(), valid.end(), better);
        result.best = valid.front();
        result.score_min = valid.front().metrics.normalized_score;
        result.score_max = valid.front().metrics.normalized_score;
        for (const auto &candidate : valid) {
            result.score_min = std::min(
                result.score_min, candidate.metrics.normalized_score);
            result.score_max = std::max(
                result.score_max, candidate.metrics.normalized_score);
        }
        result.score_range = result.score_max - result.score_min;
        for (std::size_t i = 1; i < valid.size(); ++i) {
            if (independent(valid.front(), valid[i], config)) {
                result.runner_up = valid[i];
                const double dx = valid.front().map_from_odom.map_T_odom.x_m -
                                  valid[i].map_from_odom.map_T_odom.x_m;
                const double dy = valid.front().map_from_odom.map_T_odom.y_m -
                                  valid[i].map_from_odom.map_T_odom.y_m;
                result.runner_up_xy_separation_m = std::hypot(dx, dy);
                result.runner_up_yaw_separation_rad = std::fabs(
                    sparse_slam_shortest_yaw_delta(
                        valid.front().map_from_odom.map_T_odom.yaw_rad,
                        valid[i].map_from_odom.map_T_odom.yaw_rad));
                result.score_margin =
                    valid.front().metrics.normalized_score -
                    valid[i].metrics.normalized_score;
                break;
            }
        }
        if (result.score_range < config.minimum_score_range) {
            return reject(result,
                SparseRelocalizationStatus::RejectedFlatLandscape,
                "relocalization_flat_landscape");
        }
        if (result.best->metrics.normalized_score <
            config.minimum_normalized_score) {
            return reject(result, SparseRelocalizationStatus::RejectedLowScore,
                          "relocalization_score_below_minimum");
        }
        if (!result.runner_up ||
            result.score_margin < config.minimum_score_margin) {
            return reject(result, SparseRelocalizationStatus::RejectedLowMargin,
                          "relocalization_margin_below_minimum");
        }
        if (result.score_margin <= config.multimodal_max_score_drop) {
            return reject(result, SparseRelocalizationStatus::RejectedMultimodal,
                          "relocalization_independent_modes_ambiguous");
        }
        result.status = SparseRelocalizationStatus::Accepted;
        result.accepted = true;
        result.reason = "relocalization_candidate_accepted";
        return result;
    }
};

} // namespace robot_slamd
