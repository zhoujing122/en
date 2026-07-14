#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_match_input_validator.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_grid_ray_traversal.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_observation_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace robot_slamd {

inline MapFromOdom2D propose_yaw_corrected_map_from_odom(
    const MapFromOdom2D &predicted,
    double delta_yaw_rad) {
    const RobotPose2D odom_frame_yaw_delta{0.0, 0.0, delta_yaw_rad};
    return make_map_from_odom(
        compose_robot_poses(predicted.map_T_odom, odom_frame_yaw_delta));
}

class SparseTofLocalMatcher {
public:
    SparseTofLocalMatchResult match(
        const SparseTofLocalMatchInput &input) const {
        SparseTofLocalMatchResult result = result_shell(input);
        const auto validation = validator_.validate(input);
        if (!validation.ready) {
            return reject(result, SparseTofLocalMatchStatus::RejectedInvalidInput,
                          validation.reason, false);
        }
        if (input.config.mode != SparseTofLocalMatchMode::YawOnly) {
            result.status = SparseTofLocalMatchStatus::NotImplemented;
            result.reason = "full_se2_not_implemented";
            return result;
        }
        result.matcher_executed = true;

        const auto rays = sampled_rays(input);
        if (rays.size() < input.config.min_used_rays) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedInsufficientInformation,
                          "local_match_insufficient_sampled_rays");
        }

        std::vector<double> coarse;
        if (!uniform_candidates(-input.config.max_abs_yaw_rad,
                                input.config.max_abs_yaw_rad,
                                input.config.coarse_yaw_step_rad,
                                input.config.max_coarse_candidates,
                                coarse) ||
            coarse.size() > input.config.max_total_candidates) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedCandidateBudget,
                          "local_match_coarse_candidate_budget");
        }
        result.coarse_candidate_count = coarse.size();
        if (!score_candidates(input, rays, coarse, result)) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedCandidateBudget,
                          "local_match_ray_traversal_budget");
        }

        const auto coarse_best = best_valid_index(result.candidate_metrics);
        if (!coarse_best.has_value()) {
            return reject_for_invalid_candidates(result, input.config);
        }

        std::vector<double> fine;
        const double fine_min = std::max(
            -input.config.max_abs_yaw_rad,
            result.candidate_metrics[*coarse_best].delta_yaw_rad -
                input.config.coarse_yaw_step_rad);
        const double fine_max = std::min(
            input.config.max_abs_yaw_rad,
            result.candidate_metrics[*coarse_best].delta_yaw_rad +
                input.config.coarse_yaw_step_rad);
        if (!uniform_candidates(fine_min, fine_max,
                                input.config.fine_yaw_step_rad,
                                input.config.max_fine_candidates, fine)) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedCandidateBudget,
                          "local_match_fine_candidate_budget");
        }
        erase_existing_candidates(result.candidate_metrics, fine);
        if (result.candidate_metrics.size() + fine.size() >
            input.config.max_total_candidates) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedCandidateBudget,
                          "local_match_total_candidate_budget");
        }
        result.fine_candidate_count = fine.size();
        if (!score_candidates(input, rays, fine, result)) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedCandidateBudget,
                          "local_match_ray_traversal_budget");
        }
        result.evaluated_candidate_count = result.candidate_metrics.size();
        return select_and_gate(input, std::move(result));
    }

private:
    struct RayRef {
        const ResolvedScalarTofObservationRoute *route = nullptr;
    };

    struct CandidateEvaluation {
        SparseTofLocalMatchCandidateMetrics metrics;
        bool traversal_budget_exceeded = false;
    };

    SparseTofLocalMatchInputValidator validator_;

    static SparseTofLocalMatchResult result_shell(
        const SparseTofLocalMatchInput &input) {
        SparseTofLocalMatchResult result;
        result.bundle_id = input.bundle_id;
        result.reference_map_revision = input.expected_reference_map_revision;
        result.mode = input.config.mode;
        result.predicted_map_from_odom = input.predicted_map_from_odom;
        return result;
    }

    static SparseTofLocalMatchResult reject(
        SparseTofLocalMatchResult result,
        SparseTofLocalMatchStatus status,
        const std::string &reason,
        bool executed = true) {
        result.status = status;
        result.matcher_executed = executed;
        result.accepted = false;
        result.proposal_ready = false;
        result.proposed_map_from_odom.reset();
        result.correction_delta.reset();
        result.reason = reason;
        result.evaluated_candidate_count = result.candidate_metrics.size();
        return result;
    }

    static bool same_candidate(double a, double b) {
        return std::fabs(a - b) <= 1e-12;
    }

    static bool uniform_candidates(double first, double last, double step,
                                   std::size_t budget,
                                   std::vector<double> &out) {
        out.clear();
        if (!std::isfinite(first) || !std::isfinite(last) ||
            !std::isfinite(step) || step <= 0.0 || first > last) {
            return false;
        }
        const std::size_t intervals = static_cast<std::size_t>(
            std::floor((last - first) / step + 1e-12));
        if (intervals + 2U > budget + 1U) {
            return false;
        }
        for (std::size_t i = 0; i <= intervals; ++i) {
            out.push_back(first + static_cast<double>(i) * step);
        }
        if (out.empty() || !same_candidate(out.back(), last)) {
            out.push_back(last);
        } else {
            out.back() = last;
        }
        return out.size() <= budget;
    }

    static std::vector<RayRef> sampled_rays(
        const SparseTofLocalMatchInput &input) {
        std::vector<RayRef> all;
        all.reserve(input.frozen_bundle->frames().size() * 3U);
        for (const auto &frame : input.frozen_bundle->frames()) {
            all.push_back({&frame.front});
            all.push_back({&frame.left});
            all.push_back({&frame.right});
        }
        if (all.size() <= input.config.max_scored_rays) {
            return all;
        }
        std::vector<RayRef> selected;
        selected.reserve(input.config.max_scored_rays);
        for (std::size_t i = 0; i < input.config.max_scored_rays; ++i) {
            const std::size_t index =
                ((2U * i + 1U) * all.size()) /
                (2U * input.config.max_scored_rays);
            selected.push_back(all[std::min(index, all.size() - 1U)]);
        }
        return selected;
    }

    static SparseOccupancyGridConfig traversal_config(
        const SparseTofLocalMatchInput &input) {
        SparseOccupancyGridConfig config;
        config.resolution_m = input.reference_map->resolution_m();
        config.free_threshold = input.reference_map->free_threshold();
        config.occupied_threshold = input.reference_map->occupied_threshold();
        config.minimum_mapping_range_m = 0.001;
        config.maximum_mapping_range_m =
            std::max(12.0, input.config.no_return_free_space_range_m) + 1e-9;
        config.maximum_ray_cells = input.config.max_cells_per_ray;
        return config;
    }

    static SparseTofObservationBuilder observation_builder(
        const SparseTofLocalMatchInput &input) {
        SparseTofObservationBuilderOptions options;
        options.protocol_min_range_m = 0.001;
        options.protocol_max_range_m =
            std::max(12.0, input.config.no_return_free_space_range_m) + 1e-9;
        options.mapping_min_range_m = 0.001;
        options.mapping_max_range_m = options.protocol_max_range_m;
        options.no_return_free_space_range_m =
            input.config.no_return_free_space_range_m;
        options.mapping_min_confidence = 1;
        options.max_future_timestamp_skew_s = 0.0;
        options.stale_after_s = input.config.max_bundle_duration_s + 1.0;
        return SparseTofObservationBuilder(options);
    }

    static void score_free_cell(
        SparseTofLocalMatchCandidateMetrics &m,
        const SparseReferenceCellQuery &query,
        const SparseTofLocalMatchConfig &config) {
        m.traversed_cell_count++;
        if (!query.found ||
            query.classification == SparseReferenceCellClass::Unknown ||
            query.classification == SparseReferenceCellClass::Uncertain) {
            m.unknown_cell_count++;
            return;
        }
        m.known_cell_count++;
        if (query.classification == SparseReferenceCellClass::Free) {
            m.free_agreement_count++;
            m.raw_score += config.free_agreement_weight;
        } else {
            m.contradiction_count++;
            m.raw_score += config.free_contradiction_weight;
        }
    }

    static void score_hit_endpoint(
        SparseTofLocalMatchCandidateMetrics &m,
        const SparseReferenceCellQuery &query,
        const SparseTofLocalMatchConfig &config) {
        m.traversed_cell_count++;
        if (!query.found ||
            query.classification == SparseReferenceCellClass::Unknown ||
            query.classification == SparseReferenceCellClass::Uncertain) {
            m.unknown_cell_count++;
            return;
        }
        m.known_cell_count++;
        if (query.classification == SparseReferenceCellClass::Occupied) {
            m.occupied_endpoint_agreement_count++;
            m.raw_score += config.occupied_endpoint_agreement_weight;
        } else {
            m.contradiction_count++;
            m.raw_score += config.occupied_endpoint_contradiction_weight;
        }
    }

    static CandidateEvaluation evaluate_candidate(
        const SparseTofLocalMatchInput &input,
        const std::vector<RayRef> &rays,
        double delta_yaw_rad) {
        CandidateEvaluation out;
        auto &m = out.metrics;
        m.delta_yaw_rad = delta_yaw_rad;
        const auto candidate = propose_yaw_corrected_map_from_odom(
            input.predicted_map_from_odom, delta_yaw_rad);
        m.candidate_yaw_rad = candidate.map_T_odom.yaw_rad;
        const auto grid_config = traversal_config(input);
        const auto builder = observation_builder(input);

        for (const auto &ray : rays) {
            const auto &route = *ray.route;
            if (!route.present ||
                route.return_kind == ScalarTofReturnKind::Invalid ||
                route.return_kind == ScalarTofReturnKind::Unspecified ||
                route.frame.confidence == 0) {
                m.ignored_ray_count++;
                continue;
            }
            const MapPose2D candidate_base =
                compose(candidate, route.odom_pose_at_measurement);
            SparseTofObservationBuildInput build;
            build.frame = route.frame;
            build.estimated_pose = candidate_base.map_T_base;
            build.explicit_return_kind = route.return_kind;
            build.synchronized = true;
            build.now_s = route.effective_timestamp_s;
            const auto observation = builder.build(build);
            if (!observation.ok) {
                m.ignored_ray_count++;
                continue;
            }
            const double range_m =
                route.return_kind == ScalarTofReturnKind::NoReturn
                    ? observation.observation.free_space_range_m
                    : observation.observation.measured_range_m;
            const auto traversal = sparse_grid_traverse_ray(
                observation.observation.sensor_origin_x_m,
                observation.observation.sensor_origin_y_m,
                observation.observation.ray_yaw_rad,
                range_m, grid_config);
            if (!traversal.ok) {
                out.traversal_budget_exceeded =
                    traversal.message.find("limit") != std::string::npos;
                if (out.traversal_budget_exceeded) {
                    return out;
                }
                m.ignored_ray_count++;
                continue;
            }
            m.used_ray_count++;
            if (route.return_kind == ScalarTofReturnKind::Hit) {
                m.hit_ray_count++;
                if (!traversal.cells.empty()) {
                    for (std::size_t i = 0; i + 1U < traversal.cells.size(); ++i) {
                        score_free_cell(m,
                                        input.reference_map->query_cell(
                                            traversal.cells[i]),
                                        input.config);
                    }
                    score_hit_endpoint(
                        m, input.reference_map->query_cell(traversal.cells.back()),
                        input.config);
                }
            } else {
                m.no_return_ray_count++;
                for (const auto &cell : traversal.cells) {
                    score_free_cell(m, input.reference_map->query_cell(cell),
                                    input.config);
                }
            }
        }
        if (m.traversed_cell_count > 0) {
            m.unknown_ratio = static_cast<double>(m.unknown_cell_count) /
                              static_cast<double>(m.traversed_cell_count);
            m.normalized_score = m.raw_score /
                                 static_cast<double>(m.traversed_cell_count);
        }
        if (m.known_cell_count > 0) {
            m.contradiction_ratio =
                static_cast<double>(m.contradiction_count) /
                static_cast<double>(m.known_cell_count);
        }
        m.valid = m.used_ray_count >= input.config.min_used_rays &&
                  m.known_cell_count >= input.config.min_known_cells &&
                  m.unknown_ratio <= input.config.max_unknown_ratio &&
                  m.contradiction_ratio <=
                      input.config.max_contradiction_ratio;
        return out;
    }

    static bool score_candidates(
        const SparseTofLocalMatchInput &input,
        const std::vector<RayRef> &rays,
        const std::vector<double> &yaw_deltas,
        SparseTofLocalMatchResult &result) {
        for (double delta : yaw_deltas) {
            const auto evaluated = evaluate_candidate(input, rays, delta);
            if (evaluated.traversal_budget_exceeded) {
                return false;
            }
            result.candidate_metrics.push_back(evaluated.metrics);
        }
        result.evaluated_candidate_count = result.candidate_metrics.size();
        return true;
    }

    static bool better(const SparseTofLocalMatchCandidateMetrics &a,
                       const SparseTofLocalMatchCandidateMetrics &b) {
        if (a.normalized_score != b.normalized_score) {
            return a.normalized_score > b.normalized_score;
        }
        if (std::fabs(a.delta_yaw_rad) != std::fabs(b.delta_yaw_rad)) {
            return std::fabs(a.delta_yaw_rad) < std::fabs(b.delta_yaw_rad);
        }
        return a.delta_yaw_rad < b.delta_yaw_rad;
    }

    static std::optional<std::size_t> best_valid_index(
        const std::vector<SparseTofLocalMatchCandidateMetrics> &metrics) {
        std::optional<std::size_t> best;
        for (std::size_t i = 0; i < metrics.size(); ++i) {
            if (metrics[i].valid &&
                (!best.has_value() || better(metrics[i], metrics[*best]))) {
                best = i;
            }
        }
        return best;
    }

    static void erase_existing_candidates(
        const std::vector<SparseTofLocalMatchCandidateMetrics> &existing,
        std::vector<double> &candidate_deltas) {
        candidate_deltas.erase(
            std::remove_if(candidate_deltas.begin(), candidate_deltas.end(),
                [&existing](double delta) {
                    return std::any_of(existing.begin(), existing.end(),
                        [delta](const auto &m) {
                            return same_candidate(delta, m.delta_yaw_rad);
                        });
                }), candidate_deltas.end());
    }

    static SparseTofLocalMatchResult reject_for_invalid_candidates(
        SparseTofLocalMatchResult result,
        const SparseTofLocalMatchConfig &config) {
        bool enough_rays = false;
        bool enough_known = false;
        bool unknown_ok = false;
        bool contradiction_ok = false;
        for (const auto &m : result.candidate_metrics) {
            enough_rays = enough_rays || m.used_ray_count >= config.min_used_rays;
            enough_known = enough_known || m.known_cell_count >= config.min_known_cells;
            unknown_ok = unknown_ok || m.unknown_ratio <= config.max_unknown_ratio;
            contradiction_ok = contradiction_ok ||
                               m.contradiction_ratio <=
                                   config.max_contradiction_ratio;
        }
        if (!enough_rays || !enough_known) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedInsufficientInformation,
                          "local_match_insufficient_information");
        }
        if (!unknown_ok) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedUnknownDominated,
                          "local_match_unknown_dominated");
        }
        if (!contradiction_ok) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedContradictory,
                          "local_match_contradictory");
        }
        return reject(result,
                      SparseTofLocalMatchStatus::RejectedNoValidCandidate,
                      "local_match_no_valid_candidate");
    }

    static bool is_local_peak(
        const std::vector<SparseTofLocalMatchCandidateMetrics> &curve,
        std::size_t i) {
        const double score = curve[i].normalized_score;
        const bool left = i == 0 || score >= curve[i - 1].normalized_score;
        const bool right = i + 1U == curve.size() ||
                           score >= curve[i + 1].normalized_score;
        return left && right;
    }

    static SparseTofLocalMatchResult select_and_gate(
        const SparseTofLocalMatchInput &input,
        SparseTofLocalMatchResult result) {
        const auto best_index = best_valid_index(result.candidate_metrics);
        if (!best_index.has_value()) {
            return reject_for_invalid_candidates(std::move(result), input.config);
        }
        const auto &best = result.candidate_metrics[*best_index];
        result.best_metrics = best;
        result.best_score = best.normalized_score;
        result.best_delta_yaw_rad = best.delta_yaw_rad;
        result.best_yaw_rad = best.candidate_yaw_rad;

        std::vector<SparseTofLocalMatchCandidateMetrics> curve;
        for (const auto &m : result.candidate_metrics) {
            if (m.valid) curve.push_back(m);
        }
        std::sort(curve.begin(), curve.end(), [](const auto &a, const auto &b) {
            return a.delta_yaw_rad < b.delta_yaw_rad;
        });
        result.score_min = curve.front().normalized_score;
        result.score_max = curve.front().normalized_score;
        double score_sum = 0.0;
        for (const auto &m : curve) {
            result.score_min = std::min(result.score_min, m.normalized_score);
            result.score_max = std::max(result.score_max, m.normalized_score);
            score_sum += m.normalized_score;
        }
        result.score_mean = score_sum / static_cast<double>(curve.size());
        result.score_range = result.score_max - result.score_min;

        std::optional<SparseTofLocalMatchCandidateMetrics> runner;
        for (const auto &m : curve) {
            const double separation = std::fabs(sparse_slam_shortest_yaw_delta(
                best.candidate_yaw_rad, m.candidate_yaw_rad));
            if (separation + 1e-12 <
                input.config.runner_up_exclusion_yaw_rad) {
                continue;
            }
            if (!runner.has_value() || better(m, *runner)) runner = m;
        }
        if (runner.has_value()) {
            result.runner_up_available = true;
            result.runner_up_yaw_rad = runner->candidate_yaw_rad;
            result.second_best_score = runner->normalized_score;
            result.score_margin = best.normalized_score - runner->normalized_score;
            result.runner_up_yaw_separation_rad = std::fabs(
                sparse_slam_shortest_yaw_delta(best.candidate_yaw_rad,
                                               runner->candidate_yaw_rad));
        }

        const double edge_tolerance =
            std::max(input.config.fine_yaw_step_rad * 0.51, 1e-12);
        result.best_at_search_edge =
            std::fabs(std::fabs(best.delta_yaw_rad) -
                      input.config.max_abs_yaw_rad) <= edge_tolerance;
        result.flat_curve =
            result.score_range < input.config.minimum_score_range;

        for (std::size_t i = 0; i < curve.size(); ++i) {
            if (!is_local_peak(curve, i)) continue;
            const double separation = std::fabs(sparse_slam_shortest_yaw_delta(
                best.candidate_yaw_rad, curve[i].candidate_yaw_rad));
            if (separation + 1e-12 >=
                    input.config.runner_up_exclusion_yaw_rad &&
                best.normalized_score - curve[i].normalized_score <=
                    input.config.multimodal_max_score_drop) {
                result.multimodal = true;
                break;
            }
        }

        if (result.best_at_search_edge) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedBestAtSearchEdge,
                          "local_match_best_at_search_edge");
        }
        if (result.flat_curve) {
            return reject(result, SparseTofLocalMatchStatus::RejectedFlatCurve,
                          "local_match_flat_curve");
        }
        if (result.multimodal) {
            return reject(result,
                          SparseTofLocalMatchStatus::RejectedMultimodal,
                          "local_match_multimodal");
        }
        if (best.normalized_score < input.config.minimum_normalized_score) {
            return reject(result, SparseTofLocalMatchStatus::RejectedLowScore,
                          "local_match_score_below_minimum");
        }
        if (!result.runner_up_available || !result.score_margin.has_value() ||
            *result.score_margin < input.config.minimum_score_margin) {
            return reject(result, SparseTofLocalMatchStatus::RejectedLowMargin,
                          "local_match_margin_below_minimum");
        }

        result.status = SparseTofLocalMatchStatus::AcceptedYawOnly;
        result.accepted = true;
        result.proposal_ready = true;
        result.correction_delta = RobotPose2D{0.0, 0.0, best.delta_yaw_rad};
        result.proposed_map_from_odom = propose_yaw_corrected_map_from_odom(
            input.predicted_map_from_odom, best.delta_yaw_rad);
        result.reason = "local_match_yaw_proposal_accepted";
        return result;
    }
};

} // namespace robot_slamd
