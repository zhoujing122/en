#include "sparse_tof_yaw_match_test_helpers.hpp"

int main() {
    using namespace yaw_match_test;
    SparseTofLocalMatcher matcher;

    auto budget = input_from_frames(asymmetric_frames(), 0.10);
    budget.config.max_coarse_candidates = 2;
    const auto budget_result = matcher.match(budget);
    expect(budget_result.status ==
               SparseTofLocalMatchStatus::RejectedCandidateBudget,
           "coarse budget rejected");
    expect(!budget_result.accepted, "budget does not truncate to acceptance");

    auto edge = input_from_frames(asymmetric_frames(), 0.20);
    edge.config.multimodal_max_score_drop = 0.0;
    const auto edge_result = matcher.match(edge);
    expect(edge_result.status ==
               SparseTofLocalMatchStatus::RejectedBestAtSearchEdge,
           "search-edge optimum rejected");
    expect(edge_result.best_at_search_edge, "edge diagnostic set");

    auto flat = input_from_frames(asymmetric_frames(), 0.10);
    flat.config.minimum_score_range = 2.0;
    const auto flat_result = matcher.match(flat);
    expect(flat_result.status == SparseTofLocalMatchStatus::RejectedFlatCurve,
           "flat score curve rejected");
    expect(flat_result.flat_curve, "flat diagnostic set");

    auto unknown = input_from_frames(asymmetric_frames(), 0.10);
    unknown.config.max_unknown_ratio = 0.0;
    const auto unknown_result = matcher.match(unknown);
    expect(unknown_result.status ==
               SparseTofLocalMatchStatus::RejectedUnknownDominated,
           "unknown-dominated candidates rejected");

    auto contradictory = input_from_frames(asymmetric_frames(), 0.10);
    contradictory.config.max_abs_yaw_rad = 0.0;
    contradictory.config.max_contradiction_ratio = 0.0;
    contradictory.config.max_unknown_ratio = 1.0;
    const auto contradictory_result = matcher.match(contradictory);
    expect(contradictory_result.status ==
               SparseTofLocalMatchStatus::RejectedContradictory,
           "contradictory candidate rejected");

    auto multimodal = input_from_frames(symmetric_frames(), 0.0);
    multimodal.config.max_abs_yaw_rad = 1.7;
    multimodal.config.coarse_yaw_step_rad = 0.10;
    multimodal.config.fine_yaw_step_rad = 0.02;
    multimodal.config.max_coarse_candidates = 40;
    multimodal.config.max_fine_candidates = 16;
    multimodal.config.max_total_candidates = 56;
    multimodal.config.max_candidate_count = 56;
    multimodal.config.runner_up_exclusion_yaw_rad = 0.20;
    multimodal.config.multimodal_max_score_drop = 0.40;
    const auto multimodal_result = matcher.match(multimodal);
    expect(multimodal_result.status ==
               SparseTofLocalMatchStatus::RejectedMultimodal ||
               multimodal_result.status == SparseTofLocalMatchStatus::RejectedLowMargin,
           "symmetric separated peaks rejected");
    expect(!multimodal_result.accepted, "symmetric scene has no proposal");
    return 0;
}
