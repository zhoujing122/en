#include "sparse_tof_yaw_match_test_helpers.hpp"
#include <cmath>

int main() {
    using namespace yaw_match_test;
    auto input = input_from_frames(asymmetric_frames(), 0.10);
    SparseTofLocalMatcher matcher;
    const auto a = matcher.match(input);
    const auto b = matcher.match(input);
    expect(a.status == b.status && a.reason == b.reason,
           "status deterministic");
    expect(a.evaluated_candidate_count == b.evaluated_candidate_count &&
               a.candidate_metrics.size() == b.candidate_metrics.size(),
           "candidate sequence size deterministic");
    for (std::size_t i = 0; i < a.candidate_metrics.size(); ++i) {
        expect(a.candidate_metrics[i].delta_yaw_rad ==
                   b.candidate_metrics[i].delta_yaw_rad &&
                   a.candidate_metrics[i].normalized_score ==
                   b.candidate_metrics[i].normalized_score,
               "candidate order and score deterministic");
    }
    expect(a.runner_up_yaw_separation_rad.has_value() &&
               *a.runner_up_yaw_separation_rad + 1e-12 >=
                   input.config.runner_up_exclusion_yaw_rad,
           "runner-up excludes best neighborhood");

    const auto wrapped = propose_yaw_corrected_map_from_odom(
        make_map_from_odom({1.2, -0.7, 3.13}), 0.03);
    expect(std::fabs(wrapped.map_T_odom.x_m - 1.2) < 1e-12 &&
               std::fabs(wrapped.map_T_odom.y_m + 0.7) < 1e-12,
           "proposal preserves nonzero translation");
    expect(wrapped.map_T_odom.yaw_rad < -3.12,
           "proposal normalizes pi wrap");
    return 0;
}
