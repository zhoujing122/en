#include "sparse_tof_yaw_match_test_helpers.hpp"
#include <cmath>

int main() {
    using namespace yaw_match_test;
    const double prediction_error = 0.10;
    auto input = input_from_frames(asymmetric_frames(), prediction_error);
    SparseTofLocalMatcher matcher;
    const auto result = matcher.match(input);
    expect(result.status == SparseTofLocalMatchStatus::AcceptedYawOnly,
           "unique scene accepted");
    expect(result.best_delta_yaw_rad.has_value(), "best delta available");
    expect(std::fabs(*result.best_delta_yaw_rad + prediction_error) <=
               input.config.fine_yaw_step_rad + 1e-12,
           "known yaw error recovered");
    expect(result.runner_up_available && result.score_margin.has_value(),
           "runner up available");
    expect(result.evaluated_candidate_count > 0 && result.proposal_ready,
           "proposal produced");
    expect(result.proposed_map_from_odom.has_value(), "proposal transform");
    expect(std::fabs(result.proposed_map_from_odom->map_T_odom.x_m - 0.35) < 1e-12 &&
               std::fabs(result.proposed_map_from_odom->map_T_odom.y_m + 0.20) < 1e-12,
           "yaw proposal preserves translation");
    expect(std::fabs(input.predicted_map_from_odom.map_T_odom.yaw_rad -
                     prediction_error) < 1e-12,
           "input prediction remains unchanged");
    return 0;
}
