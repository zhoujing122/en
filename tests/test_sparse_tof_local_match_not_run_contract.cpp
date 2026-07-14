#include "robot_slamd/localization/sparse_tof/sparse_tof_local_matcher.hpp"
#include "sparse_tof_local_match_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace local_match_test;
    auto input = valid_input();
    input.config.mode = SparseTofLocalMatchMode::FullSE2;
    SparseTofLocalMatcher matcher;
    const auto result = matcher.match(input);
    expect(result.status == SparseTofLocalMatchStatus::NotImplemented,
           "FullSE2 explicitly not implemented");
    expect(!result.matcher_executed && !result.accepted,
           "FullSE2 matcher did not execute");
    expect(result.evaluated_candidate_count == 0, "no candidate evaluated");
    expect(!result.best_score && !result.second_best_score &&
               !result.score_margin, "no score fabricated");
    expect(!result.correction_delta && !result.proposed_map_from_odom,
           "no correction fabricated");
    expect(result.predicted_map_from_odom.map_T_odom.yaw_rad ==
               input.predicted_map_from_odom.map_T_odom.yaw_rad,
           "prediction only echoed as input metadata");
    return 0;
}
