#include "sparse_tof_local_match_test_helpers.hpp"

#include <type_traits>

int main() {
    using namespace robot_slamd;
    using namespace local_match_test;
    SparseTofLocalMatchConfig config;
    expect(config.mode == SparseTofLocalMatchMode::YawOnly,
           "default mode yaw only");
    SparseTofLocalMatchMode mode = SparseTofLocalMatchMode::YawOnly;
    expect(parse_sparse_tof_local_match_mode("full_se2", mode) &&
               mode == SparseTofLocalMatchMode::FullSE2,
           "full se2 contract expressible");
    expect(!parse_sparse_tof_local_match_mode("legacy", mode),
           "invalid mode rejected");

    SparseTofLocalMatchResult result;
    expect(result.status == SparseTofLocalMatchStatus::NotRun, "result not run");
    expect(!result.matcher_executed && !result.accepted, "result not executed");
    expect(result.evaluated_candidate_count == 0, "zero candidates");
    expect(!result.best_score && !result.second_best_score &&
               !result.score_margin, "scores absent");
    expect(!result.correction_delta && !result.proposed_map_from_odom,
           "correction absent");

    auto prepared = PreparedSparseTofLocalMatchInput::ready(valid_input());
    expect(prepared.is_ready(), "prepared input ready");
    expect(prepared.input() != nullptr, "prepared input retained");
    static_assert(std::is_same<
        decltype(prepared.input()),
        const SparseTofLocalMatchInput *>::value,
        "prepared input must be const-only");
    return 0;
}
