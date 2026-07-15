#include "sparse_tof_yaw_match_test_helpers.hpp"
#include "robot_slamd/localization/sparse_tof/sparse_multi_tof_relocalizer.hpp"

#include <cmath>

int main() {
    using namespace robot_slamd;
    using namespace yaw_match_test;

    const auto frames = asymmetric_frames(true, true);
    auto input = input_from_frames(frames, 0.0, 3.0);
    input.predicted_map_from_odom = identity_map_from_odom();

    SparseMultiTofRelocalizerConfig config;
    config.coarse_free_cell_stride = 2;
    config.coarse_yaw_step_rad = 0.5235987755982988;
    config.refine_xy_step_m = 0.05;
    config.refine_yaw_step_rad = 0.08726646259971647;
    config.final_xy_step_m = 0.025;
    config.final_yaw_step_rad = 0.017453292519943295;
    config.max_total_candidates = 12000;
    config.minimum_normalized_score = -1.0;
    config.minimum_score_margin = 0.001;
    config.minimum_score_range = 0.001;
    config.multimodal_max_score_drop = 0.0001;
    config.exclusion_xy_m = 0.20;
    config.exclusion_yaw_rad = 0.20;

    SparseMultiTofRelocalizer relocalizer;
    const auto first = relocalizer.search_global(input, config);
    expect(first.executed, "global search executed");
    expect(first.evaluated_candidate_count > 0, "global candidates evaluated");
    expect(first.best.has_value(), "global best available");
    expect(std::fabs(first.best->map_from_odom.map_T_odom.x_m - 0.35) <= 0.10,
           "global x recovered");
    expect(std::fabs(first.best->map_from_odom.map_T_odom.y_m + 0.20) <= 0.10,
           "global y recovered");
    expect(std::fabs(first.best->map_from_odom.map_T_odom.yaw_rad) <= 0.10,
           "global yaw recovered");

    const auto repeat = relocalizer.search_global(input, config);
    expect(first.status == repeat.status, "global status deterministic");
    expect(first.evaluated_candidate_count == repeat.evaluated_candidate_count,
           "candidate count deterministic");
    expect(first.best->map_from_odom.map_T_odom.x_m ==
               repeat.best->map_from_odom.map_T_odom.x_m &&
           first.best->map_from_odom.map_T_odom.y_m ==
               repeat.best->map_from_odom.map_T_odom.y_m &&
           first.best->map_from_odom.map_T_odom.yaw_rad ==
               repeat.best->map_from_odom.map_T_odom.yaw_rad,
           "best pose deterministic");

    const auto local = relocalizer.search_local(
        input, make_map_from_odom({0.25, -0.10, 0.12}), config);
    expect(local.executed && local.best.has_value(), "local recovery searched");
    expect(std::fabs(local.best->map_from_odom.map_T_odom.x_m - 0.35) <= 0.15,
           "local x recovered");
    expect(std::fabs(local.best->map_from_odom.map_T_odom.y_m + 0.20) <= 0.15,
           "local y recovered");

    auto budget = config;
    budget.max_total_candidates = 10;
    const auto exhausted = relocalizer.search_global(input, budget);
    expect(exhausted.status ==
               SparseRelocalizationStatus::RejectedCandidateBudget,
           "candidate budget fail closed");

    auto revision = input;
    revision.current_runtime_map_revision++;
    const auto mismatch = relocalizer.search_global(revision, config);
    expect(mismatch.status ==
               SparseRelocalizationStatus::RejectedRevisionChanged,
           "revision mismatch fail closed");
    return 0;
}
