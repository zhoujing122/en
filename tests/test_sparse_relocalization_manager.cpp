#include "sparse_tof_yaw_match_test_helpers.hpp"
#include "robot_slamd/localization/sparse_tof/sparse_relocalization_manager.hpp"

#include <cmath>

namespace {

robot_slamd::SparseTofLocalMatchInput independent_input(
    const robot_slamd::SparseTofLocalMatchInput &first,
    std::uint64_t bundle_id,
    double time_offset_s) {
    using namespace robot_slamd;
    std::vector<ResolvedMultiTofObservationFrame> frames =
        first.frozen_bundle->frames();
    for (auto &frame : frames) {
        frame.snapshot_timestamp_s += time_offset_s;
        for (auto *route : {&frame.front, &frame.left, &frame.right}) {
            route->effective_timestamp_s += time_offset_s;
            route->frame.effective_timestamp_s += time_offset_s;
            route->map_pose.measurement_timestamp_s += time_offset_s;
        }
    }
    auto summary = first.frozen_bundle->summary();
    summary.bundle_id = bundle_id;
    summary.collection_start_timestamp_s += time_offset_s;
    summary.collection_end_timestamp_s += time_offset_s;
    SparseTofLocalMatchInput out = first;
    out.bundle_id = bundle_id;
    out.frozen_bundle =
        std::make_shared<const FrozenMultiTofObservationBundle>(
            summary, std::move(frames));
    out.match_request_timestamp_s += time_offset_s;
    return out;
}

robot_slamd::SparseRelocalizationManagerConfig manager_config() {
    robot_slamd::SparseRelocalizationManagerConfig config;
    config.search.coarse_free_cell_stride = 2;
    config.search.coarse_yaw_step_rad = 0.5235987755982988;
    config.search.refine_xy_step_m = 0.05;
    config.search.refine_yaw_step_rad = 0.08726646259971647;
    config.search.final_xy_step_m = 0.025;
    config.search.final_yaw_step_rad = 0.017453292519943295;
    config.search.max_total_candidates = 12000;
    config.search.minimum_normalized_score = -1.0;
    config.search.minimum_score_margin = 0.001;
    config.search.minimum_score_range = 0.001;
    config.search.multimodal_max_score_drop = 0.0001;
    config.search.exclusion_xy_m = 0.20;
    config.search.exclusion_yaw_rad = 0.20;
    config.confirmation_xy_tolerance_m = 0.15;
    config.confirmation_yaw_tolerance_rad = 0.15;
    return config;
}

} // namespace

int main() {
    using namespace robot_slamd;
    using namespace yaw_match_test;

    const auto input = input_from_frames(asymmetric_frames(true, true), 0.0);
    const auto confirmation = independent_input(input, 42, 10.0);

    SparseRelocalizationManager startup(manager_config());
    expect(startup.start_startup(11), "startup relocalization started");
    const auto first = startup.process(input);
    expect(first.ok && first.needs_confirmation && !first.succeeded,
           "first bundle only establishes provisional pose");
    expect(startup.state() ==
               SparseRelocalizationState::CollectingConfirmationBundle,
           "manager waits for independent confirmation");
    const auto confirmed = startup.process(confirmation);
    expect(confirmed.ok && confirmed.succeeded &&
               confirmed.accepted_map_from_odom.has_value(),
           "second independent bundle confirms absolute transform");
    expect(std::fabs(
               confirmed.accepted_map_from_odom->map_T_odom.x_m - 0.35) <= 0.10,
           "confirmed x recovered");
    expect(std::fabs(
               confirmed.accepted_map_from_odom->map_T_odom.y_m + 0.20) <= 0.10,
           "confirmed y recovered");

    SparseRelocalizationManager duplicate(manager_config());
    expect(duplicate.start_startup(11), "duplicate test started");
    expect(duplicate.process(input).needs_confirmation,
           "duplicate test provisional ready");
    const auto reused = duplicate.process(input);
    expect(reused.failed &&
               reused.reason ==
                   "relocalization_confirmation_bundle_not_independent",
           "same bundle cannot confirm itself");

    SparseRelocalizationManager recovery(manager_config());
    expect(recovery.start_recovery(
               11, make_map_from_odom({0.25, -0.10, 0.10})),
           "lost recovery started with current prediction");
    const auto recovered_first = recovery.process(input);
    expect(recovered_first.ok && recovered_first.search.accepted,
           "local recovery produces provisional candidate");
    expect(recovered_first.search.mode ==
               SparseRelocalizationSearchMode::LocalRecovery,
           "recovery tries bounded local search first");
    expect(recovery.process(confirmation).succeeded,
           "lost recovery also requires independent confirmation");
    return 0;
}
