#include "m3d2c_runtime_test_helpers.hpp"

namespace {

robot_slamd::Config recovery_config() {
    auto config = m3d2c_test::atomic_config();
    config.sparse_slam_local_match_min_normalized_score = 2.0;
    config.sparse_slam_localization_health_max_consistency_failures = 2;
    config.sparse_slam_localization_health_min_lost_duration_s = 0.0;
    config.sparse_slam_localization_health_min_lost_odom_distance_m = 0.0;
    config.sparse_slam_relocalization_coarse_free_cell_stride = 1;
    config.sparse_slam_relocalization_coarse_yaw_step_rad =
        robot_slamd::kPi / 4.0;
    config.sparse_slam_relocalization_refine_xy_step_m = 0.05;
    config.sparse_slam_relocalization_refine_yaw_step_rad =
        robot_slamd::kPi / 36.0;
    config.sparse_slam_relocalization_final_xy_step_m = 0.025;
    config.sparse_slam_relocalization_final_yaw_step_rad =
        robot_slamd::kPi / 180.0;
    config.sparse_slam_relocalization_max_total_candidates = 15000;
    config.sparse_slam_relocalization_min_normalized_score = -1.0;
    config.sparse_slam_relocalization_min_score_margin = 0.0001;
    config.sparse_slam_relocalization_min_score_range = 0.0001;
    config.sparse_slam_relocalization_multimodal_max_score_drop = 0.0;
    config.sparse_slam_relocalization_exclusion_xy_m = 0.15;
    config.sparse_slam_relocalization_exclusion_yaw_rad = 0.20;
    config.sparse_slam_relocalization_confirmation_xy_tolerance_m = 0.50;
    config.sparse_slam_relocalization_confirmation_yaw_tolerance_rad = 0.50;
    return config;
}

void collect_bundle(robot_slamd::SparseSlamRuntimeCore &core,
                    double start_s) {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    using namespace m3d2c_test;
    core.step(request(
        scene(start_s, 0.0, 0.0, 1000, 1400, 2200),
        ActiveObservationControl::BeginCollection));
    core.step(scene(start_s + 0.10, 0.0, 0.0, 1250, 1800, 2000),
              start_s + 0.10);
    core.step(request(
        scene(start_s + 0.20, 0.0, 0.0, 1500, 1200, 2600),
        ActiveObservationControl::EndMotionAndWaitForSettle));
    core.step(scene(start_s + 0.31, 0.0, 0.0, 1100, 1600, 2300),
              start_s + 0.31);
    core.step(scene(start_s + 0.42, 0.0, 0.0, 1000, 1400, 2200),
              start_s + 0.42);
}

void discard_rejected(robot_slamd::SparseSlamRuntimeCore &core,
                      double timestamp_s) {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    using namespace m3d2c_test;
    const auto id = core.active_bundle_id();
    core.step(request(
        scene(timestamp_s, 0.0, 0.0, 1000, 1400, 2200),
        ActiveObservationControl::DiscardFrozenBundle, id));
}

bool same_cells(const robot_slamd::SparseOccupancyGridSnapshot &a,
                const robot_slamd::SparseOccupancyGridSnapshot &b) {
    if (a.cells().size() != b.cells().size()) return false;
    for (std::size_t i = 0; i < a.cells().size(); ++i) {
        if (!(a.cells()[i].key == b.cells()[i].key) ||
            a.cells()[i].evidence != b.cells()[i].evidence) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    using namespace m3d2c_test;

    auto config = recovery_config();
    validate_config(config);
    SparseSlamRuntimeCore core(config);
    core.initialize(sparse_slam_initialization_request_from_config(config));
    core.step(scene(0.01, 0.0, 0.0, 1000, 1400, 2200), 0.01);
    for (int i = 0; i < 4; ++i) {
        const double t = 0.10 + i * 0.10;
        core.step(scene(
            t, 0.0, 0.0,
            static_cast<std::uint16_t>(1000 + i * 250),
            static_cast<std::uint16_t>(1400 + i * 200),
            static_cast<std::uint16_t>(2200 - i * 100)), t);
    }

    collect_bundle(core, 0.50);
    const auto *first = core.local_match_result();
    if (!first || core.localization_health_state() !=
                      LocalizationHealthState::Degraded) {
        std::cerr << "first status=" << core.report().matcher_status
                  << " reason=" << core.report().matcher_rejection_reason
                  << " health=" << core.report().localization_health_state
                  << " bundle=" << core.report().active_bundle_state
                  << " settle_samples="
                  << core.report().settle_consecutive_sample_count
                  << " settle_duration="
                  << core.report().settle_stable_duration_s
                  << " snapshots="
                  << core.report().bundle_snapshot_accept_count
                  << " fault=" << core.report().last_fault
                  << " message=" << core.report().last_message << "\n";
    }
    if (first) {
        expect(localization_match_evidence(first->status) ==
                   LocalizationMatchEvidence::ConsistencyFailure,
               "first forced mismatch is consistency evidence");
    }
    expect(core.localization_health_state() ==
               LocalizationHealthState::Degraded &&
               !core.relocalization_required(),
           "one consistency failure degrades but does not lose localization");
    discard_rejected(core, 1.10);

    collect_bundle(core, 1.20);
    if (!core.relocalization_required()) {
        std::cerr << "second status=" << core.report().matcher_status
                  << " reason=" << core.report().matcher_rejection_reason
                  << " health=" << core.report().localization_health_state
                  << " consistency="
                  << core.report().localization_consistency_failure_count
                  << " bundle=" << core.report().active_bundle_state << "\n";
    }
    expect(core.localization_health_state() == LocalizationHealthState::Lost &&
               core.relocalization_required() &&
               core.relocalization_state() ==
                   SparseRelocalizationState::CollectingFirstBundle,
           "second consecutive consistency failure starts lost recovery");
    expect(core.active_observation_state() ==
               ActiveObservationBundleState::Idle &&
               core.report().localization_stop_required,
           "lost transition consumes rejected bundle and requires stop");

    const auto revision_before = core.report().current_map_revision;
    const auto map_before = core.sparse_map_snapshot();
    const auto odom_before = core.report().last_odom_pose;
    const auto keyframes_before = core.keyframe_count();

    collect_bundle(core, 1.90);
    expect(core.relocalization_state() ==
               SparseRelocalizationState::CollectingConfirmationBundle,
           "first recovery bundle yields provisional absolute transform");
    collect_bundle(core, 2.60);

    if (!core.localization_ready()) {
        const auto &r = core.report();
        std::cerr << "recovery state=" << r.relocalization_state
                  << " status=" << r.relocalization_status
                  << " reason=" << r.relocalization_reason
                  << " local=" << r.recovery_local_search_count
                  << " global=" << r.recovery_global_search_count
                  << " health=" << r.localization_health_state << "\n";
    }
    expect(core.localization_ready() &&
               core.localization_health_state() ==
                   LocalizationHealthState::Healthy,
           "independent confirmation bundle restores healthy localization");
    expect(core.report().recovery_attempt_count == 1 &&
               core.report().recovery_local_search_count >= 1 &&
               core.report().recovery_success_count == 1 &&
               core.report().exploration_replan_required_after_recovery,
           "recovery runs once and requests exploration replanning");
    expect(core.report().current_map_revision == revision_before &&
               same_cells(core.sparse_map_snapshot(), map_before) &&
               core.keyframe_count() == keyframes_before,
           "recovery changes neither sparse map revision nor keyframes");
    const auto recovered = core.frame_state().current_map_from_odom().map_T_odom;
    expect(std::fabs(recovered.x_m -
                     core.report().relocalization_best_x_m) < 1e-12 &&
               std::fabs(recovered.y_m -
                         core.report().relocalization_best_y_m) < 1e-12 &&
               core.report().last_odom_pose.x_m == odom_before.x_m &&
               core.report().last_odom_pose.y_m == odom_before.y_m,
           "recovery installs confirmed map transform without resetting odom");
    expect(!core.report().localization_stop_required,
           "confirmed recovery releases the stop requirement");
    return failures == 0 ? 0 : 1;
}
