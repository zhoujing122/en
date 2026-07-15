#include "m3d2c_runtime_test_helpers.hpp"

#include <cstdio>

namespace {

robot_slamd::Config relocalization_config(const std::string &path) {
    auto config = m3d2c_test::atomic_config();
    config.sparse_slam_planar_tof_extrinsics_configured = true;
    config.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    config.sparse_slam_front_tof_x_m = 0.0;
    config.sparse_slam_front_tof_y_m = 0.0;
    config.sparse_slam_front_tof_yaw_rad = 0.0;
    config.sparse_slam_left_tof_x_m = 0.0;
    config.sparse_slam_left_tof_y_m = 0.0;
    config.sparse_slam_left_tof_yaw_rad = robot_slamd::kPi / 2.0;
    config.sparse_slam_right_tof_x_m = 0.0;
    config.sparse_slam_right_tof_y_m = 0.0;
    config.sparse_slam_right_tof_yaw_rad = -robot_slamd::kPi / 2.0;
    config.sparse_slam_map_path = path;
    config.sparse_slam_map_id = "startup_relocalization_test";
    config.sparse_slam_reference_snapshot_max_cells = 100000;
    config.sparse_slam_local_match_min_reference_cells = 1;
    config.sparse_slam_local_match_min_reference_occupied_cells = 1;
    config.sparse_slam_local_match_min_reference_free_cells = 1;
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
    config.sparse_slam_relocalization_confirmation_xy_tolerance_m = 0.20;
    config.sparse_slam_relocalization_confirmation_yaw_tolerance_rad = 0.20;
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

} // namespace

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    using namespace m3d2c_test;
    const std::string path = "/tmp/en_m3d3b_startup_relocalization.map";
    std::remove(path.c_str());

    auto create_config = relocalization_config(path);
    validate_config(create_config);
    SparseSlamRuntimeCore creator(create_config);
    creator.initialize(
        sparse_slam_initialization_request_from_config(create_config));
    creator.step(scene(0.01, 0.0, 0.0, 1000, 1400, 2200), 0.01);
    for (int i = 0; i < 3; ++i) {
        creator.step(scene(
            0.10 + i * 0.10, 0.0, 0.0,
            static_cast<std::uint16_t>(1000 + i * 250),
            static_cast<std::uint16_t>(1400 + i * 200),
            static_cast<std::uint16_t>(2200 - i * 100)),
            0.10 + i * 0.10);
    }
    expect(creator.save_sparse_map(path).ok, "reference map saved");
    const auto reference = creator.sparse_map_snapshot();

    auto load_config = create_config;
    load_config.sparse_slam_map_startup_mode = "load_existing";
    load_config.sparse_slam_initial_pose_mode = "relocalization";
    load_config.sparse_slam_has_configured_pose = false;
    validate_config(load_config);
    SparseSlamRuntimeCore core(load_config);
    const auto init = core.initialize(
        sparse_slam_initialization_request_from_config(load_config));
    expect(init.status ==
               SparseSlamInitializationStatus::WaitingForStationarySamples,
           "existing map enters stationary calibration");
    core.step(scene(0.01, 0.0, 0.0, 1000, 1400, 2200), 0.01);
    expect(core.relocalization_required() &&
               !core.report().map_from_odom_initialized,
           "startup waits for automatic relocalization without configured pose");
    const auto revision_before = core.report().current_map_revision;

    collect_bundle(core, 1.0);
    expect(core.relocalization_state() ==
               SparseRelocalizationState::CollectingConfirmationBundle &&
               core.active_observation_state() ==
                   ActiveObservationBundleState::Idle,
           "first frozen bundle is consumed only as provisional evidence");
    collect_bundle(core, 1.50);
    if (!core.localization_ready()) {
        const auto &r = core.report();
        std::cerr << "relocalization_state=" << r.relocalization_state
                  << " status=" << r.relocalization_status
                  << " reason=" << r.relocalization_reason
                  << " searches=" << r.relocalization_search_attempt_count
                  << " active_bundle=" << r.active_bundle_state
                  << " last_fault=" << r.last_fault
                  << " last_message=" << r.last_message << "\n";
    }
    expect(core.localization_ready() && core.report().initialization_complete,
           "second independent bundle confirms startup map transform");
    expect(core.report().relocalization_search_attempt_count == 2 &&
               core.report().relocalization_success_count == 1,
           "exactly two searches produce one confirmed startup result");
    expect(core.report().current_map_revision == revision_before &&
               core.sparse_map_snapshot().cell_count() ==
                   reference.cell_count(),
           "relocalization never changes map cells or revision");
    expect(core.keyframe_count() == 0 &&
               core.report().atomic_commit_success_count == 0,
           "relocalization bundles do not create keyframes");
    std::remove(path.c_str());
    return failures == 0 ? 0 : 1;
}
