#pragma once

#include "m3d2a1_runtime_test_helpers.hpp"

namespace m3d2c_test {

using namespace robot_slamd;

inline RobotSlamSensorSnapshot scene(double t, double wheel_yaw,
                                     double imu_yaw, std::uint16_t front,
                                     std::uint16_t left,
                                     std::uint16_t right) {
    auto s = m3d2a1_test::snapshot(t, 0.0, wheel_yaw, imu_yaw);
    s.multi_tof.front.distance_mm = front;
    s.multi_tof.front.distance_m = front / 1000.0;
    s.multi_tof.left.distance_mm = left;
    s.multi_tof.left.distance_m = left / 1000.0;
    s.multi_tof.right.distance_mm = right;
    s.multi_tof.right.distance_m = right / 1000.0;
    return s;
}

inline Config atomic_config() {
    auto cfg = m3d2a1_test::config();
    cfg.sparse_slam_active_bundle_min_snapshots = 4;
    cfg.sparse_slam_active_bundle_min_matchable_rays = 12;
    cfg.sparse_slam_local_match_min_bundle_frames = 4;
    cfg.sparse_slam_local_match_min_matchable_rays = 12;
    cfg.sparse_slam_local_match_min_used_rays = 6;
    cfg.sparse_slam_local_match_min_known_cells = 3;
    cfg.sparse_slam_local_match_max_unknown_ratio = 0.99;
    cfg.sparse_slam_local_match_max_contradiction_ratio = 0.99;
    cfg.sparse_slam_local_match_min_normalized_score = -1.0;
    cfg.sparse_slam_local_match_min_score_margin = 0.001;
    cfg.sparse_slam_local_match_min_score_range = 0.001;
    cfg.sparse_slam_local_match_multimodal_max_score_drop = 0.0;
    cfg.sparse_slam_local_match_runner_up_exclusion_yaw_rad = 0.05;
    cfg.sparse_slam_max_abs_yaw_correction_rad = 0.20;
    return cfg;
}

struct BeforeCommitState {
    std::uint64_t revision = 0;
    std::size_t cells = 0;
    MapFromOdom2D map_from_odom = identity_map_from_odom();
};

inline BeforeCommitState run_atomic_sequence(SparseSlamRuntimeCore &core) {
    core.step(scene(0.10, 0.0, 0.0, 1000, 1400, 2200), 0.10);
    core.step(scene(0.20, 0.0, 0.0, 1000, 1400, 2200), 0.20);
    core.step(scene(0.30, 0.0, 0.0, 1250, 1800, 2000), 0.30);
    core.step(scene(0.40, 0.0, 0.0, 1500, 1200, 2600), 0.40);
    BeforeCommitState before;
    before.revision = core.report().current_map_revision;
    before.cells = core.report().sparse_map_cell_count;
    before.map_from_odom = core.frame_state().current_map_from_odom();

    core.step(m3d2a1_test::request(
        scene(0.50, 1.0, 1.0, 1000, 1400, 2200),
        ActiveObservationControl::BeginCollection));
    core.step(scene(0.60, 0.0, 0.0, 1250, 1800, 2000), 0.60);
    core.step(m3d2a1_test::request(
        scene(0.70, 0.0, 0.0, 1500, 1200, 2600),
        ActiveObservationControl::EndMotionAndWaitForSettle));
    core.step(scene(0.81, 0.0, 0.0, 1100, 1600, 2300), 0.81);
    core.step(scene(0.92, 0.0, 0.0, 1000, 1400, 2200), 0.92);
    return before;
}

} // namespace m3d2c_test
