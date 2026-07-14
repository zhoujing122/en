#include "m3d2c_runtime_test_helpers.hpp"

#include <cstdio>

namespace {
bool same_cells(const robot_slamd::SparseOccupancyGridSnapshot &a,
                const robot_slamd::SparseOccupancyGridSnapshot &b) {
    if (a.cell_count() != b.cell_count()) return false;
    for (std::size_t i = 0; i < a.cells().size(); ++i) {
        if (!(a.cells()[i].key == b.cells()[i].key) ||
            a.cells()[i].evidence != b.cells()[i].evidence) return false;
    }
    return true;
}
}

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    using namespace m3d2c_test;
    const std::string path = "/tmp/en_m3d3a_runtime_lifecycle.map";
    std::remove(path.c_str());

    auto create_cfg = atomic_config();
    create_cfg.sparse_slam_planar_tof_extrinsics_configured = true;
    create_cfg.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    create_cfg.sparse_slam_front_tof_x_m = 0.12;
    create_cfg.sparse_slam_front_tof_y_m = 0.01;
    create_cfg.sparse_slam_front_tof_yaw_rad = 0.03;
    create_cfg.sparse_slam_left_tof_x_m = -0.01;
    create_cfg.sparse_slam_left_tof_y_m = 0.10;
    create_cfg.sparse_slam_left_tof_yaw_rad = 1.54;
    create_cfg.sparse_slam_right_tof_x_m = -0.02;
    create_cfg.sparse_slam_right_tof_y_m = -0.11;
    create_cfg.sparse_slam_right_tof_yaw_rad = -1.53;
    create_cfg.sparse_slam_map_path = path;
    create_cfg.sparse_slam_map_id = "runtime_map";
    create_cfg.sparse_slam_map_run_id = "create_run";
    validate_config(create_cfg);

    SparseSlamRuntimeCore created(create_cfg);
    created.initialize(sparse_slam_initialization_request_from_config(create_cfg));
    created.step(scene(0.01, 0.0, 0.0, 1000, 1400, 2200), 0.01);
    created.step(scene(0.02, 0.0, 0.0, 1250, 1800, 2000), 0.02);
    const auto before_save = created.sparse_map_snapshot();
    expect(before_save.valid() && before_save.cell_count() > 0,
           "create-new runtime has non-empty sparse map");
    const auto saved = created.save_sparse_map(path);
    if (!saved.ok) {
        std::cerr << "save_reason=" << saved.reason
                  << " init=" << created.report().initialization_complete
                  << " state=" << created.report().active_bundle_state
                  << " prepared=" << created.prepared_local_match_input().is_ready()
                  << " result=" << (created.local_match_result() != nullptr)
                  << " snapshot=" << (created.reference_map_snapshot() != nullptr)
                  << "\n";
    }
    expect(saved.ok, "stable idle runtime saves map");
    created.step(request(
        scene(0.03, 0.0, 0.0, 1000, 1400, 2200),
        ActiveObservationControl::BeginCollection));
    expect(!created.save_sparse_map(path).ok,
           "active bundle blocks map save");

    auto load_cfg = create_cfg;
    load_cfg.sparse_slam_map_startup_mode = "load_existing";
    load_cfg.sparse_slam_initial_pose_mode = "configured_pose";
    load_cfg.sparse_slam_has_configured_pose = true;
    load_cfg.sparse_slam_configured_pose_x_m = 2.0;
    load_cfg.sparse_slam_configured_pose_y_m = -1.0;
    load_cfg.sparse_slam_configured_pose_yaw_rad = 0.35;
    load_cfg.sparse_slam_map_run_id = "load_run";
    validate_config(load_cfg);

    SparseSlamRuntimeCore loaded(load_cfg);
    const auto load_start = loaded.initialize(
        sparse_slam_initialization_request_from_config(load_cfg));
    if (load_start.status !=
        SparseSlamInitializationStatus::WaitingForStationarySamples) {
        std::cerr << "load_reason=" << load_start.message << "\n";
    }
    expect(load_start.status ==
               SparseSlamInitializationStatus::WaitingForStationarySamples,
           "load enters normal stationary initialization");
    loaded.step(scene(0.01, 0.0, 0.0, 1000, 1400, 2200), 0.01);
    const auto after_load = loaded.sparse_map_snapshot();
    expect(same_cells(before_save, after_load) &&
               after_load.revision() == before_save.revision(),
           "loaded cells evidence and revision equal saved map");
    expect(loaded.keyframe_count() == 0,
           "map load does not fabricate historical keyframes");
    const auto map_from_odom = loaded.frame_state().current_map_from_odom();
    expect(std::fabs(map_from_odom.map_T_odom.x_m - 2.0) < 1e-12 &&
               std::fabs(map_from_odom.map_T_odom.y_m + 1.0) < 1e-12 &&
               std::fabs(map_from_odom.map_T_odom.yaw_rad - 0.35) < 1e-12,
           "configured map_T_base defines new map_T_odom");

    const auto loaded_revision = loaded.report().current_map_revision;
    const auto before_commit = run_atomic_sequence(loaded);
    expect(before_commit.revision >= loaded_revision,
           "loaded map remains usable before local slam commit");
    expect(loaded.report().atomic_commit_success_count == 1 &&
               loaded.report().current_map_revision == before_commit.revision + 1,
           "loaded runtime continues one atomic keyframe revision");
    expect(loaded.report().sparse_map_loaded &&
               loaded.report().sparse_map_load_success_count == 1,
           "runtime reports one transactional load");
    std::remove(path.c_str());
    return failures == 0 ? 0 : 1;
}
