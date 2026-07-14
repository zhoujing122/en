#include "m3d2c_runtime_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    using namespace m3d2c_test;
    auto cfg = atomic_config();
    cfg.sparse_slam_max_cells_per_keyframe_transaction = 1;
    SparseSlamRuntimeCore core(cfg);
    core.initialize(sparse_slam_initialization_request_from_config(cfg));
    const auto before = run_atomic_sequence(core);
    const auto &r = core.report();
    const auto after_transform = core.frame_state().current_map_from_odom();

    expect(r.matcher_accept_count == 1, "matcher proposal accepted");
    expect(r.atomic_commit_success_count == 0 &&
               r.atomic_commit_reject_count == 1,
           "transaction failure is fail closed");
    expect(r.active_bundle_state == "aborted",
           "failed transaction remains locked");
    expect(r.current_map_revision == before.revision,
           "failed transaction preserves revision");
    expect(r.sparse_map_cell_count == before.cells,
           "failed transaction preserves map");
    expect(core.keyframe_count() == 0,
           "failed transaction creates no keyframe");
    expect(after_transform.map_T_odom.x_m ==
               before.map_from_odom.map_T_odom.x_m &&
               after_transform.map_T_odom.y_m ==
               before.map_from_odom.map_T_odom.y_m &&
               after_transform.map_T_odom.yaw_rad ==
               before.map_from_odom.map_T_odom.yaw_rad,
           "failed transaction preserves map_T_odom");
    expect(core.local_match_result() != nullptr,
           "failure retains matcher result for diagnosis");
    return failures == 0 ? 0 : 1;
}
