#include "m3d2c_runtime_test_helpers.hpp"
#include <iostream>

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    using namespace m3d2c_test;
    auto cfg = atomic_config();
    SparseSlamRuntimeCore core(cfg);
    core.initialize(sparse_slam_initialization_request_from_config(cfg));
    const auto before = run_atomic_sequence(core);

    const auto &r = core.report();
    if (r.atomic_commit_success_count != 1) {
        std::cerr << "status=" << r.matcher_status
                  << " reason=" << r.matcher_rejection_reason
                  << " atomic=" << r.last_atomic_commit_fault
                  << " state=" << r.active_bundle_state << "\n";
    }
    expect(r.matcher_execution_count == 1, "matcher executes once");
    expect(r.atomic_commit_success_count == 1, "atomic commit succeeds");
    expect(r.correction_apply_success_count == 1,
           "correction applies exactly once");
    expect(r.keyframe_commit_count == 1 && core.keyframe_count() == 1,
           "one keyframe committed");
    expect(r.current_map_revision == before.revision + 1,
           "bundle transaction increments revision exactly once");
    expect(r.sparse_map_cell_count >= before.cells &&
               r.keyframe_transaction_changed_cell_count > 0,
           "bundle changes sparse map evidence");
    expect(r.active_bundle_state == "idle", "success returns to idle");
    expect(core.latest_keyframe() != nullptr &&
               core.latest_keyframe()->bundle_id == r.committed_bundle_id,
           "keyframe records consumed bundle");
    expect(std::fabs(core.latest_keyframe()->matcher_delta_yaw_rad) > 0.05,
           "non-zero accepted yaw correction recorded");
    const auto committed_transform = core.frame_state().current_map_from_odom();
    expect(committed_transform.map_T_odom.x_m ==
               core.latest_keyframe()->map_from_odom_after.map_T_odom.x_m &&
               committed_transform.map_T_odom.y_m ==
               core.latest_keyframe()->map_from_odom_after.map_T_odom.y_m &&
               committed_transform.map_T_odom.yaw_rad ==
               core.latest_keyframe()->map_from_odom_after.map_T_odom.yaw_rad,
           "map_T_odom becomes exact matcher proposal");
    expect(core.local_match_result() == nullptr,
           "consumed matcher result cleared");
    expect(core.prepared_local_match_input().input() == nullptr,
           "consumed prepared input cleared");
    expect(r.committed_revision_after == r.committed_revision_before + 1,
           "keyframe revision contract");
    const auto committed_revision = r.current_map_revision;
    core.step(scene(1.02, 0.0, 0.0, 1300, 1500, 2100), 1.02);
    expect(core.report().current_map_revision == committed_revision + 1,
           "idle mapping resumes after local slam commit");
    return failures == 0 ? 0 : 1;
}
