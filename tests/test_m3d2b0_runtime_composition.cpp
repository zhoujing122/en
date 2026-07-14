#include "m3d2b0_runtime_test_helpers.hpp"

int main() {
    using namespace m3d2b0_test;
    auto core = m3d2a1_test::ready_core();
    freeze_ready_bundle(core);
    const auto before = core.report();
    core.step(snapshot(0.7, 0.0), 0.7);
    const auto &after = core.report();
    expect(after.reference_snapshot_capture_attempt_count == 1,
           "frozen lifecycle does not recapture");
    expect(after.matcher_input_prepare_attempt_count == 1,
           "frozen lifecycle does not reprepare");
    expect(after.current_map_revision == before.current_map_revision,
           "frozen map revision unchanged");
    expect(after.sparse_map_cell_count == before.sparse_map_cell_count,
           "frozen map cell count unchanged");
    expect(after.matcher_attempt_count == 0, "no matcher execution path");
    expect(after.keyframe_attempt_count == 0, "no keyframe path");
    expect(after.pose_correction_attempt_count == 0, "no correction path");
    expect(!after.real_hardware_accessed && !after.real_motion_attempted &&
           !after.real_map_write_attempted && !after.real_pose_writeback_attempted,
           "safety remains fail closed");
    return m3d2a1_test::failures == 0 ? 0 : 1;
}
