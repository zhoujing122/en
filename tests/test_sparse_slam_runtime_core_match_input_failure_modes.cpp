#include "m3d2b0_runtime_test_helpers.hpp"

int main() {
    using namespace m3d2b0_test;
    auto cfg = m3d2a1_test::config();
    cfg.sparse_slam_reference_snapshot_max_cells = 1;
    auto core = m3d2a1_test::ready_core(cfg);
    freeze_ready_bundle(core);
    const auto &r = core.report();
    expect(r.active_bundle_state == "aborted", "snapshot failure aborts phase");
    expect(r.reference_snapshot_capture_attempt_count == 1, "capture attempted once");
    expect(r.reference_snapshot_capture_reject_count == 1, "capture rejected once");
    expect(r.matcher_input_prepare_reject_count == 1, "input rejected once");
    expect(!r.reference_snapshot_ready, "no partial snapshot");
    expect(!r.matcher_input_ready, "no partial prepared input");
    expect(core.reference_map_snapshot() == nullptr, "no snapshot ownership on failure");
    expect(!core.prepared_local_match_input().is_ready(), "prepared input remains rejected");
    expect(r.matcher_attempt_count == 0, "matcher remains uncalled");
    return m3d2a1_test::failures == 0 ? 0 : 1;
}
