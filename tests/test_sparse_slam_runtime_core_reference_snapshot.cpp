#include "m3d2b0_runtime_test_helpers.hpp"

int main() {
    using namespace m3d2b0_test;
    auto core = m3d2a1_test::ready_core();
    core.step(snapshot(0.2, 0.0), 0.2);
    core.step(request(snapshot(0.3, 0.05), ActiveObservationControl::BeginCollection));
    expect(core.report().reference_snapshot_capture_attempt_count == 0,
           "collecting does not capture map");
    core.step(request(snapshot(0.4, 0.0), ActiveObservationControl::EndMotionAndWaitForSettle));
    expect(core.report().reference_snapshot_capture_attempt_count == 0,
           "waiting does not capture map before freeze");
    core.step(snapshot(0.51, 0.0), 0.51);
    core.step(snapshot(0.62, 0.0), 0.62);
    const auto &r = core.report();
    expect(r.active_bundle_state == "frozen_ready", "bundle frozen");
    expect(r.reference_snapshot_capture_attempt_count == 1, "snapshot captured once");
    expect(r.reference_snapshot_capture_success_count == 1, "snapshot capture succeeded");
    expect(r.reference_snapshot_ready, "snapshot ready");
    expect(r.reference_snapshot_revision == r.reference_map_revision, "revision preserved");
    expect(r.reference_snapshot_cell_count == r.reference_map_cell_count, "cell count preserved");
    expect(core.reference_map_snapshot() != nullptr, "core owns snapshot");
    return m3d2a1_test::failures == 0 ? 0 : 1;
}
