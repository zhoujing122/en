#include "m3d2a1_runtime_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    auto core = ready_core();
    core.step(snapshot(0.2, 0.0), 0.2);
    core.step(request(snapshot(0.3, 0.05), ActiveObservationControl::BeginCollection));
    core.step(request(snapshot(0.4, 0.0), ActiveObservationControl::EndMotionAndWaitForSettle));
    core.step(snapshot(0.51, 0.0), 0.51);
    core.step(snapshot(0.62, 0.0), 0.62);
    const auto frozen = core.report();
    expect(frozen.active_bundle_state == "frozen_ready", "bundle frozen before frozen test");
    const auto attempts = frozen.backend_update_attempt_count;
    const auto accepted = frozen.bundle_snapshot_accept_count;
    const auto revision = frozen.current_map_revision;
    const auto cells = frozen.sparse_map_cell_count;

    auto after = core.step(snapshot(0.73, 0.0), 0.73);
    const auto &r = core.report();
    expect(after.odom_updated, "odometry still updates after freeze");
    expect(!after.backend_called, "backend blocked after freeze");
    expect(r.backend_update_attempt_count == attempts, "backend attempts unchanged after freeze");
    expect(r.bundle_snapshot_accept_count == accepted, "frozen bundle did not grow");
    expect(r.current_map_revision == revision, "revision unchanged after freeze");
    expect(r.sparse_map_cell_count == cells, "cell count unchanged after freeze");
    expect(r.odom_update_after_freeze_count == 1, "odom counted after freeze");
    return failures ? 1 : 0;
}
