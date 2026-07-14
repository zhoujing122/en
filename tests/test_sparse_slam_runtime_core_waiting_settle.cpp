#include "m3d2a1_runtime_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    auto core = ready_core();
    core.step(snapshot(0.2, 0.0), 0.2);
    const auto reference = core.report();
    core.step(request(snapshot(0.3, 0.05), ActiveObservationControl::BeginCollection));
    core.step(request(snapshot(0.4, 0.0), ActiveObservationControl::EndMotionAndWaitForSettle));
    auto mid = core.report();
    expect(mid.active_bundle_state == "waiting_for_motion_settle", "state waiting for settle");
    expect(mid.bundle_snapshot_accept_count == 2, "waiting step appended bundle frame");
    expect(mid.odom_update_during_settle_count == 1, "odometry continued in settle");
    expect(mid.tof_snapshot_during_settle_count == 1, "tof continued in settle");
    expect(mid.backend_update_attempt_count == reference.backend_update_attempt_count, "backend not called in settle");
    expect(mid.current_map_revision == reference.current_map_revision, "revision blocked in settle");

    core.step(snapshot(0.51, 0.0), 0.51);
    core.step(snapshot(0.62, 0.0), 0.62);
    const auto &r = core.report();
    expect(r.active_bundle_state == "frozen_ready", "settle freezes bundle");
    expect(r.bundle_freeze_success_count == 1, "one freeze success");
    expect(r.frozen_bundle_available, "frozen bundle available");
    expect(r.frozen_bundle_immutable, "frozen bundle immutable");
    expect(r.bundle_snapshot_accept_count >= 4, "bundle continued growing through settle");
    expect(r.settle_success_count == 1, "settle success once");
    expect(r.backend_update_attempt_count == reference.backend_update_attempt_count, "backend still not called");
    expect(r.current_map_revision == reference.current_map_revision, "revision still unchanged");
    return failures ? 1 : 0;
}
