#include "m3d2a1_runtime_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    auto core = ready_core();
    auto idle = core.step(snapshot(0.2, 0.0), 0.2);
    const auto before = core.report();
    expect(idle.backend_called, "idle mapping reaches backend");
    expect(before.backend_accept_count == 1, "idle backend accepted once");
    expect(before.current_map_revision == 1, "idle mapping increments revision");
    const auto cell_count = before.sparse_map_cell_count;

    auto begin = core.step(request(snapshot(0.3, 0.05), ActiveObservationControl::BeginCollection));
    const auto &r = core.report();
    expect(begin.ok, "begin step succeeds");
    expect(!begin.backend_called, "begin step does not call backend");
    expect(r.active_bundle_state == "collecting_during_motion", "state collecting");
    expect(r.backend_update_attempt_count == before.backend_update_attempt_count, "backend attempts unchanged while collecting");
    expect(r.current_map_revision == before.current_map_revision, "revision unchanged while collecting");
    expect(r.sparse_map_cell_count == cell_count, "cell count unchanged while collecting");
    expect(r.bundle_snapshot_accept_count == 1, "bundle accepted first motion snapshot");
    expect(r.odom_update_during_motion_count == 1, "odometry continued during motion");
    expect(r.pose_buffer_append_count > before.pose_buffer_append_count, "pose buffer continued during motion");
    expect(r.map_commit_blocked_count == 1, "map commit blocked once");
    expect(r.map_update_during_bundle_count == 0, "no map update during bundle");
    expect(r.matcher_attempt_count == 0, "matcher not attempted");
    expect(r.keyframe_attempt_count == 0, "keyframe not attempted");
    expect(r.pose_correction_attempt_count == 0, "pose correction not attempted");
    return failures ? 1 : 0;
}
