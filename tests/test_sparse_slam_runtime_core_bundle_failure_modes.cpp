#include "m3d2a1_runtime_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace m3d2a1_test;
    auto cfg = config();
    cfg.sparse_slam_active_bundle_max_snapshots = 2;
    cfg.sparse_slam_active_bundle_max_rays = 6;
    auto core = ready_core(cfg);
    core.step(snapshot(0.2, 0.0), 0.2);
    const auto idle = core.report();
    core.step(request(snapshot(0.3, 0.05), ActiveObservationControl::BeginCollection));
    core.step(snapshot(0.4, 0.05), 0.4);
    auto overflow = core.step(snapshot(0.5, 0.05), 0.5);
    const auto &r = core.report();
    expect(!overflow.backend_called, "overflow does not call backend");
    expect(r.active_bundle_state == "aborted", "capacity overflow aborts");
    expect(r.bundle_abort_count == 1, "abort counted");
    expect(r.current_map_revision == idle.current_map_revision, "aborted bundle did not update map");
    expect(r.backend_update_attempt_count == idle.backend_update_attempt_count, "backend attempts unchanged on abort");
    expect(r.map_commit_blocked_count >= 3, "map stayed blocked through abort");
    return failures ? 1 : 0;
}
