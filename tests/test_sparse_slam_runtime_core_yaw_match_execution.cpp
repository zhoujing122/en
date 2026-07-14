#include "m3d2b0_runtime_test_helpers.hpp"
#include <cmath>

int main() {
    using namespace m3d2b0_test;
    auto core = m3d2a1_test::ready_core();
    core.step(snapshot(0.2, 0.0), 0.2);
    const auto reference_revision = core.report().current_map_revision;
    const auto reference_cells = core.report().sparse_map_cell_count;
    const auto frame_before = core.frame_state().current_map_from_odom();

    core.step(request(snapshot(0.3, 0.05),
                      ActiveObservationControl::BeginCollection));
    core.step(request(snapshot(0.4, 0.0),
                      ActiveObservationControl::EndMotionAndWaitForSettle));
    core.step(snapshot(0.51, 0.0), 0.51);
    core.step(snapshot(0.62, 0.0), 0.62);

    const auto &r = core.report();
    expect(r.active_bundle_state == "frozen_ready", "bundle frozen");
    expect(r.matcher_attempt_count == 1 && r.matcher_execution_count == 1,
           "matcher executes exactly once");
    expect(r.matcher_evaluated_candidate_count > 0,
           "runtime evaluates candidates");
    expect(r.matcher_accept_count + r.matcher_reject_count == 1,
           "accepted or rejected result retained");
    expect(core.local_match_result() != nullptr, "result available");
    expect(r.current_map_revision == reference_revision,
           "matcher does not change map revision");
    expect(r.sparse_map_cell_count == reference_cells,
           "matcher does not change map cells");
    const auto frame_after = core.frame_state().current_map_from_odom();
    expect(frame_after.map_T_odom.x_m == frame_before.map_T_odom.x_m &&
               frame_after.map_T_odom.y_m == frame_before.map_T_odom.y_m &&
               frame_after.map_T_odom.yaw_rad == frame_before.map_T_odom.yaw_rad,
           "matcher proposal is not applied");
    expect(r.map_from_odom_before_match.x_m ==
               r.map_from_odom_after_match.x_m &&
               r.map_from_odom_before_match.y_m ==
               r.map_from_odom_after_match.y_m &&
               r.map_from_odom_before_match.yaw_rad ==
               r.map_from_odom_after_match.yaw_rad,
           "report proves frame unchanged");
    expect(r.pose_correction_attempt_count == 0 &&
               r.keyframe_attempt_count == 0,
           "no correction or keyframe");

    core.step(snapshot(0.72, 0.0), 0.72);
    expect(core.report().matcher_attempt_count == 1,
           "frozen steps do not repeat matcher");
    return m3d2a1_test::failures == 0 ? 0 : 1;
}
