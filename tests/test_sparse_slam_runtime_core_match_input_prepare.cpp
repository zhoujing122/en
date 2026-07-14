#include "m3d2b0_runtime_test_helpers.hpp"

int main() {
    using namespace m3d2b0_test;
    auto core = m3d2a1_test::ready_core();
    freeze_ready_bundle(core);
    const auto &r = core.report();
    const auto &prepared = core.prepared_local_match_input();
    expect(prepared.is_ready(), "prepared input ready");
    expect(r.matcher_input_prepare_attempt_count == 1, "prepare attempted once");
    expect(r.matcher_input_prepare_success_count == 1, "prepare succeeded once");
    expect(r.matcher_input_ready, "report input ready");
    const auto *input = prepared.input();
    expect(input != nullptr, "prepared input available");
    if (input != nullptr) {
        expect(input->bundle_id == r.bundle_id, "bundle id preserved");
        expect(input->expected_reference_map_revision == r.reference_map_revision,
               "expected revision preserved");
        expect(input->current_runtime_map_revision == r.current_map_revision,
               "current revision preserved");
        expect(input->frozen_bundle->summary().map_from_odom_at_start.map_T_odom.yaw_rad ==
                   input->map_from_odom_at_collection_start.map_T_odom.yaw_rad,
               "collection-start map transform preserved");
        expect(input->predicted_map_from_odom.map_T_odom.yaw_rad ==
                   core.frame_state().current_map_from_odom().map_T_odom.yaw_rad,
               "prediction comes from frame state");
    }
    expect(r.matcher_attempt_count == 0, "matcher not attempted");
    expect(!r.matcher_executed, "matcher not executed");
    expect(r.matcher_evaluated_candidate_count == 0, "no candidates evaluated");
    return m3d2a1_test::failures == 0 ? 0 : 1;
}
