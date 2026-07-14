#include "m3d2b0_runtime_test_helpers.hpp"

int main() {
    using namespace m3d2b0_test;
    auto core = m3d2a1_test::ready_core();
    freeze_ready_bundle(core);
    const auto revision = core.report().current_map_revision;
    const auto pose_buffer_size = core.report().pose_buffer_size;
    const auto bundle_id = core.report().bundle_id;
    const auto result = core.step(request(snapshot(0.7, 0.0),
                                          ActiveObservationControl::DiscardFrozenBundle,
                                          bundle_id));
    expect(result.ok, "discard succeeds");
    expect(core.report().active_bundle_state == "idle", "discard returns idle");
    expect(!core.report().reference_snapshot_ready, "discard clears snapshot");
    expect(!core.report().matcher_input_ready, "discard clears prepared input");
    expect(core.reference_map_snapshot() == nullptr, "snapshot ownership released");
    expect(!core.prepared_local_match_input().is_ready(), "prepared input reset");
    expect(core.report().current_map_revision == revision + 1,
           "same discard step resumes idle mapping");
    expect(core.report().pose_buffer_size >= pose_buffer_size,
           "discard does not clear pose buffer");
    expect(core.report().matcher_attempt_count == 1,
           "discard does not repeat prior match");
    expect(core.local_match_result() == nullptr, "discard clears match result");
    return m3d2a1_test::failures == 0 ? 0 : 1;
}
