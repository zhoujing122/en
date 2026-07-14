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
    auto discard = core.step(request(snapshot(0.73, 0.0), ActiveObservationControl::DiscardFrozenBundle, frozen.bundle_id));
    const auto discarded = core.report();
    expect(discard.backend_called, "discard step resumes idle mapping for current snapshot");
    expect(discarded.active_bundle_state == "idle", "discard returned idle");
    expect(discarded.bundle_discard_count == 1, "discard counted");
    expect(discarded.current_map_revision == frozen.current_map_revision + 1, "revision increments after discard idle mapping");
    expect(discarded.backend_accept_count == frozen.backend_accept_count + 1, "backend accepted after discard");
    return failures ? 1 : 0;
}
