#include "robot_slamd/autonomy/real_adapters/real_adapter_stub_types.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
} // namespace

int main() {
    using namespace robot_slamd;

    expect(to_string(RealAdapterStubKind::Sensor) == "Sensor",
           "sensor kind string");
    expect(to_string(RealAdapterStubState::StubOnly) == "StubOnly",
           "stub only string");
    expect(to_string(LiveHandoffReadinessState::Blocked) == "Blocked",
           "blocked readiness string");
    expect(to_string(LiveHandoffBlockReason::RealSensorAdapterMissing) ==
               "RealSensorAdapterMissing",
           "sensor missing reason string");
    expect(real_adapter_stub_kind_id(RealAdapterStubKind::Sensor) == 0,
           "kind id stable");
    expect(real_adapter_stub_state_id(RealAdapterStubState::StubOnly) == 1,
           "state id stable");
    expect(live_handoff_readiness_state_id(
               LiveHandoffReadinessState::Blocked) == 0,
           "readiness id stable");
    expect(live_handoff_block_reason_id(
               LiveHandoffBlockReason::RealSensorAdapterMissing) == 2,
           "block reason id stable");

    RealAdapterStubStatus status;
    expect(!status.ready, "default stub status not ready");

    LiveHandoffReadinessReport report;
    expect(!report.ok, "default handoff report not ok");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
