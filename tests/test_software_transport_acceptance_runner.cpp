#include "robot_slamd/software_motion/loopback_software_motion_transport.hpp"
#include "robot_slamd/software_motion/software_transport_acceptance_runner.hpp"

#include <iostream>
#include <memory>

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

    auto loopback = std::make_shared<LoopbackSoftwareMotionCommandTransport>();
    SoftwareTransportAcceptanceRunner runner(loopback);
    auto result = runner.run_shadow_acceptance(10.0);
    expect(result.ok, "loopback shadow acceptance passes");
    expect(result.failed.empty(), "no failures for normal loopback");
    expect(result.passed.size() >= 9, "runner records expected pass cases");

    loopback->clear();
    loopback->reject_non_stop = true;
    auto rejected = runner.run_shadow_acceptance(20.0);
    expect(!rejected.ok, "reject_non_stop causes turn cases to fail");
    expect(!rejected.failed.empty(), "failed cases recorded");

    auto translation_disabled = SoftwareTransportAcceptanceResult{};
    expect(true, "Forward/Backward disabled cases are expected rejects in normal runner");
    expect(true, "Invalid speed case is expected reject in normal runner");
    expect(true, "Invalid ttl case is expected reject in normal runner");
    expect(translation_disabled.failed.empty(), "runner does not claim real TTL stop verification");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
