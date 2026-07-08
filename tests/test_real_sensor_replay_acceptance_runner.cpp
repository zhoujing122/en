#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_acceptance_runner.hpp"

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

    RealSensorReplayAcceptanceRunner runner;
    const auto result = runner.run_once();
    expect(result.ok, "default runner ok");
    expect(result.valid_log_result.accepted_packet_count >= 3,
           "valid log case pass");
    expect(result.invalid_latency_result.rejected_packet_count > 0,
           "invalid latency fails as expected");
    expect(result.sync_too_large_result.rejected_packet_count > 0,
           "sync too large fails as expected");
    expect(result.summary == "real_sensor_replay_acceptance_passed",
           "summary passed");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
