#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_adapter_acceptance_runner.hpp"

#include <algorithm>
#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool has_case(const std::vector<std::string> &items, const std::string &name) {
    return std::find(items.begin(), items.end(), name) != items.end();
}
} // namespace

int main() {
    using namespace robot_slamd;

    RealSensorAdapterAcceptanceRunner runner;
    const auto result = runner.run_once();
    expect(result.ok, "default runner ok");
    expect(has_case(result.passed, "real_sensor_valid_packet_passed"),
           "valid packet pass");
    expect(has_case(result.passed, "real_sensor_missing_tof_failed_as_expected"),
           "missing tof expected");
    expect(has_case(result.passed,
                    "real_sensor_missing_imu_and_wheel_failed_as_expected"),
           "missing imu wheel expected");
    expect(has_case(result.passed,
                    "real_sensor_empty_tof_ranges_failed_as_expected"),
           "empty tof expected");
    expect(has_case(result.passed, "real_sensor_sync_too_large_failed_as_expected"),
           "sync expected");
    expect(has_case(result.passed,
                    "real_sensor_request_latency_too_high_failed_as_expected"),
           "latency expected");
    expect(result.summary == "real_sensor_adapter_acceptance_passed",
           "summary passed");
    expect(result.valid_build_result.ok, "valid build stored");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
