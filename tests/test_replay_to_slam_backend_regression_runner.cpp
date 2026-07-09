#include "robot_slamd/autonomy/real_adapters/slam_backend/regression/replay_to_slam_backend_regression_runner.hpp"

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
    ReplayToSlamBackendRegressionRunner runner;
    const auto report = runner.run_once();
    expect(report.ok, "default runner ok");
    expect(report.accepted_update_count >= 3, "valid replay updates accepted");
    expect(report.final_quality.valid_scan_count >= 3, "final scan count");
    expect(report.rejected_update_count == 0, "valid replay no rejected updates");
    bool invalid_rejected = false;
    for (const auto &entry : report.passed) {
        invalid_rejected = invalid_rejected || entry == "invalid_replay_rejected";
    }
    expect(invalid_rejected, "invalid replay rejected");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
