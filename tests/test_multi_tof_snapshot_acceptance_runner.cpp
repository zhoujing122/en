#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_acceptance_runner.hpp"

#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    const auto report = MultiTofSnapshotAcceptanceRunner().run_once();
    expect(report.ok, "runner ok");
    expect(report.valid_result.ok, "valid case pass");
    expect(report.degraded_result.ok, "degraded case pass");
    expect(report.degraded_result.degraded, "degraded marked");
    expect(report.failed_case_count == 0, "invalid cases expected fail");
    expect(report.summary == "multi_tof_snapshot_acceptance_passed", "summary passed");
    return failures ? 1 : 0;
}
