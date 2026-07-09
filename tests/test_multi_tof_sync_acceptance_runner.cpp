#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_acceptance_runner.hpp"

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
    const auto report = MultiTofSyncAcceptanceRunner().run_once();
    expect(report.ok, "runner ok");
    expect(report.case_count == 10, "case count");
    expect(report.valid_result.ok, "valid case pass");
    expect(report.degraded_result.ok && report.degraded_result.degraded, "degraded case pass");
    expect(report.failed_case_count == 0, "expected invalid cases fail");
    expect(report.summary == "multi_tof_sync_acceptance_passed", "summary passed");
    return failures ? 1 : 0;
}
