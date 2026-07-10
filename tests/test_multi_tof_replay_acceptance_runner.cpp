#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_acceptance_runner.hpp"

#include <iostream>
#include <string>

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
    const auto report = MultiTofReplayAcceptanceRunner().run_once();
    expect(report.ok, "acceptance ok");
    expect(report.passed_case_count >= 8, "cases passed");
    expect(report.failed_case_count == 0, "no failed cases");
    expect(report.summary == "multi_tof_replay_acceptance_passed", "summary passed");
    return failures ? 1 : 0;
}
