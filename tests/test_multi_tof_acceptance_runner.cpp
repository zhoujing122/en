#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_acceptance_runner.hpp"

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
    MultiTofAcceptanceRunner runner;
    const auto report = runner.run_once();
    expect(report.ok, "default runner ok");
    expect(report.case_count == 7, "case count");
    expect(report.failed_case_count == 0, "expected invalid cases rejected");
    expect(report.summary == "multi_tof_acceptance_passed", "summary passed");
    expect(report.valid_result.valid_tof_count == 3, "valid count");
    return failures ? 1 : 0;
}
