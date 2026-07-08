#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_types.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool contains(const std::vector<std::string> &values, const std::string &needle) {
    for (const auto &value : values) {
        if (value == needle) {
            return true;
        }
    }
    return false;
}
} // namespace

int main() {
    using namespace robot_slamd;

    expect(to_string(PreLiveIntegrationStage::NotStarted) == "not_started", "NotStarted string");
    expect(to_string(PreLiveIntegrationStage::Passed) == "passed", "Passed string");
    expect(to_string(PreLiveBlockReason::ConfigDisabled) == "config_disabled", "ConfigDisabled string");
    expect(prelive_stage_id(PreLiveIntegrationStage::RunningCoordinator) == 4, "stage id stable");
    expect(prelive_block_reason_id(PreLiveBlockReason::ReadinessFailed) == 2, "block reason id stable");

    PreLiveIntegrationReport report;
    expect(!report.ok, "default report not ok");
    expect(report.stage == PreLiveIntegrationStage::NotStarted, "default stage not started");
    expect(report.block_reason == PreLiveBlockReason::None, "default block none");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
