#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_scenario_types.hpp"

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

    expect(to_string(AutonomousSlamE2EScenarioKind::MinimalMapAlreadyGood) ==
               "MinimalMapAlreadyGood",
           "minimal map string");
    expect(to_string(AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood) ==
               "ActiveScanThenMapGood",
           "active scan string");
    expect(to_string(AutonomousSlamE2EBlockReason::ConfigDisabled) ==
               "ConfigDisabled",
           "config disabled string");
    expect(autonomous_slam_e2e_scenario_kind_id(
               AutonomousSlamE2EScenarioKind::MinimalMapAlreadyGood) == 0,
           "scenario id stable");
    expect(autonomous_slam_e2e_stage_id(AutonomousSlamE2EStage::Passed) == 6,
           "stage id stable");
    expect(autonomous_slam_e2e_block_reason_id(
               AutonomousSlamE2EBlockReason::ConfigDisabled) == 1,
           "block reason id stable");

    AutonomousSlamE2EScenarioReport report;
    expect(!report.ok, "default report not ok");
    expect(report.stage == AutonomousSlamE2EStage::NotStarted,
           "default stage not started");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
