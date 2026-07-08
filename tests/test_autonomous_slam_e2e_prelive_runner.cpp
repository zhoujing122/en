#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_prelive_runner.hpp"

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

robot_slamd::AutonomousSlamE2EScenarioOptions enabled(
    robot_slamd::AutonomousSlamE2EScenarioKind kind) {
    robot_slamd::AutonomousSlamE2EScenarioOptions options;
    options.enabled = true;
    options.scenario_kind = kind;
    options.max_iterations = 20;
    return options;
}
} // namespace

int main() {
    using namespace robot_slamd;

    AutonomousSlamE2EPreLiveRunner disabled;
    auto report = disabled.run();
    expect(!report.ok, "disabled not ok");
    expect(report.block_reason == AutonomousSlamE2EBlockReason::ConfigDisabled,
           "disabled reason");

    AutonomousSlamE2EPreLiveRunner minimal(
        enabled(AutonomousSlamE2EScenarioKind::MinimalMapAlreadyGood));
    report = minimal.run();
    expect(report.ok, "minimal ok");
    expect(report.prelive_report.ok, "minimal prelive ok");
    expect(report.prelive_report.counters.stop_command_count > 0, "minimal stop seen");

    AutonomousSlamE2EPreLiveRunner active(
        enabled(AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood));
    report = active.run();
    expect(report.ok, "active scan scenario ok");
    expect(report.slam_backend_acceptance.ok, "backend acceptance recorded");
    expect(report.prelive_report.ok, "prelive report recorded");
    expect(report.prelive_report.counters.active_scan_command_count > 0 ||
               report.prelive_report.counters.stop_command_count > 0,
           "active scan or stop seen");
    expect(report.prelive_report.counters.active_scan_command_count > 0,
           "active scan seen when map poor");
    expect(report.summary == "autonomous_slam_e2e_prelive_passed" ||
               report.summary == "autonomous_slam_e2e_prelive_blocked",
           "summary stable");

    for (const auto &command : report.prelive_report.algorithm_commands) {
        expect(command.kind != AlgorithmMotionKind::Forward &&
                   command.kind != AlgorithmMotionKind::Backward,
               "no forward backward generated");
    }

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
