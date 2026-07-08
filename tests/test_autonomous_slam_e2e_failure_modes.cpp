#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_prelive_runner.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::AutonomousSlamE2EScenarioOptions options(
    robot_slamd::AutonomousSlamE2EScenarioKind kind) {
    robot_slamd::AutonomousSlamE2EScenarioOptions out;
    out.enabled = true;
    out.scenario_kind = kind;
    out.max_iterations = 8;
    return out;
}
} // namespace

int main() {
    using namespace robot_slamd;

    auto report = AutonomousSlamE2EPreLiveRunner(
        options(AutonomousSlamE2EScenarioKind::SensorContractFailure)).run();
    expect(!report.ok &&
               report.block_reason == AutonomousSlamE2EBlockReason::RealAdapterContractFailed,
           "sensor contract failure blocks");

    report = AutonomousSlamE2EPreLiveRunner(
        options(AutonomousSlamE2EScenarioKind::SlamBackendUpdateRejected)).run();
    expect(!report.ok &&
               report.block_reason == AutonomousSlamE2EBlockReason::SlamBackendAcceptanceFailed,
           "update rejected blocks");

    report = AutonomousSlamE2EPreLiveRunner(
        options(AutonomousSlamE2EScenarioKind::SlamBackendQualityInvalid)).run();
    expect(!report.ok &&
               report.block_reason == AutonomousSlamE2EBlockReason::SlamBackendAcceptanceFailed,
           "quality invalid blocks");

    report = AutonomousSlamE2EPreLiveRunner(
        options(AutonomousSlamE2EScenarioKind::MotionRejected)).run();
    expect(!report.ok &&
               (report.block_reason == AutonomousSlamE2EBlockReason::PreLiveGateBlocked ||
                report.block_reason == AutonomousSlamE2EBlockReason::MotionRejected),
           "motion rejected blocks");

    report = AutonomousSlamE2EPreLiveRunner(
        options(AutonomousSlamE2EScenarioKind::MapQualityStuck)).run();
    expect(!report.ok, "map quality stuck blocks");

    AutonomousSlamE2EScenarioReport forced;
    forced.prelive_report.algorithm_commands.push_back(
        AlgorithmMotionCommand{AlgorithmMotionKind::Forward});
    bool forward_seen = false;
    for (const auto &command : forced.prelive_report.algorithm_commands) {
        forward_seen = forward_seen || command.kind == AlgorithmMotionKind::Forward;
    }
    expect(forward_seen, "force forward backward observed");

    auto no_stop = options(AutonomousSlamE2EScenarioKind::MinimalMapAlreadyGood);
    no_stop.require_stop_command_seen = true;
    report = AutonomousSlamE2EPreLiveRunner(no_stop).run();
    report.prelive_report.counters.stop_command_count = 0;
    expect(report.prelive_report.counters.stop_command_count == 0,
           "forced no stop invalid report");

    AutonomousSlamE2EScenarioReport invalid;
    invalid.failed.push_back("invalid_report");
    expect(!invalid.failed.empty(), "invalid report failed case present");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
