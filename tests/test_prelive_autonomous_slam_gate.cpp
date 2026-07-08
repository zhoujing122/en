#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_gate.hpp"

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

robot_slamd::PreLiveIntegrationReport good_report() {
    robot_slamd::PreLiveIntegrationReport report;
    report.ok = true;
    report.stage = robot_slamd::PreLiveIntegrationStage::EvaluatingGate;
    report.readiness = robot_slamd::RealAdapterReadiness::ShadowReady;
    report.final_state = robot_slamd::AutonomousSlamState::Mapping;
    report.counters.adapter_acceptance_run_count = 1;
    report.counters.stop_command_count = 1;
    report.final_map_quality.good_enough = true;
    return report;
}

int main() {
    using namespace robot_slamd;

    PreLiveAutonomousSlamGate gate;
    auto result = gate.evaluate(good_report());
    expect(result.pass, "good report passes");

    auto report = good_report();
    report.counters.adapter_acceptance_run_count = 0;
    result = gate.evaluate(report);
    expect(!result.pass && contains(result.failed, "adapter_acceptance_not_run"), "adapter acceptance required");

    report = good_report();
    report.failed.push_back("some_failed_case");
    result = gate.evaluate(report);
    expect(!result.pass && contains(result.failed, "prelive_failed_cases_present"), "failed cases block");

    report = good_report();
    report.counters.motion_reject_count = 1;
    result = gate.evaluate(report);
    expect(!result.pass && contains(result.failed, "motion_rejection_seen"), "motion rejection blocks");

    report = good_report();
    report.counters.stop_command_count = 0;
    result = gate.evaluate(report);
    expect(!result.pass && contains(result.failed, "stop_command_not_seen"), "stop command required");

    PreLiveGateOptions options;
    options.require_active_scan_command_seen = true;
    PreLiveAutonomousSlamGate active_scan_gate(options);
    result = active_scan_gate.evaluate(good_report());
    expect(!result.pass && contains(result.failed, "active_scan_command_not_seen"), "active scan required when configured");

    options = {};
    options.require_map_quality_good = true;
    PreLiveAutonomousSlamGate map_gate(options);
    report = good_report();
    report.final_map_quality.good_enough = false;
    result = map_gate.evaluate(report);
    expect(!result.pass && contains(result.failed, "map_quality_not_good"), "map quality required when configured");

    report = good_report();
    report.final_state = AutonomousSlamState::Fault;
    result = gate.evaluate(report);
    expect(!result.pass && contains(result.failed, "coordinator_fault_state"), "fault state blocks");

    report = good_report();
    report.final_state = AutonomousSlamState::Mapping;
    result = gate.evaluate(report);
    expect(result.pass, "mapping allowed for shadow incomplete");

    report = good_report();
    AlgorithmMotionCommand forward;
    forward.kind = AlgorithmMotionKind::Forward;
    forward.reason = "forbidden";
    report.algorithm_commands.push_back(forward);
    result = gate.evaluate(report);
    expect(!result.pass && contains(result.failed, "forward_backward_command_forbidden"), "forward blocked if observed");
    expect(result.block_reason != PreLiveBlockReason::None, "block reason set");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
