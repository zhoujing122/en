#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_report_writer.hpp"

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

    PreLiveIntegrationReport report;
    report.ok = true;
    report.stage = PreLiveIntegrationStage::Passed;
    report.block_reason = PreLiveBlockReason::None;
    report.readiness = RealAdapterReadiness::ShadowReady;
    report.final_state = AutonomousSlamState::Completed;
    report.final_fault = AutonomousSlamFault::None;
    report.counters.readiness_check_count = 1;
    report.passed.push_back("readiness_check_passed");
    report.failed.push_back("example_failed_case");
    report.warnings.push_back("example_warning");
    report.summary = "prelive_autonomous_slam_passed";
    report.algorithm_commands.push_back(AlgorithmMotionCommand{});
    report.software_commands.push_back(SoftwareMotionCommand{});

    PreLiveAutonomousSlamReportWriter writer;
    const auto text = writer.write_text_report(report);
    expect(text.find("Pre-live Autonomous SLAM Integration Report") != std::string::npos, "title present");
    expect(text.find("overall_ok") != std::string::npos, "ok present");
    expect(text.find("stage: passed") != std::string::npos, "stage present");
    expect(text.find("block_reason: none") != std::string::npos, "block reason present");
    expect(text.find("readiness: shadow_ready") != std::string::npos, "readiness present");
    expect(text.find("final_state: completed") != std::string::npos, "final state present");
    expect(text.find("final_fault: none") != std::string::npos, "final fault present");
    expect(text.find("counters:") != std::string::npos, "counters present");
    expect(text.find("passed_cases:") != std::string::npos, "passed present");
    expect(text.find("failed_cases:") != std::string::npos, "failed present");
    expect(text.find("warnings:") != std::string::npos, "warnings present");
    expect(text.find("algorithm_command_count: 1") != std::string::npos, "algorithm command count present");
    expect(text.find("software_command_count: 1") != std::string::npos, "software command count present");
    expect(text.find("This report does not prove real robot motion.") != std::string::npos, "motion disclaimer present");
    expect(text.find("This report does not prove real ToF/IMU/Wheel driver readiness.") != std::string::npos, "driver disclaimer present");
    expect(text.find("This report only validates pre-live shadow integration.") != std::string::npos, "shadow disclaimer present");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
