#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_report_writer.hpp"

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
} // namespace

int main() {
    using namespace robot_slamd;

    AutonomousSlamE2EScenarioReport report;
    report.ok = true;
    report.scenario_kind = AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood;
    report.stage = AutonomousSlamE2EStage::Passed;
    report.slam_backend_acceptance.ok = true;
    report.prelive_report.ok = true;
    report.prelive_report.final_state = AutonomousSlamState::Completed;
    report.prelive_report.final_map_quality.good_enough = true;
    report.prelive_report.counters.motion_command_count = 1;
    report.summary = "autonomous_slam_e2e_prelive_passed";
    report.passed.push_back("case_passed");
    report.failed.push_back("case_failed");
    report.warnings.push_back("case_warning");

    AutonomousSlamE2EReportWriter writer;
    const auto text = writer.write_text_report(report);
    expect(text.find("Autonomous SLAM End-to-End Pre-live Scenario Report") !=
               std::string::npos,
           "title present");
    expect(text.find("ActiveScanThenMapGood") != std::string::npos,
           "scenario present");
    expect(text.find("stage: Passed") != std::string::npos, "stage present");
    expect(text.find("block_reason: None") != std::string::npos, "block reason present");
    expect(text.find("slam_backend_acceptance_ok: true") != std::string::npos,
           "backend result present");
    expect(text.find("prelive_ok: true") != std::string::npos, "prelive result present");
    expect(text.find("final_map_quality_good: true") != std::string::npos,
           "map quality present");
    expect(text.find("motion_command_count: 1") != std::string::npos,
           "motion count present");
    expect(text.find("This report does not prove real robot motion.") !=
               std::string::npos,
           "motion disclaimer");
    expect(text.find("This report does not prove real ToF/IMU/Wheel driver readiness.") !=
               std::string::npos,
           "sensor disclaimer");
    expect(text.find("This report does not prove real SLAM backend quality.") !=
               std::string::npos,
           "slam disclaimer");
    expect(text.find("This report only validates end-to-end pre-live shadow integration.") !=
               std::string::npos,
           "shadow disclaimer");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
