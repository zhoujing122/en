#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_report_writer.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FullAutonomousSlamPipelineReport report;
    report.ok = true;
    report.stage = FullAutonomousSlamPipelineStage::Completed;
    report.final_state = AutonomousSlamState::Completed;
    report.backend_accepted_update_count = 3;
    report.active_scan_command_count = 1;
    report.final_quality.good_enough = true;
    report.final_quality.coverage_ratio = 1.0;
    report.summary = "full_autonomous_slam_pipeline_passed";
    const auto text = FullAutonomousSlamReportWriter{}.write_text_report(report);
    expect(text.find("Full Autonomous SLAM Fake Pipeline Report") != std::string::npos, "title");
    expect(text.find("final state") != std::string::npos, "state");
    expect(text.find("backend accepted update count") != std::string::npos, "backend count");
    expect(text.find("active scan command count") != std::string::npos, "scan count");
    expect(text.find("final coverage_ratio") != std::string::npos, "quality");
    expect(text.find("fake/replay sensor data") != std::string::npos, "fake disclaimer");
    expect(text.find("No real robot motion is executed.") != std::string::npos, "motion disclaimer");
    return failures ? 1 : 0;
}
