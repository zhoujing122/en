#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_report_writer.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FakeRelocalizationResult result;
    result.ok = true;
    result.status = FakeRelocalizationStatus::Accepted;
    result.map_artifact.metadata.map_id = "fake_map_report";
    result.map_artifact.final_quality.coverage_ratio = 1.0;
    result.map_artifact.final_quality.yaw_coverage_ratio = 0.5;
    result.valid_range_count = 3;
    result.pose.valid = true;
    result.pose.confidence = 0.8;
    result.pose.x_m = 1.0;
    result.pose.y_m = 0.5;
    result.pose.yaw_rad = 0.1;
    result.confidence_band = FakeRelocalizationConfidenceBand::Medium;
    result.summary = "fake_relocalization_accepted";
    FakeRelocalizationReportWriter writer;
    const auto text = writer.write_text_report(result);
    expect(text.find("Fake Relocalization Report") != std::string::npos, "title");
    expect(text.find("fake_map_report") != std::string::npos, "map id");
    expect(text.find("confidence") != std::string::npos, "confidence");
    expect(text.find("pose candidate") != std::string::npos, "pose candidate");
    expect(text.find("No pose writeback is performed.") != std::string::npos, "writeback disclaimer");
    expect(text.find("real relocalization") != std::string::npos, "relocalization disclaimer");
    FakeMapRelocalizationRunnerReport runner_report;
    runner_report.ok = true;
    runner_report.pipeline_report.summary = "full_autonomous_slam_pipeline_passed";
    runner_report.relocalization_result = result;
    runner_report.summary = "fake_map_relocalization_pipeline_passed";
    const auto runner_text = writer.write_runner_report(runner_report);
    expect(runner_text.find("pipeline summary") != std::string::npos, "runner pipeline summary");
    return failures ? 1 : 0;
}
