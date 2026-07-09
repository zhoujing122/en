#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_report_writer.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_runner.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FullAutonomousSlamScenarioBuilder builder;
    FullAutonomousSlamRunner runner;
    auto report = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::CompletesWithoutActiveScan));
    expect(report.ok, "pipeline ok");
    expect(report.fake_map_built, "built");
    expect(report.fake_map_saved, "saved");
    expect(report.fake_map_artifact.metadata.coverage_ratio == report.final_quality.coverage_ratio, "coverage matches");
    expect(report.fake_map_artifact.metadata.map_id.find("fake_map") == 0, "prefix");
    auto scenario = builder.build(FullAutonomousSlamPipelineScenarioKind::CompletesWithoutActiveScan);
    scenario.pipeline_options.save_fake_map_on_completed = false;
    scenario.pipeline_options.require_fake_map_saved = true;
    auto fail = runner.run_once(scenario);
    expect(!fail.ok && fail.fault == FullAutonomousSlamPipelineFault::FakeMapSaveFailed, "save required fail");
    const auto text = FullAutonomousSlamReportWriter{}.write_text_report(report);
    expect(text.find("fake map built") != std::string::npos, "report built");
    expect(text.find("fake map id") != std::string::npos, "report id");
    return failures ? 1 : 0;
}
