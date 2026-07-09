#include "robot_slamd/autonomy/fake_relocalization/fake_map_relocalization_runner.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FakeMapRelocalizationRunnerOptions options;
    options.enabled = true;
    FakeMapRelocalizationRunner runner(options);
    const auto report = runner.run_once();
    expect(report.ok, "runner ok");
    expect(report.pipeline_report.ok, "pipeline ok");
    expect(report.pipeline_report.fake_map_built, "map built");
    expect(report.pipeline_report.fake_map_saved, "map saved");
    expect(report.map_loaded, "map loaded");
    expect(report.relocalization_result.ok, "relocalization ok");
    expect(report.relocalization_result.pose.valid, "pose valid");
    expect(!report.pose_writeback_attempted, "no pose writeback");
    expect(report.summary == "fake_map_relocalization_pipeline_passed", "summary");
    return failures ? 1 : 0;
}
