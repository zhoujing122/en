#include "robot_slamd/autonomy/fake_map/fake_map_storage.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
static robot_slamd::FullAutonomousSlamPipelineReport good_report() {
    robot_slamd::FullAutonomousSlamPipelineReport report;
    report.ok = true;
    report.final_state = robot_slamd::AutonomousSlamState::Completed;
    report.final_quality.good_enough = true;
    report.final_quality.coverage_ratio = 1.0;
    report.final_quality.yaw_coverage_ratio = 0.5;
    report.final_quality.valid_scan_count = 3;
    report.backend_accepted_update_count = 3;
    report.active_scan_command_count = 1;
    return report;
}
int main() {
    using namespace robot_slamd;
    FakeMapStorage disabled;
    auto build = disabled.build_from_pipeline_report(good_report(), "map_a", 10.0);
    expect(build.ok, "build ok");
    expect(!build.artifact.metadata.map_id.empty(), "map id");
    expect(build.artifact.serialized_text.find("coverage") != std::string::npos, "coverage text");
    expect(build.artifact.serialized_text.find("yaw_coverage") != std::string::npos, "yaw text");
    expect(build.artifact.serialized_text.find("valid_scan_count") != std::string::npos, "scan text");
    auto save_disabled = disabled.save_artifact(build.artifact);
    expect(!save_disabled.ok && save_disabled.artifact.fault == FakeMapArtifactFault::SaveDisabled, "save disabled");
    FakeMapSaveOptions save; save.enabled = true;
    FakeMapLoadOptions load; load.enabled = true;
    FakeMapStorage storage(save, load);
    build = storage.build_from_pipeline_report(good_report(), "map_a", 10.0);
    auto saved = storage.save_artifact(build.artifact);
    expect(saved.ok, "save ok");
    expect(storage.artifact_count() == 1, "count one");
    auto duplicate = storage.save_artifact(build.artifact);
    expect(!duplicate.ok, "duplicate fail");
    FakeMapStorage load_disabled(save, FakeMapLoadOptions{});
    auto load_fail = load_disabled.load_artifact("map_a");
    expect(!load_fail.ok && load_fail.artifact.fault == FakeMapArtifactFault::LoadDisabled, "load disabled");
    auto loaded = storage.load_artifact("map_a");
    expect(loaded.ok && loaded.artifact.status == FakeMapArtifactStatus::Loaded, "load ok");
    auto bad = good_report(); bad.final_quality.good_enough = false;
    auto bad_build = storage.build_from_pipeline_report(bad, "bad", 10.0);
    expect(!bad_build.ok && bad_build.artifact.fault == FakeMapArtifactFault::QualityNotGood, "bad quality fail");
    return failures ? 1 : 0;
}
