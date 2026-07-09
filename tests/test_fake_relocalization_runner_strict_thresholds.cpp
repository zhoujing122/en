#include "robot_slamd/autonomy/fake_relocalization/fake_map_relocalization_runner.hpp"
#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_binding.hpp"
#include <iostream>
#include <limits>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
static robot_slamd::FakeMapArtifact artifact(double yaw) {
    robot_slamd::FakeMapArtifact a;
    a.status = robot_slamd::FakeMapArtifactStatus::Loaded;
    a.fault = robot_slamd::FakeMapArtifactFault::None;
    a.metadata.map_id = "strict_map";
    a.metadata.coverage_ratio = 1.0;
    a.metadata.yaw_coverage_ratio = yaw;
    a.metadata.valid_scan_count = 3;
    a.final_quality.good_enough = true;
    a.final_quality.coverage_ratio = 1.0;
    a.final_quality.yaw_coverage_ratio = yaw;
    a.final_quality.valid_scan_count = 3;
    a.serialized_text = "fake_map_artifact\nmap_id=strict_map\n";
    return a;
}
static robot_slamd::RobotSlamSensorSnapshot snapshot() {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_tof = true;
    s.tof.ranges_m = {1.0, 1.2, std::numeric_limits<double>::quiet_NaN(), 1.5};
    return s;
}
int main() {
    using namespace robot_slamd;
    auto strict = strict_fake_map_relocalization_options();
    expect(strict.min_map_yaw_coverage_ratio >= 0.30, "default yaw threshold strict");
    FakeMapRelocalizationRunnerOptions options;
    options.enabled = true;
    FakeMapRelocalizationRunner runner(options);
    const auto report = runner.run_once();
    expect(report.ok, "strict runner success");
    expect(report.pipeline_report.fake_map_artifact.final_quality.yaw_coverage_ratio >= 0.30, "success map yaw coverage strict");
    expect(report.relocalization_result.ok, "strict relocalization ok");
    FakeRelocalizationBinding binding(strict);
    auto low_yaw = binding.relocalize(artifact(0.10), snapshot());
    expect(!low_yaw.ok && low_yaw.fault == FakeRelocalizationFault::MapQualityTooLow, "low yaw fails");
    options.use_strict_default_relocalization_options = false;
    options.relocalization_options = strict;
    options.relocalization_options.min_map_yaw_coverage_ratio = 0.30 - 0.01;
    FakeMapRelocalizationRunner relaxed(options);
    const auto relaxed_report = relaxed.run_once();
    expect(!relaxed_report.ok, "relaxed threshold rejected");
    expect(relaxed_report.summary == "fake_map_relocalization_yaw_threshold_relaxed", "relaxed summary");
    return failures ? 1 : 0;
}
