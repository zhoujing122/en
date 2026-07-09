#include "robot_slamd/autonomy/fake_relocalization/fake_map_relocalization_runner.hpp"
#include <iostream>
#include <limits>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
static robot_slamd::FakeMapArtifact map(double coverage, double yaw, bool good = true) {
    robot_slamd::FakeMapArtifact artifact;
    artifact.status = robot_slamd::FakeMapArtifactStatus::Loaded;
    artifact.fault = robot_slamd::FakeMapArtifactFault::None;
    artifact.metadata.map_id = "fake_map_failure";
    artifact.metadata.coverage_ratio = coverage;
    artifact.metadata.yaw_coverage_ratio = yaw;
    artifact.metadata.valid_scan_count = 3;
    artifact.final_quality.good_enough = good;
    artifact.final_quality.coverage_ratio = coverage;
    artifact.final_quality.yaw_coverage_ratio = yaw;
    artifact.final_quality.valid_scan_count = 3;
    artifact.serialized_text = "fake_map_artifact\nmap_id=fake_map_failure\n";
    return artifact;
}
static robot_slamd::RobotSlamSensorSnapshot snapshot() {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_tof = true;
    s.tof.ranges_m = {1.0, 1.2, std::numeric_limits<double>::quiet_NaN(), 1.5};
    return s;
}
int main() {
    using namespace robot_slamd;
    FakeMapRelocalizationRunner disabled;
    expect(!disabled.run_once().ok, "disabled runner fail");
    FakeRelocalizationOptions options; options.enabled = true;
    FakeRelocalizationBinding binding(options);
    auto missing_artifact = binding.relocalize(FakeMapArtifact{}, snapshot());
    expect(!missing_artifact.ok && missing_artifact.fault == FakeRelocalizationFault::InvalidMapArtifact, "missing artifact");
    auto bad_quality = binding.relocalize(map(0.2, 0.1, false), snapshot());
    expect(!bad_quality.ok && bad_quality.fault == FakeRelocalizationFault::MapQualityTooLow, "bad quality");
    RobotSlamSensorSnapshot missing_tof;
    auto no_tof = binding.relocalize(map(0.9, 0.5), missing_tof);
    expect(!no_tof.ok && no_tof.fault == FakeRelocalizationFault::MissingTof, "missing tof");
    options.min_confidence = 0.95;
    FakeRelocalizationBinding strict(options);
    auto low_confidence = strict.relocalize(map(0.7, 0.3), snapshot());
    expect(!low_confidence.ok && low_confidence.fault == FakeRelocalizationFault::ConfidenceTooLow, "confidence low");
    options.allow_pose_writeback = true;
    FakeRelocalizationBinding writeback(options);
    auto writeback_result = writeback.relocalize(map(0.9, 0.5), snapshot());
    expect(!writeback_result.ok && writeback_result.fault == FakeRelocalizationFault::PoseWritebackForbidden, "writeback forbidden");
    return failures ? 1 : 0;
}
