#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_binding.hpp"
#include <iostream>
#include <limits>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
static robot_slamd::FakeMapArtifact valid_map() {
    robot_slamd::FakeMapArtifact map;
    map.status = robot_slamd::FakeMapArtifactStatus::Saved;
    map.fault = robot_slamd::FakeMapArtifactFault::None;
    map.metadata.map_id = "fake_map_a";
    map.metadata.coverage_ratio = 0.9;
    map.metadata.yaw_coverage_ratio = 0.5;
    map.metadata.valid_scan_count = 4;
    map.metadata.active_scan_command_count = 1;
    map.final_quality.good_enough = true;
    map.final_quality.coverage_ratio = 0.9;
    map.final_quality.yaw_coverage_ratio = 0.5;
    map.final_quality.valid_scan_count = 4;
    map.serialized_text = "fake_map_artifact\nmap_id=fake_map_a\n";
    return map;
}
static robot_slamd::RobotSlamSensorSnapshot valid_snapshot() {
    robot_slamd::RobotSlamSensorSnapshot snapshot;
    snapshot.has_tof = true;
    snapshot.tof.timestamp_s = 100.0;
    snapshot.tof.ranges_m = {1.0, 1.2, std::numeric_limits<double>::quiet_NaN(), 1.5};
    snapshot.tof.angle_increment_rad = 0.1;
    snapshot.tof.max_range_m = 4.0;
    snapshot.tof.frame_id = "tof_frame";
    return snapshot;
}
int main() {
    using namespace robot_slamd;
    FakeRelocalizationBinding disabled;
    expect(!disabled.ready(), "default not ready");
    FakeRelocalizationOptions options; options.enabled = true;
    FakeRelocalizationBinding binding(options);
    expect(binding.ready(), "enabled ready");
    auto ok = binding.relocalize(valid_map(), valid_snapshot());
    expect(ok.ok, "accepted");
    expect(ok.pose.valid, "pose valid");
    expect(ok.pose.confidence >= options.min_confidence, "confidence");
    expect(ok.summary == "fake_relocalization_accepted", "summary");
    FakeMapArtifact missing;
    auto missing_map = binding.relocalize(missing, valid_snapshot());
    expect(!missing_map.ok && missing_map.fault == FakeRelocalizationFault::InvalidMapArtifact, "missing map");
    auto bad_map = valid_map(); bad_map.final_quality.good_enough = false;
    auto bad_quality = binding.relocalize(bad_map, valid_snapshot());
    expect(!bad_quality.ok && bad_quality.fault == FakeRelocalizationFault::MapQualityTooLow, "bad map quality");
    RobotSlamSensorSnapshot no_tof;
    auto missing_tof = binding.relocalize(valid_map(), no_tof);
    expect(!missing_tof.ok && missing_tof.fault == FakeRelocalizationFault::MissingTof, "missing tof");
    auto few = valid_snapshot(); few.tof.ranges_m = {1.0, std::numeric_limits<double>::quiet_NaN()};
    auto few_result = binding.relocalize(valid_map(), few);
    expect(!few_result.ok && few_result.fault == FakeRelocalizationFault::TooFewValidRanges, "few ranges");
    options.allow_pose_writeback = true;
    FakeRelocalizationBinding writeback_forbidden(options);
    auto writeback = writeback_forbidden.relocalize(valid_map(), valid_snapshot());
    expect(!writeback.ok && writeback.fault == FakeRelocalizationFault::PoseWritebackForbidden, "writeback forbidden");
    expect(!writeback.pose.valid, "no pose writeback");
    return failures ? 1 : 0;
}
