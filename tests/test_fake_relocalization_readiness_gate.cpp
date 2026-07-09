#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_readiness_gate.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
static robot_slamd::FakeRelocalizationResult valid_result() {
    robot_slamd::FakeRelocalizationResult r;
    r.ok = true;
    r.status = robot_slamd::FakeRelocalizationStatus::Accepted;
    r.pose.valid = true;
    r.pose.confidence = 0.80;
    r.map_artifact.final_quality.good_enough = true;
    r.map_artifact.final_quality.coverage_ratio = 0.80;
    r.map_artifact.final_quality.yaw_coverage_ratio = 0.40;
    return r;
}
int main() {
    using namespace robot_slamd;
    FakeRelocalizationReadinessGate disabled;
    expect(!disabled.check(valid_result(), false).ok, "disabled blocked");
    FakeRelocalizationReadinessOptions options; options.enabled = true;
    FakeRelocalizationReadinessGate gate(options);
    auto rejected = valid_result(); rejected.ok = false;
    expect(gate.check(rejected, false).block_reason == FakeRelocalizationReadinessBlockReason::RelocalizationRejected, "rejected blocked");
    auto invalid_pose = valid_result(); invalid_pose.pose.valid = false;
    expect(gate.check(invalid_pose, false).block_reason == FakeRelocalizationReadinessBlockReason::PoseCandidateInvalid, "pose invalid blocked");
    expect(gate.check(valid_result(), true).block_reason == FakeRelocalizationReadinessBlockReason::PoseWritebackAttempted, "writeback blocked");
    auto low_cov = valid_result(); low_cov.map_artifact.final_quality.coverage_ratio = 0.20;
    expect(gate.check(low_cov, false).block_reason == FakeRelocalizationReadinessBlockReason::MapQualityTooLow, "coverage blocked");
    auto low_yaw = valid_result(); low_yaw.map_artifact.final_quality.yaw_coverage_ratio = 0.10;
    expect(gate.check(low_yaw, false).block_reason == FakeRelocalizationReadinessBlockReason::YawCoverageTooLow, "yaw blocked");
    auto ready = gate.check(valid_result(), false);
    expect(ready.ok && ready.summary == "fake_relocalization_readiness_ready", "ready");
    return failures ? 1 : 0;
}
