#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_runner.hpp"

#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_sensor_reject(const robot_slamd::MultiTofFullPipelineReport &report,
                          robot_slamd::MultiTofReplayFault fault,
                          const char *label) {
    using namespace robot_slamd;
    expect(!report.ok, label);
    expect(report.fault == MultiTofFullPipelineFault::SensorSnapshotMissing,
           "sensor reject maps to snapshot missing");
    expect(report.last_replay_fault == fault, "specific replay fault");
    expect(report.full_pipeline.backend_accepted_update_count == 0,
           "backend did not accept rejected packet");
    expect(report.full_pipeline.shadow_motion_command_count == 0,
           "motion not sent for rejected packet");
    expect(!report.stale_snapshot_reused, "no stale snapshot reuse");
    expect(!report.real_hardware_accessed, "no real hardware");
    expect(!report.forward_seen, "no forward");
    expect(!report.backward_seen, "no backward");
}
}

int main() {
    using namespace robot_slamd;
    MultiTofFullPipelineScenarioBuilder builder;
    MultiTofFullPipelineRunner runner;

    expect_sensor_reject(
        runner.run_once(builder.build(MultiTofFullPipelineScenarioKind::MissingRightRequireAll)),
        MultiTofReplayFault::SnapshotBuildFailed,
        "missing right require all fails");

    const auto low_confidence = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::LowConfidenceFront));
    expect_sensor_reject(low_confidence,
                         MultiTofReplayFault::SnapshotBuildFailed,
                         "low confidence front fails legacy projection");
    expect(!low_confidence.last_success_snapshot.has_tof,
           "low confidence did not create success legacy snapshot");

    expect_sensor_reject(
        runner.run_once(builder.build(MultiTofFullPipelineScenarioKind::InvalidPayloadLength)),
        MultiTofReplayFault::InvalidRecord,
        "invalid payload length fails");

    expect_sensor_reject(
        runner.run_once(builder.build(MultiTofFullPipelineScenarioKind::InvalidHexPayload)),
        MultiTofReplayFault::InvalidRecord,
        "invalid hex payload fails");

    expect_sensor_reject(
        runner.run_once(builder.build(MultiTofFullPipelineScenarioKind::DistanceAboveProtocolMaximum)),
        MultiTofReplayFault::SnapshotBuildFailed,
        "distance 12001 fails");

    expect_sensor_reject(
        runner.run_once(builder.build(MultiTofFullPipelineScenarioKind::PairwiseSyncFailure)),
        MultiTofReplayFault::SnapshotBuildFailed,
        "pairwise sync fails");

    expect_sensor_reject(
        runner.run_once(builder.build(MultiTofFullPipelineScenarioKind::ImuSyncFailure)),
        MultiTofReplayFault::SnapshotBuildFailed,
        "imu sync fails");

    expect_sensor_reject(
        runner.run_once(builder.build(MultiTofFullPipelineScenarioKind::WheelSyncFailure)),
        MultiTofReplayFault::SnapshotBuildFailed,
        "wheel sync fails");

    const auto ended = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::EndOfReplayEarly));
    expect(!ended.ok, "end early fails deterministically");
    expect(ended.last_replay_fault == MultiTofReplayFault::EndOfReplay,
           "end replay fault preserved");
    expect(ended.end_of_replay_return_count >= 1, "end count");
    expect(!ended.stale_snapshot_reused, "end does not reuse snapshot");

    const auto null_sensor = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::NullSensorPort));
    expect(!null_sensor.ok, "nullptr sensor fails");
    expect(null_sensor.fault == MultiTofFullPipelineFault::SensorReplayNotReady,
           "nullptr not ready fault");

    const auto not_ready = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::NotReadySensorPort));
    expect(!not_ready.ok, "not-ready sensor fails");
    expect(not_ready.fault == MultiTofFullPipelineFault::SensorReplayNotReady,
           "not-ready fault");

    const auto backend = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::BackendReject));
    expect(!backend.ok, "backend reject fails");
    expect(backend.fault == MultiTofFullPipelineFault::BackendRejected,
           "backend reject fault");
    expect(backend.full_pipeline.backend_rejected_update_count > 0,
           "backend rejected update counted");
    expect(!backend.real_hardware_accessed, "backend reject no hardware");

    return failures ? 1 : 0;
}
