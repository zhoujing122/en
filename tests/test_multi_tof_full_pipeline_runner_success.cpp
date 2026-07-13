#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_runner.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
void expect_near(double actual, double expected, double tol, const char *message) {
    if (std::fabs(actual - expected) > tol) {
        std::cerr << "FAIL: " << message << " actual=" << actual
                  << " expected=" << expected << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    MultiTofFullPipelineScenarioBuilder builder;
    MultiTofFullPipelineRunner runner;

    const auto normal = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::NormalRequireAll));
    expect(normal.ok, "normal pipeline ok");
    expect(normal.full_pipeline.final_state == AutonomousSlamState::Completed,
           "normal completed");
    expect(normal.full_pipeline.backend_accepted_update_count >= 2,
           "backend accepted updates");
    expect(normal.multi_tof_snapshot_count >= 2, "multi tof reached runner");
    expect(normal.multi_tof_reached_coordinator, "multi tof reached coordinator");
    expect(normal.legacy_projection_reached_backend,
           "legacy projection reached backend");
    expect(normal.maximum_legacy_projection_range_count == 1,
           "legacy range count one");
    expect(normal.has_last_success_snapshot, "has success snapshot");
    const auto &snapshot = normal.last_success_snapshot;
    expect(snapshot.has_multi_tof, "snapshot has multi tof");
    expect(snapshot.has_tof, "snapshot has legacy tof");
    expect(snapshot.tof.ranges_m.size() == 1, "legacy single range");
    expect(snapshot.tof.source.find("legacy_scalar_tof_projection") != std::string::npos,
           "legacy source");
    expect(snapshot.multi_tof.has_front, "front present");
    expect(snapshot.multi_tof.has_left, "left present");
    expect(snapshot.multi_tof.has_right, "right present");
    expect(snapshot.multi_tof.front.confidence >= 70, "front confidence kept");
    expect(snapshot.multi_tof.front.echo_tag_u48 != 0, "echo tag kept");
    expect(snapshot.multi_tof.front.usable_for_mapping, "front mapping usable");
    expect_near(snapshot.multi_tof.front.full_fov_rad,
                1.6 * 3.14159265358979323846 / 180.0,
                1e-6,
                "full fov kept");
    expect_near(snapshot.tof.timestamp_s,
                snapshot.multi_tof.front.effective_timestamp_s,
                1e-9,
                "legacy timestamp equals front effective time");
    expect(normal.scalar_tof_codec_used, "codec path used");
    expect(normal.echo_tag_preserved, "echo tag report");
    expect(normal.confidence_preserved, "confidence report");
    expect(normal.effective_timestamp_preserved, "timestamp report");
    expect(normal.synchronized_time_verified, "sync time report");
    expect(normal.fake_motion_only, "fake motion only");
    expect(!normal.native_multi_tof_backend_consumption,
           "native backend false");
    expect(!normal.real_hardware_accessed, "no real hardware");
    expect(!normal.real_motion_attempted, "no real motion");
    expect(!normal.real_map_write_attempted, "no real map write");
    expect(!normal.real_pose_writeback_attempted, "no pose writeback");

    const auto active = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::ActiveScanRequired));
    expect(active.ok, "active scan pipeline ok");
    expect(active.full_pipeline.active_scan_command_count > 0,
           "active scan triggered");
    expect(active.full_pipeline.sensor_skipped_count > 0,
           "phase-aware skipped sensors");
    expect(active.phase_aware_sensor_consumption_verified,
           "phase-aware report");
    expect(active.replay_read_call_count == active.successful_snapshot_count,
           "no reads consumed during motion wait");
    expect(!active.forward_seen, "no forward");
    expect(!active.backward_seen, "no backward");

    bool skipped_motion_phase = false;
    for (const auto &event : active.full_pipeline.trace.events) {
        if (event.kind == FullAutonomousSlamTraceEventKind::SensorSkipped &&
            (event.state == AutonomousSlamState::SendingMotionCommand ||
             event.state == AutonomousSlamState::WaitingMotionSettle)) {
            skipped_motion_phase = true;
        }
    }
    expect(skipped_motion_phase, "motion wait skipped replay consumption");

    const auto degraded = runner.run_once(
        builder.build(MultiTofFullPipelineScenarioKind::AllowMissingRight));
    expect(degraded.ok, "allow missing right passes");
    expect(degraded.last_success_snapshot.has_multi_tof, "degraded multi tof");
    expect(degraded.last_success_snapshot.has_tof, "degraded legacy tof");
    expect(degraded.last_success_snapshot.multi_tof.has_front, "degraded front");
    expect(degraded.last_success_snapshot.multi_tof.has_left, "degraded left");
    expect(!degraded.last_success_snapshot.multi_tof.has_right, "right missing");
    expect(degraded.last_success_snapshot.multi_tof.degraded, "degraded flag");
    expect(degraded.last_success_snapshot.multi_tof.valid_tof_count == 2,
           "valid tof count two");
    expect(degraded.degraded_snapshot_count > 0, "degraded count reported");

    return failures ? 1 : 0;
}
