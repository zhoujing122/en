#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_scenario_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sample_log.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sensor_port.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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

robot_slamd::MultiTofReplaySensorPort make_port(
    const std::vector<robot_slamd::MultiTofReplayRecord> &records,
    robot_slamd::MultiTofReplayOptions replay_options = {}) {
    using namespace robot_slamd;
    if (!replay_options.enabled) {
        replay_options.enabled = true;
    }
    replay_options.reject_invalid_records = true;
    replay_options.require_snapshot_build = true;
    replay_options.time_mode = MultiTofReplayTimeMode::RecordPacketTime;
    MultiTofContractOptions contract_options;
    MultiTofSyncOptions sync_options;
    sync_options.enabled = true;
    MultiTofSnapshotBuildOptions snapshot_options;
    snapshot_options.enabled = true;
    return MultiTofReplaySensorPort(MultiTofReplayPort(records,
                                                       contract_options,
                                                       sync_options,
                                                       snapshot_options,
                                                       replay_options));
}

std::vector<robot_slamd::MultiTofReplayRecord> records_from_lines(
    const std::vector<std::string> &lines) {
    return robot_slamd::MultiTofReplayLogCodec().decode_lines(lines);
}
} // namespace

int main() {
    using namespace robot_slamd;

    MultiTofReplayOptions disabled_options;
    disabled_options.enabled = false;
    MultiTofReplaySensorPort disabled(MultiTofReplayPort(
        records_from_lines(MultiTofReplaySampleLog::valid_log_lines()),
        {},
        MultiTofSyncOptions{},
        MultiTofSnapshotBuildOptions{},
        disabled_options));
    expect(!disabled.ready(), "disabled not ready");

    MultiTofReplayOptions replay_options;
    replay_options.enabled = true;
    replay_options.reject_invalid_records = true;
    replay_options.require_snapshot_build = true;
    MultiTofReplaySensorPort empty(MultiTofReplayPort(
        {}, {}, MultiTofSyncOptions{}, MultiTofSnapshotBuildOptions{}, replay_options));
    expect(!empty.ready(), "empty log not ready");

    auto port = make_port(records_from_lines(MultiTofReplaySampleLog::valid_log_lines()));
    expect(port.ready(), "port ready");
    const auto first = port.latest_snapshot(0.0);
    expect(port.read_call_count() == 1, "read call counted");
    expect(port.successful_snapshot_count() == 1, "success counted");
    expect(port.consumed_packet_count() == 1, "comment did not count as packet");
    expect(first.has_multi_tof, "snapshot has multi tof");
    expect(first.has_tof, "snapshot has legacy tof");
    expect(first.multi_tof.has_front, "front present");
    expect(first.multi_tof.has_left, "left present");
    expect(first.multi_tof.has_right, "right present");
    expect(first.multi_tof.front.distance_mm == 2731, "front distance mm");
    expect(first.multi_tof.front.confidence == 100, "front confidence");
    expect(first.multi_tof.front.echo_tag_u48 == 0x000044332211ULL,
           "front echo tag preserved");
    expect_near(first.multi_tof.front.effective_timestamp_s, 100.000, 1e-9,
                "front effective timestamp");
    expect_near(first.multi_tof.front.full_fov_rad,
                1.6 * 3.14159265358979323846 / 180.0,
                1e-6,
                "full fov 1.6 deg");
    expect(first.tof.ranges_m.size() == 1, "legacy projection single range");
    expect(first.tof.source.find("legacy_scalar_tof_projection") != std::string::npos,
           "legacy projection source");

    const auto ended = port.latest_snapshot(0.0);
    expect(!snapshot_has_any_payload(ended), "end returns empty snapshot");
    expect(port.end_of_replay_return_count() == 1, "end counted");
    expect(port.last_read_result().fault == MultiTofReplayFault::EndOfReplay,
           "end fault deterministic");
    const auto ended_again = port.latest_snapshot(0.0);
    expect(!snapshot_has_any_payload(ended_again), "second end still empty");
    expect(port.end_of_replay_return_count() == 2, "second end counted");

    auto mixed_lines = MultiTofReplaySampleLog::pairwise_sync_failure_log_lines();
    const auto valid_lines = MultiTofReplaySampleLog::valid_log_lines();
    mixed_lines.insert(mixed_lines.end(), valid_lines.begin(), valid_lines.end());
    auto mixed = make_port(records_from_lines(mixed_lines));
    const auto rejected = mixed.latest_snapshot(0.0);
    expect(!snapshot_has_any_payload(rejected), "sync failure empty");
    expect(mixed.rejected_snapshot_count() == 1, "sync failure counted");
    expect(mixed.last_read_result().fault == MultiTofReplayFault::SnapshotBuildFailed,
           "sync failure exposed");
    const auto ok_after_reject = mixed.latest_snapshot(0.0);
    expect(ok_after_reject.has_multi_tof, "valid packet after reject works");
    expect(ok_after_reject.multi_tof.front.echo_tag_u48 == 0x000044332211ULL,
           "valid after reject not polluted");

    auto invalid_lines = MultiTofFullPipelineScenarioBuilder()
        .build(MultiTofFullPipelineScenarioKind::InvalidHexPayload)
        .records;
    auto invalid = make_port(invalid_lines);
    expect(invalid.ready(), "invalid record with later packet ready");
    const auto invalid_snapshot = invalid.latest_snapshot(0.0);
    expect(!snapshot_has_any_payload(invalid_snapshot), "invalid record empty");
    expect(invalid.last_read_result().fault == MultiTofReplayFault::InvalidRecord,
           "invalid record fault");
    expect(invalid.status().find("last_fault=invalid_record") != std::string::npos,
           "status contains last fault");

    return failures ? 1 : 0;
}
