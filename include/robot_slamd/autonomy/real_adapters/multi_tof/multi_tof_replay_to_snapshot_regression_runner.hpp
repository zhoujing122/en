#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sample_log.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct MultiTofReplayToSnapshotRegressionReport {
    bool ok = false;
    MultiTofReplayReadResult valid_result;
    MultiTofReplayReadResult degraded_result;
    MultiTofReplayReadResult rejected_result;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class MultiTofReplayToSnapshotRegressionRunner {
public:
    MultiTofReplayToSnapshotRegressionReport run_once() const {
        MultiTofReplayToSnapshotRegressionReport report;
        MultiTofReplayLogCodec codec;
        MultiTofSyncOptions sync;
        sync.enabled = true;
        MultiTofSnapshotBuildOptions snapshot;
        snapshot.enabled = true;
        MultiTofReplayOptions replay;
        replay.enabled = true;
        MultiTofReplayPort port(codec.decode_lines(MultiTofReplaySampleLog::valid_log_lines()),
                                {}, sync, snapshot, replay);
        report.valid_result = port.latest_snapshot(100.0);
        check(report.valid_result.ok && report.valid_result.snapshot.has_multi_tof,
              "valid_log_has_multi_tof", report);
        check(report.valid_result.snapshot.multi_tof.valid_tof_count == 3,
              "valid_tof_count_3", report);
        check(report.valid_result.snapshot.has_tof,
              "legacy_tof_present", report);
        check(report.valid_result.snapshot.multi_tof.front.frame_id == "tof_front_frame" &&
                  report.valid_result.snapshot.multi_tof.left.frame_id == "tof_left_frame" &&
                  report.valid_result.snapshot.multi_tof.right.frame_id == "tof_right_frame",
              "frame_ids_preserved", report);

        MultiTofContractOptions degraded_contract;
        degraded_contract.require_left = false;
        degraded_contract.min_required_tof_count = 2;
        MultiTofSyncOptions degraded_sync;
        degraded_sync.enabled = true;
        degraded_sync.degradation_mode = MultiTofDegradationMode::AllowMissingOne;
        degraded_sync.min_required_tof_count = 2;
        MultiTofSnapshotBuildOptions degraded_snapshot;
        degraded_snapshot.enabled = true;
        degraded_snapshot.min_required_tof_count = 2;
        MultiTofReplayPort degraded_port(
            codec.decode_lines(MultiTofReplaySampleLog::degraded_missing_left_log_lines()),
            degraded_contract,
            degraded_sync,
            degraded_snapshot,
            replay);
        report.degraded_result = degraded_port.latest_snapshot(100.0);
        check(report.degraded_result.ok && report.degraded_result.snapshot.multi_tof.degraded,
              "degraded_missing_left_passed", report);

        MultiTofReplayPort rejected_port(
            codec.decode_lines(MultiTofReplaySampleLog::pairwise_sync_failure_log_lines()),
            {}, sync, snapshot, replay);
        report.rejected_result = rejected_port.latest_snapshot(100.0);
        check(!report.rejected_result.ok,
              "pairwise_sync_failure_rejected", report);

        report.ok = report.failed.empty();
        report.summary = report.ok ? "multi_tof_replay_to_snapshot_regression_passed"
                                   : "multi_tof_replay_to_snapshot_regression_failed";
        return report;
    }

private:
    static void check(
        bool condition,
        const std::string &name,
        MultiTofReplayToSnapshotRegressionReport &report) {
        if (condition) {
            report.passed.push_back(name);
        } else {
            report.failed.push_back(name);
        }
    }
};

} // namespace robot_slamd
