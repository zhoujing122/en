#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_to_snapshot_regression_runner.hpp"

namespace robot_slamd {

struct MultiTofReplayAcceptanceReport {
    bool ok = false;
    int record_count = 0;
    int packet_count = 0;
    int invalid_record_count = 0;
    int consumed_packet_count = 0;
    MultiTofReplayReadResult last_result;
    int case_count = 0;
    int passed_case_count = 0;
    int failed_case_count = 0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class MultiTofReplayAcceptanceRunner {
public:
    MultiTofReplayAcceptanceReport run_once() const {
        MultiTofReplayAcceptanceReport report;
        MultiTofReplayLogCodec codec;
        MultiTofSyncOptions sync;
        sync.enabled = true;
        MultiTofSnapshotBuildOptions snapshot;
        snapshot.enabled = true;
        MultiTofReplayOptions replay;
        replay.enabled = true;

        run_case("valid_log", codec.decode_lines(MultiTofReplaySampleLog::valid_log_lines()),
                 true, {}, sync, snapshot, replay, report);
        run_case("invalid_numeric", codec.decode_lines(MultiTofReplaySampleLog::invalid_numeric_log_lines()),
                 false, {}, sync, snapshot, replay, report);
        run_case("missing_field", codec.decode_lines(MultiTofReplaySampleLog::missing_field_log_lines()),
                 false, {}, sync, snapshot, replay, report);
        run_case("invalid_payload", codec.decode_lines(MultiTofReplaySampleLog::invalid_payload_log_lines()),
                 false, {}, sync, snapshot, replay, report);
        run_case("pairwise_sync_failure", codec.decode_lines(MultiTofReplaySampleLog::pairwise_sync_failure_log_lines()),
                 false, {}, sync, snapshot, replay, report);
        run_case("imu_sync_failure", codec.decode_lines(MultiTofReplaySampleLog::imu_sync_failure_log_lines()),
                 false, {}, sync, snapshot, replay, report);
        run_case("wheel_sync_failure", codec.decode_lines(MultiTofReplaySampleLog::wheel_sync_failure_log_lines()),
                 false, {}, sync, snapshot, replay, report);

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
        run_case("degraded_missing_left", codec.decode_lines(MultiTofReplaySampleLog::degraded_missing_left_log_lines()),
                 true, degraded_contract, degraded_sync, degraded_snapshot, replay, report);

        report.ok = report.failed.empty();
        report.summary = report.ok ? "multi_tof_replay_acceptance_passed"
                                   : "multi_tof_replay_acceptance_failed";
        return report;
    }

private:
    static void run_case(
        const std::string &name,
        const std::vector<MultiTofReplayRecord> &records,
        bool expect_pass,
        const MultiTofContractOptions &contract,
        const MultiTofSyncOptions &sync,
        const MultiTofSnapshotBuildOptions &snapshot,
        const MultiTofReplayOptions &replay,
        MultiTofReplayAcceptanceReport &report) {
        report.case_count++;
        MultiTofReplayPort port(records, contract, sync, snapshot, replay);
        const auto result = port.latest_snapshot(100.0);
        report.record_count = static_cast<int>(records.size());
        report.packet_count = port.packet_count();
        report.invalid_record_count = port.invalid_record_count();
        report.consumed_packet_count = port.consumed_packet_count();
        report.last_result = result;
        if (result.ok == expect_pass) {
            report.passed_case_count++;
            report.passed.push_back(name);
        } else {
            report.failed_case_count++;
            report.failed.push_back(name);
        }
    }
};

} // namespace robot_slamd
