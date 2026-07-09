#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_log_codec.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/regression/real_sensor_replay_regression_types.hpp"

#include <utility>

namespace robot_slamd {

class RealSensorReplayRegressionRunner {
public:
    explicit RealSensorReplayRegressionRunner(
        RealSensorReplayRegressionOptions options = {})
        : options_(std::move(options)) {}

    RealSensorReplayRegressionReport run_once() const {
        RealSensorReplayRegressionReport report;
        report.status = RealSensorReplayRegressionStatus::Running;
        const auto cases = default_cases();
        report.case_count = static_cast<int>(cases.size());
        for (const auto &test_case : cases) {
            if (run_case(test_case, report)) {
                report.pass_count++;
            } else {
                report.fail_count++;
            }
        }
        report.ok = report.fail_count == 0;
        report.status = report.ok ? RealSensorReplayRegressionStatus::Passed
                                  : RealSensorReplayRegressionStatus::Failed;
        report.summary = report.ok ? "real_sensor_replay_regression_passed"
                                   : "real_sensor_replay_regression_failed";
        return report;
    }

private:
    std::vector<RealSensorReplayRegressionCase> default_cases() const {
        return {{RealSensorReplayRegressionCaseKind::ValidShortLog,
                 "valid_short_log",
                 RealSensorReplaySampleLog::valid_log_lines(),
                 true},
                {RealSensorReplayRegressionCaseKind::InvalidLatencyLog,
                 "invalid_latency_log",
                 RealSensorReplaySampleLog::invalid_latency_log_lines(),
                 false},
                {RealSensorReplayRegressionCaseKind::SyncTooLargeLog,
                 "sync_too_large_log",
                 RealSensorReplaySampleLog::sync_too_large_log_lines(),
                 false},
                {RealSensorReplayRegressionCaseKind::ParseErrorLog,
                 "parse_error_log",
                 {"kind=packet,packet_time=abc"},
                 false},
                {RealSensorReplayRegressionCaseKind::CommentOnlyLog,
                 "comment_only_log",
                 {"# only comments", "# no packets"},
                 false}};
    }

    bool run_case(
        const RealSensorReplayRegressionCase &test_case,
        RealSensorReplayRegressionReport &report) const {
        RealSensorReplayLogCodec codec;
        const auto log = codec.parse_lines(test_case.log_lines,
                                           test_case.name);
        const auto validation = codec.validate_log(log);
        RealSensorReplayPort port(log);
        bool observed_pass = false;
        if (port.ready()) {
            port.latest_snapshot(0.0);
            const auto replay_result = port.result();
            observed_pass = replay_result.accepted_packet_count > 0 &&
                            replay_result.rejected_packet_count == 0;
        }

        // Parse-error and comment-only cases must be rejected before replay.
        if (test_case.kind == RealSensorReplayRegressionCaseKind::ParseErrorLog &&
            validation.invalid_record_count > 0) {
            report.passed.push_back(test_case.name + "_parse_error_detected");
        }
        if (test_case.kind == RealSensorReplayRegressionCaseKind::ParseErrorLog &&
            validation.invalid_record_count <= 0) {
            report.block_reason =
                RealSensorReplayRegressionBlockReason::ParseErrorNotDetected;
            report.failed.push_back(test_case.name + ":parse_error_not_detected");
            return false;
        }
        if (test_case.kind == RealSensorReplayRegressionCaseKind::CommentOnlyLog &&
            validation.fault == RealSensorReplayFault::NoPacketRecords) {
            report.passed.push_back(test_case.name + "_matched_expectation");
        }
        if (test_case.kind == RealSensorReplayRegressionCaseKind::CommentOnlyLog &&
            validation.fault != RealSensorReplayFault::NoPacketRecords) {
            report.block_reason =
                RealSensorReplayRegressionBlockReason::NoPacketRecords;
            report.failed.push_back(test_case.name + ":comment_only_not_rejected");
            return false;
        }

        if (observed_pass == test_case.expect_pass) {
            report.passed.push_back(test_case.name);
            return true;
        }
        report.block_reason = observed_pass
                                  ? RealSensorReplayRegressionBlockReason::UnexpectedPass
                                  : RealSensorReplayRegressionBlockReason::UnexpectedFail;
        report.failed.push_back(test_case.name);
        return false;
    }

    RealSensorReplayRegressionOptions options_;
};

} // namespace robot_slamd
