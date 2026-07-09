#pragma once

#include <string>
#include <vector>

namespace robot_slamd {

enum class RealSensorReplayRegressionCaseKind {
    ValidShortLog,
    InvalidLatencyLog,
    SyncTooLargeLog,
    ParseErrorLog,
    CommentOnlyLog
};

enum class RealSensorReplayRegressionStatus {
    NotStarted,
    Running,
    Passed,
    Failed,
    Blocked
};

enum class RealSensorReplayRegressionBlockReason {
    None,
    ReplayRejected,
    UnexpectedPass,
    UnexpectedFail,
    ParseErrorNotDetected,
    NoPacketRecords,
    ReportInvalid
};

struct RealSensorReplayRegressionCase {
    RealSensorReplayRegressionCaseKind kind =
        RealSensorReplayRegressionCaseKind::ValidShortLog;
    std::string name;
    std::vector<std::string> log_lines;
    bool expect_pass = true;
};

struct RealSensorReplayRegressionOptions {
    bool enabled = false;
    bool require_valid_log_pass = true;
    bool require_invalid_logs_fail = true;
    bool require_parse_errors_detected = true;
    bool require_comment_only_log_rejected = true;
};

struct RealSensorReplayRegressionReport {
    bool ok = false;
    RealSensorReplayRegressionStatus status =
        RealSensorReplayRegressionStatus::NotStarted;
    RealSensorReplayRegressionBlockReason block_reason =
        RealSensorReplayRegressionBlockReason::None;
    int case_count = 0;
    int pass_count = 0;
    int fail_count = 0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(RealSensorReplayRegressionCaseKind kind) {
    switch (kind) {
    case RealSensorReplayRegressionCaseKind::ValidShortLog:
        return "ValidShortLog";
    case RealSensorReplayRegressionCaseKind::InvalidLatencyLog:
        return "InvalidLatencyLog";
    case RealSensorReplayRegressionCaseKind::SyncTooLargeLog:
        return "SyncTooLargeLog";
    case RealSensorReplayRegressionCaseKind::ParseErrorLog:
        return "ParseErrorLog";
    case RealSensorReplayRegressionCaseKind::CommentOnlyLog:
        return "CommentOnlyLog";
    }
    return "Unknown";
}

inline std::string to_string(RealSensorReplayRegressionStatus status) {
    switch (status) {
    case RealSensorReplayRegressionStatus::NotStarted:
        return "NotStarted";
    case RealSensorReplayRegressionStatus::Running:
        return "Running";
    case RealSensorReplayRegressionStatus::Passed:
        return "Passed";
    case RealSensorReplayRegressionStatus::Failed:
        return "Failed";
    case RealSensorReplayRegressionStatus::Blocked:
        return "Blocked";
    }
    return "Unknown";
}

inline std::string to_string(
    RealSensorReplayRegressionBlockReason reason) {
    switch (reason) {
    case RealSensorReplayRegressionBlockReason::None:
        return "None";
    case RealSensorReplayRegressionBlockReason::ReplayRejected:
        return "ReplayRejected";
    case RealSensorReplayRegressionBlockReason::UnexpectedPass:
        return "UnexpectedPass";
    case RealSensorReplayRegressionBlockReason::UnexpectedFail:
        return "UnexpectedFail";
    case RealSensorReplayRegressionBlockReason::ParseErrorNotDetected:
        return "ParseErrorNotDetected";
    case RealSensorReplayRegressionBlockReason::NoPacketRecords:
        return "NoPacketRecords";
    case RealSensorReplayRegressionBlockReason::ReportInvalid:
        return "ReportInvalid";
    }
    return "Unknown";
}

inline int real_sensor_replay_regression_case_kind_id(
    RealSensorReplayRegressionCaseKind kind) {
    return static_cast<int>(kind);
}

inline int real_sensor_replay_regression_status_id(
    RealSensorReplayRegressionStatus status) {
    return static_cast<int>(status);
}

inline int real_sensor_replay_regression_block_reason_id(
    RealSensorReplayRegressionBlockReason reason) {
    return static_cast<int>(reason);
}

} // namespace robot_slamd
