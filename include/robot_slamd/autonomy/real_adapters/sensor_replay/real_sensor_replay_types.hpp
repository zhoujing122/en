#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class RealSensorReplayRecordKind {
    Packet,
    Comment,
    EndOfLog,
    InvalidRecord
};

enum class RealSensorReplayStatus {
    NotStarted,
    Parsed,
    Running,
    Completed,
    Rejected,
    Fault
};

enum class RealSensorReplayFault {
    None,
    EmptyLog,
    ParseError,
    InvalidRecord,
    ContractFailed,
    SnapshotBuildFailed,
    EndOfLog,
    InvalidNumericValue,
    MissingRequiredField,
    NoPacketRecords,
    RecordTimeInvalid,
    RecordTimeModeMismatch
};

enum class RealSensorReplayTimeMode {
    ExternalNow,
    RecordPacketTime,
    RecordSensorMaxTime
};

struct RealSensorReplayRecord {
    RealSensorReplayRecordKind kind = RealSensorReplayRecordKind::Packet;
    int line_number = 0;
    RealSensorRawPacket packet;
    std::string raw_line;
    std::string message;
};

struct RealSensorReplayLog {
    std::vector<RealSensorReplayRecord> records;
    std::string source_name;
};

struct RealSensorReplayOptions {
    bool enabled = false;
    bool loop = false;
    bool fail_on_contract_error = true;
    bool require_non_empty_log = true;
    double start_time_s = 100.0;
    RealSensorReplayTimeMode time_mode =
        RealSensorReplayTimeMode::RecordPacketTime;
    bool reject_invalid_records = true;
    bool require_packet_records = true;
    bool preserve_parse_errors = true;
    int max_records_per_run = 10000;
};

struct RealSensorReplayResult {
    bool ok = false;
    RealSensorReplayStatus status = RealSensorReplayStatus::NotStarted;
    RealSensorReplayFault fault = RealSensorReplayFault::None;
    int parsed_record_count = 0;
    int accepted_packet_count = 0;
    int rejected_packet_count = 0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
    int comment_record_count = 0;
    int invalid_record_count = 0;
    int packet_record_count = 0;
    double last_validation_now_s = 0.0;
    double last_packet_time_s = 0.0;
    double last_effective_sensor_time_s = 0.0;
};

inline std::string to_string(RealSensorReplayRecordKind kind) {
    switch (kind) {
    case RealSensorReplayRecordKind::Packet:
        return "Packet";
    case RealSensorReplayRecordKind::Comment:
        return "Comment";
    case RealSensorReplayRecordKind::EndOfLog:
        return "EndOfLog";
    case RealSensorReplayRecordKind::InvalidRecord:
        return "InvalidRecord";
    }
    return "Unknown";
}

inline std::string to_string(RealSensorReplayStatus status) {
    switch (status) {
    case RealSensorReplayStatus::NotStarted:
        return "NotStarted";
    case RealSensorReplayStatus::Parsed:
        return "Parsed";
    case RealSensorReplayStatus::Running:
        return "Running";
    case RealSensorReplayStatus::Completed:
        return "Completed";
    case RealSensorReplayStatus::Rejected:
        return "Rejected";
    case RealSensorReplayStatus::Fault:
        return "Fault";
    }
    return "Unknown";
}

inline std::string to_string(RealSensorReplayFault fault) {
    switch (fault) {
    case RealSensorReplayFault::None:
        return "None";
    case RealSensorReplayFault::EmptyLog:
        return "EmptyLog";
    case RealSensorReplayFault::ParseError:
        return "ParseError";
    case RealSensorReplayFault::InvalidRecord:
        return "InvalidRecord";
    case RealSensorReplayFault::ContractFailed:
        return "ContractFailed";
    case RealSensorReplayFault::SnapshotBuildFailed:
        return "SnapshotBuildFailed";
    case RealSensorReplayFault::EndOfLog:
        return "EndOfLog";
    case RealSensorReplayFault::InvalidNumericValue:
        return "InvalidNumericValue";
    case RealSensorReplayFault::MissingRequiredField:
        return "MissingRequiredField";
    case RealSensorReplayFault::NoPacketRecords:
        return "NoPacketRecords";
    case RealSensorReplayFault::RecordTimeInvalid:
        return "RecordTimeInvalid";
    case RealSensorReplayFault::RecordTimeModeMismatch:
        return "RecordTimeModeMismatch";
    }
    return "Unknown";
}

inline std::string to_string(RealSensorReplayTimeMode mode) {
    switch (mode) {
    case RealSensorReplayTimeMode::ExternalNow:
        return "ExternalNow";
    case RealSensorReplayTimeMode::RecordPacketTime:
        return "RecordPacketTime";
    case RealSensorReplayTimeMode::RecordSensorMaxTime:
        return "RecordSensorMaxTime";
    }
    return "Unknown";
}

inline int real_sensor_replay_record_kind_id(
    RealSensorReplayRecordKind kind) {
    return static_cast<int>(kind);
}

inline int real_sensor_replay_status_id(RealSensorReplayStatus status) {
    return static_cast<int>(status);
}

inline int real_sensor_replay_fault_id(RealSensorReplayFault fault) {
    return static_cast<int>(fault);
}

inline int real_sensor_replay_time_mode_id(RealSensorReplayTimeMode mode) {
    return static_cast<int>(mode);
}

} // namespace robot_slamd
