#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class MultiTofReplayRecordKind {
    Comment,
    Packet,
    InvalidRecord
};

enum class MultiTofReplayStatus {
    Ready,
    NotReady,
    Completed,
    Rejected,
    Fault
};

enum class MultiTofReplayFault {
    None,
    EmptyLog,
    NoPacketRecords,
    InvalidRecord,
    ParseError,
    MissingField,
    InvalidNumeric,
    InvalidBool,
    EmptyRanges,
    PacketContractFailed,
    SyncFailed,
    SnapshotBuildFailed,
    EndOfReplay,
    ReplayDisabled
};

enum class MultiTofReplayTimeMode {
    ExternalNow,
    RecordPacketTime,
    RecordSensorMaxTime
};

struct MultiTofReplayRecord {
    MultiTofReplayRecordKind kind =
        MultiTofReplayRecordKind::InvalidRecord;
    MultiTofRawPacket packet;
    std::string raw_line;
    std::string error;
    int line_number = 0;
};

struct MultiTofReplayOptions {
    bool enabled = false;
    bool reject_invalid_records = true;
    bool require_snapshot_build = true;
    MultiTofReplayTimeMode time_mode =
        MultiTofReplayTimeMode::RecordPacketTime;
    double external_now_s = 0.0;
};

struct MultiTofReplayReadResult {
    bool ok = false;
    MultiTofReplayStatus status = MultiTofReplayStatus::Rejected;
    MultiTofReplayFault fault = MultiTofReplayFault::None;
    MultiTofReplayRecord record;
    RobotSlamSensorSnapshot snapshot;
    MultiTofSnapshotBuildResult build_result;
    int packet_index = -1;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string message;
};

inline std::string to_string(MultiTofReplayRecordKind kind) {
    switch (kind) {
    case MultiTofReplayRecordKind::Comment:
        return "comment";
    case MultiTofReplayRecordKind::Packet:
        return "packet";
    case MultiTofReplayRecordKind::InvalidRecord:
        return "invalid_record";
    }
    return "unknown";
}

inline std::string to_string(MultiTofReplayStatus status) {
    switch (status) {
    case MultiTofReplayStatus::Ready:
        return "ready";
    case MultiTofReplayStatus::NotReady:
        return "not_ready";
    case MultiTofReplayStatus::Completed:
        return "completed";
    case MultiTofReplayStatus::Rejected:
        return "rejected";
    case MultiTofReplayStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(MultiTofReplayFault fault) {
    switch (fault) {
    case MultiTofReplayFault::None:
        return "none";
    case MultiTofReplayFault::EmptyLog:
        return "empty_log";
    case MultiTofReplayFault::NoPacketRecords:
        return "no_packet_records";
    case MultiTofReplayFault::InvalidRecord:
        return "invalid_record";
    case MultiTofReplayFault::ParseError:
        return "parse_error";
    case MultiTofReplayFault::MissingField:
        return "missing_field";
    case MultiTofReplayFault::InvalidNumeric:
        return "invalid_numeric";
    case MultiTofReplayFault::InvalidBool:
        return "invalid_bool";
    case MultiTofReplayFault::EmptyRanges:
        return "empty_ranges";
    case MultiTofReplayFault::PacketContractFailed:
        return "packet_contract_failed";
    case MultiTofReplayFault::SyncFailed:
        return "sync_failed";
    case MultiTofReplayFault::SnapshotBuildFailed:
        return "snapshot_build_failed";
    case MultiTofReplayFault::EndOfReplay:
        return "end_of_replay";
    case MultiTofReplayFault::ReplayDisabled:
        return "replay_disabled";
    }
    return "unknown";
}

inline std::string to_string(MultiTofReplayTimeMode mode) {
    switch (mode) {
    case MultiTofReplayTimeMode::ExternalNow:
        return "external_now";
    case MultiTofReplayTimeMode::RecordPacketTime:
        return "record_packet_time";
    case MultiTofReplayTimeMode::RecordSensorMaxTime:
        return "record_sensor_max_time";
    }
    return "unknown";
}

inline int multi_tof_replay_record_kind_id(
    MultiTofReplayRecordKind kind) {
    return static_cast<int>(kind);
}

inline int multi_tof_replay_status_id(MultiTofReplayStatus status) {
    return static_cast<int>(status);
}

inline int multi_tof_replay_fault_id(MultiTofReplayFault fault) {
    return static_cast<int>(fault);
}

inline int multi_tof_replay_time_mode_id(MultiTofReplayTimeMode mode) {
    return static_cast<int>(mode);
}

} // namespace robot_slamd
