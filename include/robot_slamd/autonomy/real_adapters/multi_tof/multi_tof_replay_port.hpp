#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_log_codec.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_builder.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

class MultiTofReplayPort {
public:
    MultiTofReplayPort(
        std::vector<MultiTofReplayRecord> records,
        MultiTofContractOptions contract_options = {},
        MultiTofSyncOptions sync_options = {},
        MultiTofSnapshotBuildOptions snapshot_options = {},
        MultiTofReplayOptions replay_options = {})
        : records_(std::move(records)),
          replay_options_(replay_options),
          builder_(contract_options, sync_options, snapshot_options) {}

    bool ready() const {
        return replay_options_.enabled && !records_.empty() &&
               packet_count() > 0 &&
               (!replay_options_.reject_invalid_records ||
                invalid_record_count() == 0);
    }

    MultiTofReplayReadResult latest_snapshot(double external_now_s) {
        if (!replay_options_.enabled) {
            return reject(MultiTofReplayFault::ReplayDisabled,
                          "multi_tof_replay_disabled");
        }
        if (records_.empty()) {
            return reject(MultiTofReplayFault::EmptyLog,
                          "multi_tof_replay_empty_log");
        }
        while (cursor_ < records_.size()) {
            const auto record = records_[cursor_++];
            if (record.kind == MultiTofReplayRecordKind::Comment) {
                continue;
            }
            if (record.kind == MultiTofReplayRecordKind::InvalidRecord) {
                if (replay_options_.reject_invalid_records) {
                    auto result = reject(MultiTofReplayFault::InvalidRecord,
                                         "multi_tof_replay_invalid_record");
                    result.record = record;
                    return result;
                }
                continue;
            }
            consumed_packet_count_++;
            MultiTofReplayReadResult result;
            result.record = record;
            result.packet_index = consumed_packet_count_ - 1;
            const double now_s = effective_now(record.packet, external_now_s);
            result.build_result = builder_.build(record.packet, now_s);
            if (replay_options_.require_snapshot_build &&
                !result.build_result.ok) {
                result.ok = false;
                result.status = MultiTofReplayStatus::Rejected;
                result.fault = MultiTofReplayFault::SnapshotBuildFailed;
                result.failed.push_back("multi_tof_replay_snapshot_build_failed");
                result.message = "multi_tof_replay_snapshot_build_failed";
                return result;
            }
            result.ok = true;
            result.status = MultiTofReplayStatus::Ready;
            result.fault = MultiTofReplayFault::None;
            result.snapshot = result.build_result.snapshot;
            result.passed.push_back("multi_tof_replay_snapshot_ready");
            result.message = "multi_tof_replay_snapshot_ready";
            return result;
        }
        return reject(MultiTofReplayFault::EndOfReplay,
                      "multi_tof_replay_end_of_replay",
                      MultiTofReplayStatus::Completed);
    }

    int packet_count() const {
        int count = 0;
        for (const auto &record : records_) {
            if (record.kind == MultiTofReplayRecordKind::Packet) {
                count++;
            }
        }
        return count;
    }

    int invalid_record_count() const {
        int count = 0;
        for (const auto &record : records_) {
            if (record.kind == MultiTofReplayRecordKind::InvalidRecord) {
                count++;
            }
        }
        return count;
    }

    int consumed_packet_count() const {
        return consumed_packet_count_;
    }

private:
    std::vector<MultiTofReplayRecord> records_;
    MultiTofReplayOptions replay_options_;
    MultiTofSnapshotBuilder builder_;
    std::size_t cursor_ = 0;
    int consumed_packet_count_ = 0;

    double effective_now(
        const MultiTofRawPacket &packet,
        double external_now_s) const {
        if (replay_options_.time_mode == MultiTofReplayTimeMode::ExternalNow) {
            return external_now_s;
        }
        if (replay_options_.time_mode ==
            MultiTofReplayTimeMode::RecordSensorMaxTime) {
            double max_time = packet.packet_timestamp_s;
            if (packet.has_front) {
                max_time = std::max(max_time, packet.front.timing.estimated_sample_time_s);
            }
            if (packet.has_left) {
                max_time = std::max(max_time, packet.left.timing.estimated_sample_time_s);
            }
            if (packet.has_right) {
                max_time = std::max(max_time, packet.right.timing.estimated_sample_time_s);
            }
            if (packet.has_imu) {
                max_time = std::max(max_time, packet.imu.timestamp_s);
            }
            if (packet.has_wheel) {
                max_time = std::max(max_time, packet.wheel.timing.estimated_sample_time_s);
            }
            return max_time;
        }
        return packet.packet_timestamp_s;
    }

    MultiTofReplayReadResult reject(
        MultiTofReplayFault fault,
        const std::string &message,
        MultiTofReplayStatus status = MultiTofReplayStatus::Rejected) const {
        MultiTofReplayReadResult result;
        result.ok = false;
        result.status = status;
        result.fault = fault;
        result.failed.push_back(message);
        result.message = message;
        return result;
    }
};

} // namespace robot_slamd
