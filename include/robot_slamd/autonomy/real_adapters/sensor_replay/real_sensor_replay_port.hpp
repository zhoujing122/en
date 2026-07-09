#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_snapshot_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_types.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace robot_slamd {

class RealSensorReplayPort final : public RobotSlamSensorPort {
public:
    RealSensorReplayPort(
        RealSensorReplayLog log,
        RealSensorSnapshotBuilder builder = {},
        RealSensorReplayOptions options = {})
        : log_(std::move(log)),
          builder_(std::move(builder)),
          options_(std::move(options)) {
        result_.status = RealSensorReplayStatus::Parsed;
        result_.parsed_record_count = static_cast<int>(log_.records.size());
        for (const auto &record : log_.records) {
            if (record.kind == RealSensorReplayRecordKind::Packet) {
                result_.packet_record_count++;
            } else if (record.kind == RealSensorReplayRecordKind::Comment) {
                result_.comment_record_count++;
            } else if (record.kind == RealSensorReplayRecordKind::InvalidRecord) {
                result_.invalid_record_count++;
            }
        }

        // Constructor diagnostics describe why ready() may be false.
        if (log_.records.empty()) {
            result_.fault = RealSensorReplayFault::EmptyLog;
            result_.summary = "real_sensor_replay_empty_log";
        } else if (result_.packet_record_count == 0) {
            result_.fault = RealSensorReplayFault::NoPacketRecords;
            result_.summary = "real_sensor_replay_no_packet_records";
        } else if (result_.invalid_record_count > 0) {
            result_.fault = RealSensorReplayFault::InvalidRecord;
            result_.summary = "real_sensor_replay_invalid_records_present";
        } else {
            result_.fault = RealSensorReplayFault::None;
            result_.summary = "real_sensor_replay_parsed";
        }
    }

    bool ready() const override {
        if (options_.require_non_empty_log && log_.records.empty()) {
            return false;
        }
        if (options_.require_packet_records && !has_packet_records()) {
            return false;
        }
        if (options_.reject_invalid_records && has_invalid_records()) {
            return false;
        }
        return has_packet_records();
    }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        if (!ready()) {
            result_.ok = false;
            result_.status = RealSensorReplayStatus::Rejected;
            if (log_.records.empty()) {
                result_.fault = RealSensorReplayFault::EmptyLog;
                result_.failed.push_back("real_sensor_replay_empty_log");
            } else if (!has_packet_records()) {
                result_.fault = RealSensorReplayFault::NoPacketRecords;
                result_.failed.push_back("real_sensor_replay_no_packet_records");
            } else if (has_invalid_records()) {
                result_.fault = RealSensorReplayFault::InvalidRecord;
                result_.failed.push_back("real_sensor_replay_invalid_records_present");
            }
            return empty_snapshot();
        }

        result_.status = RealSensorReplayStatus::Running;
        int processed = 0;
        while (processed < options_.max_records_per_run) {
            if (index_ >= static_cast<int>(log_.records.size())) {
                if (options_.loop) {
                    index_ = 0;
                } else {
                    result_.status = RealSensorReplayStatus::Completed;
                    result_.fault = RealSensorReplayFault::EndOfLog;
                    result_.summary = "real_sensor_replay_completed";
                    return last_snapshot_;
                }
            }

            const auto &record = log_.records[static_cast<std::size_t>(index_)];
            processed++;
            if (record.kind == RealSensorReplayRecordKind::Comment) {
                index_++;
                continue;
            }
            if (record.kind == RealSensorReplayRecordKind::InvalidRecord) {
                result_.ok = false;
                result_.status = RealSensorReplayStatus::Rejected;
                result_.fault = RealSensorReplayFault::InvalidRecord;
                result_.rejected_packet_count++;
                result_.failed.push_back("real_sensor_replay_invalid_record");
                if (options_.fail_on_contract_error) {
                    return empty_snapshot();
                }
                index_++;
                continue;
            }
            if (record.kind != RealSensorReplayRecordKind::Packet) {
                index_++;
                continue;
            }

            // Replay packets through the same contract-backed snapshot builder.
            const double validation_now_s = validation_now_for_record(record, now_s);
            result_.last_validation_now_s = validation_now_s;
            result_.last_packet_time_s = record.packet.packet_timestamp_s;
            result_.last_effective_sensor_time_s = effective_sensor_time(record);
            auto build = builder_.build_snapshot(record.packet, validation_now_s);
            if (build.ok) {
                result_.ok = true;
                result_.fault = RealSensorReplayFault::None;
                result_.accepted_packet_count++;
                result_.passed.push_back("real_sensor_replay_packet_accepted");
                last_snapshot_ = build.snapshot;
                index_++;
                return last_snapshot_;
            }

            result_.ok = false;
            result_.status = RealSensorReplayStatus::Rejected;
            result_.fault = RealSensorReplayFault::SnapshotBuildFailed;
            result_.rejected_packet_count++;
            result_.failed.push_back("real_sensor_replay_packet_rejected");
            if (options_.fail_on_contract_error) {
                return empty_snapshot();
            }
            index_++;
        }

        result_.status = RealSensorReplayStatus::Fault;
        result_.fault = RealSensorReplayFault::RecordTimeModeMismatch;
        result_.failed.push_back("real_sensor_replay_max_records_per_run_reached");
        return empty_snapshot();
    }

    std::string status() const override {
        std::ostringstream o;
        o << "RealSensorReplayPort: index=" << index_
          << " status=" << to_string(result_.status)
          << " fault=" << to_string(result_.fault)
          << " packet_count=" << result_.packet_record_count
          << " invalid_count=" << result_.invalid_record_count;
        return o.str();
    }

    RealSensorReplayResult result() const {
        return result_;
    }

    int current_index() const {
        return index_;
    }

private:
    bool has_packet_records() const {
        return result_.packet_record_count > 0;
    }

    bool has_invalid_records() const {
        return result_.invalid_record_count > 0;
    }

    double validation_now_for_record(
        const RealSensorReplayRecord &record,
        double external_now_s) const {
        if (options_.time_mode == RealSensorReplayTimeMode::ExternalNow) {
            return external_now_s;
        }
        if (options_.time_mode == RealSensorReplayTimeMode::RecordSensorMaxTime) {
            return effective_sensor_time(record);
        }
        return record.packet.packet_timestamp_s;
    }

    static double effective_sensor_time(const RealSensorReplayRecord &record) {
        double value = record.packet.packet_timestamp_s;
        if (record.packet.has_tof) {
            value = std::max(value,
                             real_sensor_effective_time_s(record.packet.tof.timing));
        }
        if (record.packet.has_imu) {
            value = std::max(value, record.packet.imu.timestamp_s);
        }
        if (record.packet.has_wheel) {
            value = std::max(
                value,
                real_sensor_effective_time_s(record.packet.wheel.timing));
        }
        return value;
    }

    static RobotSlamSensorSnapshot empty_snapshot() {
        return RobotSlamSensorSnapshot{};
    }

    RealSensorReplayLog log_;
    RealSensorSnapshotBuilder builder_;
    RealSensorReplayOptions options_;
    mutable RealSensorReplayResult result_;
    mutable int index_ = 0;
    mutable RobotSlamSensorSnapshot last_snapshot_;
};

} // namespace robot_slamd
