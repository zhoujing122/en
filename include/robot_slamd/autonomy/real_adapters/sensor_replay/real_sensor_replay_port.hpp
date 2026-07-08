#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_snapshot_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_types.hpp"

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
        if (log_.records.empty()) {
            result_.fault = RealSensorReplayFault::EmptyLog;
            result_.summary = "real_sensor_replay_empty_log";
        }
    }

    bool ready() const override {
        if (options_.require_non_empty_log && log_.records.empty()) {
            return false;
        }
        return true;
    }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        if (!ready()) {
            result_.status = RealSensorReplayStatus::Rejected;
            result_.fault = RealSensorReplayFault::EmptyLog;
            result_.failed.push_back("real_sensor_replay_empty_log");
            return last_snapshot_;
        }

        result_.status = RealSensorReplayStatus::Running;
        while (index_ < static_cast<int>(log_.records.size())) {
            const auto &record = log_.records[static_cast<std::size_t>(index_)];
            if (record.kind != RealSensorReplayRecordKind::Packet) {
                index_++;
                continue;
            }

            // Replay packets through the same contract-backed snapshot builder.
            auto build = builder_.build_snapshot(record.packet, now_s);
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
                return last_snapshot_;
            }
            index_++;
        }

        if (options_.loop && !log_.records.empty()) {
            index_ = 0;
            return latest_snapshot(now_s);
        }
        result_.status = RealSensorReplayStatus::Completed;
        result_.fault = RealSensorReplayFault::EndOfLog;
        result_.summary = "real_sensor_replay_completed";
        return last_snapshot_;
    }

    std::string status() const override {
        std::ostringstream o;
        o << "RealSensorReplayPort: index=" << index_
          << " status=" << to_string(result_.status);
        return o.str();
    }

    RealSensorReplayResult result() const {
        return result_;
    }

    int current_index() const {
        return index_;
    }

private:
    RealSensorReplayLog log_;
    RealSensorSnapshotBuilder builder_;
    RealSensorReplayOptions options_;
    mutable RealSensorReplayResult result_;
    mutable int index_ = 0;
    mutable RobotSlamSensorSnapshot last_snapshot_;
};

} // namespace robot_slamd
