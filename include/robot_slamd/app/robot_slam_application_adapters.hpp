#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sensor_port.hpp"
#include "robot_slamd/config/config.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

struct RobotSlamReplayAdapterBuildResult {
    bool ok = false;
    std::shared_ptr<MultiTofReplaySensorPort> sensor;
    std::string reason = "replay_adapter_not_built";
};

inline bool parse_multi_tof_replay_time_mode(
    const std::string &value,
    MultiTofReplayTimeMode &mode) {
    if (value == "external_now") {
        mode = MultiTofReplayTimeMode::ExternalNow;
        return true;
    }
    if (value == "record_packet_time") {
        mode = MultiTofReplayTimeMode::RecordPacketTime;
        return true;
    }
    if (value == "record_sensor_max_time") {
        mode = MultiTofReplayTimeMode::RecordSensorMaxTime;
        return true;
    }
    return false;
}

inline RobotSlamReplayAdapterBuildResult build_replay_sensor_adapter(
    const Config &config) {
    RobotSlamReplayAdapterBuildResult result;
    if (config.runtime_replay_path.empty()) {
        result.reason = "runtime.replay_path_empty";
        return result;
    }

    std::ifstream input(config.runtime_replay_path);
    if (!input) {
        result.reason = "runtime.replay_path_unreadable:" +
                        config.runtime_replay_path;
        return result;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    const auto records = MultiTofReplayLogCodec().decode_lines(lines);

    MultiTofReplayOptions replay_options;
    replay_options.enabled = true;
    replay_options.reject_invalid_records =
        config.multi_tof_replay_reject_invalid_records;
    replay_options.require_snapshot_build =
        config.multi_tof_replay_require_snapshot_build;
    if (!parse_multi_tof_replay_time_mode(
            config.multi_tof_replay_time_mode, replay_options.time_mode)) {
        result.reason = "multi_tof_replay_time_mode_invalid:" +
                        config.multi_tof_replay_time_mode;
        return result;
    }

    MultiTofSyncOptions sync_options;
    sync_options.enabled = true;
    MultiTofSnapshotBuildOptions snapshot_options;
    snapshot_options.enabled = true;
    auto replay = MultiTofReplayPort(
        records, {}, sync_options, snapshot_options, replay_options);
    result.sensor = std::make_shared<MultiTofReplaySensorPort>(
        std::move(replay));
    if (!result.sensor->ready()) {
        result.sensor.reset();
        result.reason = "runtime.replay_path_has_no_valid_multi_tof_packets";
        return result;
    }

    result.ok = true;
    result.reason = "multi_tof_replay_adapter_ready";
    return result;
}

} // namespace robot_slamd
