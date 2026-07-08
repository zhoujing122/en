#pragma once

// Replay-only sensor adapter for contract and coordinator tests. No hardware IO.
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

class ReplayRobotSlamSensorPort final : public RobotSlamSensorPort {
public:
    ReplayRobotSlamSensorPort(std::vector<RobotSlamSensorSnapshot> snapshots,
                              bool ready = true)
        : snapshots_(std::move(snapshots)), ready_(ready) {}

    bool ready() const override {
        return ready_ && !snapshots_.empty();
    }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        (void)now_s;
        if (snapshots_.empty()) {
            return RobotSlamSensorSnapshot{};
        }
        const std::size_t current = index_;
        if (index_ + 1 < snapshots_.size()) {
            index_++;
        }
        return snapshots_[current];
    }

    std::string status() const override {
        if (snapshots_.empty()) {
            return "replay_sensor_empty";
        }
        return ready_ ? "replay_sensor_ready" : "replay_sensor_not_ready";
    }

    void set_ready(bool ready) {
        ready_ = ready;
    }

private:
    std::vector<RobotSlamSensorSnapshot> snapshots_;
    bool ready_ = true;
    std::size_t index_ = 0;
};

} // namespace robot_slamd
