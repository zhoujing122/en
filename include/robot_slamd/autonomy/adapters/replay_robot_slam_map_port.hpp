#pragma once

// Replay-only map adapter for contract and coordinator tests. No map backend IO.
#include "robot_slamd/autonomy/ports/robot_slam_map_port.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

class ReplayRobotSlamMapPort final : public RobotSlamMapPort {
public:
    ReplayRobotSlamMapPort(std::vector<RobotSlamMapQuality> qualities,
                           bool ready = true)
        : qualities_(std::move(qualities)), ready_(ready) {}

    bool ready() const override {
        return ready_ && !qualities_.empty();
    }

    bool integrate_sensor_snapshot(const RobotSlamSensorSnapshot &snapshot,
                                   double now_s) override {
        (void)snapshot;
        (void)now_s;
        integrate_count_++;
        return ready();
    }

    RobotSlamMapQuality latest_quality(double now_s) const override {
        (void)now_s;
        if (qualities_.empty()) {
            return RobotSlamMapQuality{};
        }
        const std::size_t current = index_;
        if (index_ + 1 < qualities_.size()) {
            index_++;
        }
        return qualities_[current];
    }

    bool save_map(const std::string &output_path) override {
        last_saved_path_ = output_path;
        save_count_++;
        return ready();
    }

    std::string status() const override {
        if (qualities_.empty()) {
            return "replay_map_empty";
        }
        return ready_ ? "replay_map_ready" : "replay_map_not_ready";
    }

    void set_ready(bool ready) {
        ready_ = ready;
    }

    int integrate_count() const {
        return integrate_count_;
    }

    int save_count() const {
        return save_count_;
    }

    const std::string &last_saved_path() const {
        return last_saved_path_;
    }

private:
    std::vector<RobotSlamMapQuality> qualities_;
    bool ready_ = true;
    mutable std::size_t index_ = 0;
    int integrate_count_ = 0;
    int save_count_ = 0;
    std::string last_saved_path_;
};

} // namespace robot_slamd
