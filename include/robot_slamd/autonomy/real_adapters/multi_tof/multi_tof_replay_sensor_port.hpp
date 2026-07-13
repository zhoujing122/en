#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_port.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

namespace robot_slamd {

class MultiTofReplaySensorPort final : public RobotSlamSensorPort {
public:
    explicit MultiTofReplaySensorPort(MultiTofReplayPort replay)
        : replay_(std::move(replay)), ready_at_construction_(replay_.ready()) {}

    bool ready() const override {
        return ready_at_construction_;
    }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        read_call_count_++;
        last_result_ = replay_.latest_snapshot(now_s);
        has_last_result_ = true;

        if (last_result_.status == MultiTofReplayStatus::Completed ||
            last_result_.fault == MultiTofReplayFault::EndOfReplay) {
            end_of_replay_return_count_++;
            replay_completed_ = true;
            return RobotSlamSensorSnapshot{};
        }

        if (!last_result_.ok) {
            rejected_snapshot_count_++;
            replay_faulted_ = true;
            return RobotSlamSensorSnapshot{};
        }

        successful_snapshot_count_++;
        last_success_snapshot_ = last_result_.snapshot;
        has_last_success_snapshot_ = true;
        return last_result_.snapshot;
    }

    std::string status() const override {
        std::ostringstream o;
        o << "MultiTofReplaySensorPort"
          << " ready=" << (ready() ? 1 : 0)
          << " read_calls=" << read_call_count_
          << " successful=" << successful_snapshot_count_
          << " rejected=" << rejected_snapshot_count_
          << " completed=" << end_of_replay_return_count_
          << " consumed_packets=" << replay_.consumed_packet_count()
          << " last_status=" << to_string(last_result_.status)
          << " last_fault=" << to_string(last_result_.fault)
          << " last_message=" << last_result_.message;
        return o.str();
    }

    bool has_last_read_result() const {
        return has_last_result_;
    }

    const MultiTofReplayReadResult &last_read_result() const {
        if (!has_last_result_) {
            throw std::logic_error("multi_tof_replay_sensor_port_no_last_result");
        }
        return last_result_;
    }

    const MultiTofReplayReadResult &last_result() const {
        return last_read_result();
    }

    bool replay_completed() const {
        return replay_completed_;
    }

    bool replay_faulted() const {
        return replay_faulted_;
    }

    int read_call_count() const {
        return read_call_count_;
    }

    int successful_snapshot_count() const {
        return successful_snapshot_count_;
    }

    int rejected_snapshot_count() const {
        return rejected_snapshot_count_;
    }

    int end_of_replay_return_count() const {
        return end_of_replay_return_count_;
    }

    bool has_last_success_snapshot() const {
        return has_last_success_snapshot_;
    }

    const RobotSlamSensorSnapshot &last_success_snapshot() const {
        return last_success_snapshot_;
    }

    int consumed_packet_count() const {
        return replay_.consumed_packet_count();
    }

private:
    MultiTofReplayPort replay_;
    bool ready_at_construction_ = false;
    MultiTofReplayReadResult last_result_;
    bool has_last_result_ = false;
    bool replay_completed_ = false;
    bool replay_faulted_ = false;
    int read_call_count_ = 0;
    int successful_snapshot_count_ = 0;
    int rejected_snapshot_count_ = 0;
    int end_of_replay_return_count_ = 0;
    bool has_last_success_snapshot_ = false;
    RobotSlamSensorSnapshot last_success_snapshot_;
};

} // namespace robot_slamd
