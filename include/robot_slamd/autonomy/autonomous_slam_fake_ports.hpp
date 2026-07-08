#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

class FakeRobotSlamSensorPort final : public RobotSlamSensorPort {
public:
    bool ready() const override {
        return ready_flag;
    }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        snapshot.tof.timestamp_s = now_s;
        snapshot.imu.timestamp_s = now_s;
        snapshot.wheel.timestamp_s = now_s;
        if (snapshot.has_tof && snapshot.tof.ranges_m.empty()) {
            snapshot.tof.ranges_m = {0.6, 0.7, 0.8};
            snapshot.tof.angle_increment_rad = 0.1;
            snapshot.tof.max_range_m = 3.0;
            snapshot.tof.frame_id = "fake_tof";
        }
        return snapshot;
    }

    std::string status() const override {
        return ready_flag ? "fake_sensor_ready" : "fake_sensor_not_ready";
    }

    bool ready_flag = true;
    RobotSlamSensorSnapshot snapshot = make_default_snapshot();

private:
    static RobotSlamSensorSnapshot make_default_snapshot() {
        RobotSlamSensorSnapshot s;
        s.has_tof = true;
        s.has_imu = true;
        s.has_wheel = true;
        s.wheel.valid = true;
        return s;
    }
};

class FakeRobotSlamMotionPort final : public RobotSlamMotionPort {
public:
    bool ready() const override {
        return ready_flag;
    }

    AlgorithmMotionFacadeResult send_algorithm_command(
        const AlgorithmMotionCommand &command) override {
        AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;
        sent_commands.push_back(command);
        send_count++;

        if (transport_fail_next_command) {
            transport_fail_next_command = false;
            result.error = "fake_motion_transport_failed";
            feedback.fault = true;
            feedback.fault_reason = result.error;
            return result;
        }
        if (reject_next_command) {
            reject_next_command = false;
            result.error = "fake_motion_rejected";
            feedback.fault = true;
            feedback.fault_reason = result.error;
            return result;
        }

        result.ok = true;
        result.accepted = true;
        feedback.command_active = false;
        feedback.command_settled = settle_immediately;
        feedback.fault = false;
        feedback.fault_reason.clear();
        feedback.timestamp_s = command.timestamp_s;
        return result;
    }

    RobotSlamMotionFeedback latest_feedback(double now_s) override {
        feedback.timestamp_s = now_s;
        return feedback;
    }

    std::string status() const override {
        return ready_flag ? "fake_motion_ready" : "fake_motion_not_ready";
    }

    bool ready_flag = true;
    bool reject_next_command = false;
    bool transport_fail_next_command = false;
    bool settle_immediately = true;
    uint64_t send_count = 0;
    RobotSlamMotionFeedback feedback;
    std::vector<AlgorithmMotionCommand> sent_commands;
};

} // namespace robot_slamd
