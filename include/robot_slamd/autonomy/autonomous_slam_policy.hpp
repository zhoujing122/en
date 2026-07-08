#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"

#include <string>

namespace robot_slamd {

struct AutonomousSlamPolicyOptions {
    double active_scan_speed = 0.05;
    double active_scan_duration_s = 0.50;
    int max_active_scan_commands = 24;
    bool prefer_left_turn = true;
};

struct AutonomousSlamPolicyDecision {
    bool should_stop = false;
    bool should_send_motion = false;
    bool completed = false;
    std::string reason;
    AlgorithmMotionCommand command;
};

class AutonomousSlamPolicy {
public:
    explicit AutonomousSlamPolicy(AutonomousSlamPolicyOptions options = {})
        : options_(options) {}

    AutonomousSlamPolicyDecision decide(const RobotSlamMapQuality &quality,
                                        AlgorithmMotionCommandBuilder &builder,
                                        double now_s) {
        AutonomousSlamPolicyDecision decision;
        if (quality.good_enough) {
            decision.should_stop = true;
            decision.completed = true;
            decision.reason = "map_quality_good";
            decision.command = builder.stop(now_s, decision.reason);
            return decision;
        }

        if (active_scan_command_count_ >= options_.max_active_scan_commands) {
            decision.should_stop = true;
            decision.reason = "max_active_scan_commands_reached";
            decision.command = builder.stop(now_s, decision.reason);
            return decision;
        }

        decision.should_send_motion = true;
        decision.reason = "map_quality_poor";
        decision.command = options_.prefer_left_turn
                               ? builder.active_scan_turn_left(now_s)
                               : builder.active_scan_turn_right(now_s);
        decision.command.speed_normalized = options_.active_scan_speed;
        decision.command.duration_s = options_.active_scan_duration_s;
        active_scan_command_count_++;
        return decision;
    }

    int active_scan_command_count() const {
        return active_scan_command_count_;
    }

private:
    AutonomousSlamPolicyOptions options_;
    int active_scan_command_count_ = 0;
};

} // namespace robot_slamd
