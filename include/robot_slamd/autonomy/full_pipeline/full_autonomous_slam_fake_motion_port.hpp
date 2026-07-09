#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class FullAutonomousSlamFakeMotionPort final : public RobotSlamMotionPort {
public:
    explicit FullAutonomousSlamFakeMotionPort(
        bool reject_motion = false)
        : reject_motion_(reject_motion) {
        feedback_.command_active = false;
        feedback_.command_settled = true;
    }

    bool ready() const override {
        return true;
    }

    AlgorithmMotionFacadeResult send_algorithm_command(
        const AlgorithmMotionCommand &command) override {
        AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;
        command_count_++;
        feedback_.timestamp_s = command.timestamp_s;
        feedback_.command_active = false;
        feedback_.command_settled = true;
        feedback_.fault = false;
        feedback_.fault_reason.clear();

        if (command.kind == AlgorithmMotionKind::Forward ||
            command.kind == AlgorithmMotionKind::Backward ||
            command.kind == AlgorithmMotionKind::RecoveryForward ||
            command.kind == AlgorithmMotionKind::RecoveryBackward) {
            saw_forward_or_backward_ = true;
            result.error = "full_pipeline_forward_backward_rejected";
            feedback_.fault = true;
            feedback_.fault_reason = result.error;
            return result;
        }

        if (is_stop(command)) {
            stop_command_count_++;
            result.ok = true;
            result.accepted = true;
            return result;
        }

        if (!is_allowed_turn(command)) {
            result.error = "full_pipeline_motion_kind_rejected";
            feedback_.fault = true;
            feedback_.fault_reason = result.error;
            return result;
        }

        active_scan_command_count_++;
        if (reject_motion_) {
            result.error = "full_pipeline_motion_rejected";
            feedback_.fault = true;
            feedback_.fault_reason = result.error;
            return result;
        }

        result.ok = true;
        result.accepted = true;
        return result;
    }

    RobotSlamMotionFeedback latest_feedback(double now_s) override {
        feedback_.timestamp_s = now_s;
        return feedback_;
    }

    std::string status() const override {
        std::ostringstream o;
        o << "FullAutonomousSlamFakeMotionPort: commands=" << command_count_
          << " active_scan=" << active_scan_command_count_
          << " stop=" << stop_command_count_
          << " forward_backward=" << (saw_forward_or_backward_ ? 1 : 0);
        return o.str();
    }

    int command_count() const {
        return command_count_;
    }

    int active_scan_command_count() const {
        return active_scan_command_count_;
    }

    int stop_command_count() const {
        return stop_command_count_;
    }

    bool saw_forward_or_backward() const {
        return saw_forward_or_backward_;
    }

private:
    static bool is_stop(const AlgorithmMotionCommand &command) {
        return command.kind == AlgorithmMotionKind::Stop ||
               command.kind == AlgorithmMotionKind::EmergencyStop;
    }

    static bool is_allowed_turn(const AlgorithmMotionCommand &command) {
        return command.kind == AlgorithmMotionKind::TurnLeft ||
               command.kind == AlgorithmMotionKind::TurnRight ||
               command.kind == AlgorithmMotionKind::ActiveScanTurnLeft ||
               command.kind == AlgorithmMotionKind::ActiveScanTurnRight;
    }

    bool reject_motion_ = false;
    int command_count_ = 0;
    int active_scan_command_count_ = 0;
    int stop_command_count_ = 0;
    bool saw_forward_or_backward_ = false;
    RobotSlamMotionFeedback feedback_;
};

} // namespace robot_slamd
