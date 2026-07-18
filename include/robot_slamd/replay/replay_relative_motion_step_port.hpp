#pragma once

#include "robot_slamd/autonomy/ports/relative_motion_step_port.hpp"

#include <sstream>

namespace robot_slamd {

class ReplayNoMotionStepPort final : public RelativeMotionStepPort {
public:
    bool ready() const override { return true; }

    RelativeMotionStepResult submit(const RelativeMotionStepCommand &command,
                                    double) override {
        RelativeMotionStepResult result;
        result.command_id = command.command_id;
        result.action = command.action;
        result.requested_amount = command.target_amount;
        result.state = RelativeMotionStepState::Fault;
        result.error_code = 30;
        result.reason = "replay_motion_disabled_no_motion_transport";
        return result;
    }
    RelativeMotionStepResult poll(double) override { return last_; }
    RelativeMotionStepResult cancel(double) override {
        last_.state = RelativeMotionStepState::Cancelled;
        last_.reason = "replay_cancel_no_motion";
        return last_;
    }
    RelativeMotionStepResult stop(double) override {
        last_.state = RelativeMotionStepState::Done;
        last_.reason = "replay_stop_no_motion";
        return last_;
    }
    RelativeMotionStepResult emergency_stop(double) override {
        estop_ = true;
        last_.state = RelativeMotionStepState::EstopLatched;
        last_.reason = "replay_estop_latched";
        return last_;
    }
    RelativeMotionStepResult clear_estop(double) override {
        estop_ = false;
        last_.state = RelativeMotionStepState::Done;
        last_.reason = "replay_estop_cleared_explicitly";
        return last_;
    }
    std::string status() const override {
        return estop_ ? "replay_no_motion estop=1" : "replay_no_motion ready=1";
    }

private:
    RelativeMotionStepResult last_;
    bool estop_ = false;
};

} // namespace robot_slamd
