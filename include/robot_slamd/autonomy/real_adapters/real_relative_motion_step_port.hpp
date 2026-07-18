#pragma once

#include "robot_slamd/autonomy/ports/relative_motion_step_port.hpp"

namespace robot_slamd {

// Deliberately no device discovery or transport code exists in P2/P3.
class RealRelativeMotionStepPort final : public RelativeMotionStepPort {
public:
    bool ready() const override { return false; }
    RelativeMotionStepResult submit(const RelativeMotionStepCommand &command,
                                    double) override {
        return reject(command, "real_relative_motion_fail_closed_no_adapter");
    }
    RelativeMotionStepResult poll(double) override { return last_; }
    RelativeMotionStepResult cancel(double) override { return stop_result("real_cancel_fail_closed"); }
    RelativeMotionStepResult stop(double) override { return stop_result("real_stop_fail_closed"); }
    RelativeMotionStepResult emergency_stop(double) override {
        last_.state = RelativeMotionStepState::EstopLatched;
        last_.reason = "real_estop_fail_closed_no_adapter";
        return last_;
    }
    RelativeMotionStepResult clear_estop(double) override {
        last_.state = RelativeMotionStepState::Fault;
        last_.reason = "real_clear_estop_requires_hardware_adapter";
        return last_;
    }
    std::string status() const override {
        return "real_relative_motion_unavailable_fail_closed";
    }

private:
    RelativeMotionStepResult reject(const RelativeMotionStepCommand &command,
                                    const char *reason) {
        last_ = {};
        last_.command_id = command.command_id;
        last_.action = command.action;
        last_.requested_amount = command.target_amount;
        last_.state = RelativeMotionStepState::Fault;
        last_.error_code = 40;
        last_.reason = reason;
        return last_;
    }
    RelativeMotionStepResult stop_result(const char *reason) {
        last_.state = RelativeMotionStepState::Done;
        last_.reason = reason;
        return last_;
    }
    RelativeMotionStepResult last_;
};

} // namespace robot_slamd
