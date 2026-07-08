#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/autonomy/real_adapters/real_adapter_stub_types.hpp"

#include <utility>
#include <vector>

namespace robot_slamd {

class RealSoftwareMotionPortStub final : public RobotSlamMotionPort {
public:
    explicit RealSoftwareMotionPortStub(
        RealAdapterStubStatus status = default_status())
        : status_(std::move(status)) {}

    bool ready() const override {
        return status_.ready;
    }

    AlgorithmMotionFacadeResult send_algorithm_command(
        const AlgorithmMotionCommand &command) override {
        AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;

        // M3-B0 records the rejected request for contract tests only.
        rejected_commands_.push_back(command);
        result.ok = false;
        result.accepted = false;
        result.error = "real_software_motion_port_stub_not_implemented";
        return result;
    }

    RobotSlamMotionFeedback latest_feedback(double now_s) override {
        RobotSlamMotionFeedback feedback;
        feedback.command_active = false;
        feedback.command_settled = true;
        feedback.fault = true;
        feedback.fault_reason =
            "real_software_motion_port_stub_not_implemented";
        feedback.timestamp_s = now_s;
        return feedback;
    }

    std::string status() const override {
        return "RealSoftwareMotionPortStub: "
               "waiting_for_real_motion_adapter_implementation";
    }

    RealAdapterStubStatus stub_status() const {
        return status_;
    }

    const std::vector<AlgorithmMotionCommand> &rejected_commands() const {
        return rejected_commands_;
    }

private:
    static RealAdapterStubStatus default_status() {
        RealAdapterStubStatus status;
        status.kind = RealAdapterStubKind::Motion;
        status.state = RealAdapterStubState::WaitingForImplementation;
        status.ready = false;
        status.name = "RealSoftwareMotionPortStub";
        status.message = "waiting_for_real_motion_adapter_implementation";
        return status;
    }

    RealAdapterStubStatus status_;
    std::vector<AlgorithmMotionCommand> rejected_commands_;
};

} // namespace robot_slamd
