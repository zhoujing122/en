#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/motion/algorithm_motion_command_adapter.hpp"
#include "robot_slamd/motion/algorithm_motion_facade.hpp"
#include "robot_slamd/software_motion/software_motion_shadow_adapter.hpp"

#include <memory>
#include <string>
#include <utility>

namespace robot_slamd {

class RobotSlamMotionPort {
public:
    virtual ~RobotSlamMotionPort() = default;

    virtual bool ready() const = 0;

    virtual AlgorithmMotionFacadeResult send_algorithm_command(
        const AlgorithmMotionCommand &command) = 0;

    virtual RobotSlamMotionFeedback latest_feedback(double now_s) = 0;

    virtual std::string status() const = 0;
};

class ShadowRobotSlamMotionPort final : public RobotSlamMotionPort {
public:
    explicit ShadowRobotSlamMotionPort(
        std::shared_ptr<SoftwareMotionCommandTransport> transport)
        : transport_(std::move(transport)) {}

    bool ready() const override {
        return static_cast<bool>(transport_);
    }

    AlgorithmMotionFacadeResult send_algorithm_command(
        const AlgorithmMotionCommand &command) override {
        AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;

        if (!transport_) {
            result.error = "software_motion_transport_missing";
            last_feedback_.fault = true;
            last_feedback_.fault_reason = result.error;
            return result;
        }

        auto converted = adapter_.to_software_command(command);
        if (!converted.ok) {
            result.error = converted.error;
            last_feedback_.fault = true;
            last_feedback_.fault_reason = result.error;
            return result;
        }
        result.software_command = converted.command;
        result.send_result = transport_->send_command(converted.command);

        if (!result.send_result.ok) {
            result.error = "software_motion_send_failed";
            last_feedback_.fault = true;
            last_feedback_.fault_reason = result.error;
            return result;
        }
        if (!result.send_result.accepted) {
            result.error = "software_motion_rejected";
            last_feedback_.fault = true;
            last_feedback_.fault_reason = result.error;
            return result;
        }

        result.ok = true;
        result.accepted = true;
        last_feedback_.command_active = false;
        last_feedback_.command_settled = true;
        last_feedback_.fault = false;
        last_feedback_.fault_reason.clear();
        last_feedback_.timestamp_s = command.timestamp_s;
        return result;
    }

    RobotSlamMotionFeedback latest_feedback(double now_s) override {
        last_feedback_.timestamp_s = now_s;
        return last_feedback_;
    }

    std::string status() const override {
        return transport_ ? "shadow_motion_ready" : "shadow_motion_missing_transport";
    }

private:
    std::shared_ptr<SoftwareMotionCommandTransport> transport_;
    AlgorithmMotionCommandAdapter adapter_;
    RobotSlamMotionFeedback last_feedback_;
};

} // namespace robot_slamd
