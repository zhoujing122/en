#pragma once
#include "robot_slamd/motion/algorithm_motion_command_adapter.hpp"
#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"
#include "robot_slamd/software_motion/software_motion_transport.hpp"

#include <memory>

namespace robot_slamd {

struct AlgorithmMotionFacadeResult {
    bool ok = false;
    bool accepted = false;
    std::string error;
    AlgorithmMotionCommand algorithm_command;
    SoftwareMotionCommand software_command;
    SoftwareMotionSendResult send_result;
};

class AlgorithmMotionFacade {
public:
    AlgorithmMotionFacade(std::shared_ptr<SoftwareMotionCommandTransport> transport,
                          AlgorithmMotionCommandBuilder builder,
                          AlgorithmMotionCommandAdapter adapter)
        : transport_(std::move(transport)), builder_(builder), adapter_(adapter) {}

    AlgorithmMotionFacadeResult stop(double timestamp_s, std::string reason = "facade_stop") {
        return send_algorithm_command(builder_.stop(timestamp_s, std::move(reason)));
    }

    AlgorithmMotionFacadeResult emergency_stop(double timestamp_s,
                                               std::string reason = "facade_emergency_stop") {
        return send_algorithm_command(builder_.emergency_stop(timestamp_s, std::move(reason)));
    }

    AlgorithmMotionFacadeResult direction_probe_left(double timestamp_s) {
        return send_algorithm_command(builder_.direction_probe_left(timestamp_s));
    }

    AlgorithmMotionFacadeResult direction_probe_right(double timestamp_s) {
        return send_algorithm_command(builder_.direction_probe_right(timestamp_s));
    }

    AlgorithmMotionFacadeResult active_scan_turn_left(double timestamp_s) {
        return send_algorithm_command(builder_.active_scan_turn_left(timestamp_s));
    }

    AlgorithmMotionFacadeResult active_scan_turn_right(double timestamp_s) {
        return send_algorithm_command(builder_.active_scan_turn_right(timestamp_s));
    }

    AlgorithmMotionFacadeResult recovery_forward(double timestamp_s) {
        return send_algorithm_command(builder_.recovery_forward(timestamp_s));
    }

    AlgorithmMotionFacadeResult recovery_backward(double timestamp_s) {
        return send_algorithm_command(builder_.recovery_backward(timestamp_s));
    }

    AlgorithmMotionFacadeResult recovery_turn_left(double timestamp_s) {
        return send_algorithm_command(builder_.recovery_turn_left(timestamp_s));
    }

    AlgorithmMotionFacadeResult recovery_turn_right(double timestamp_s) {
        return send_algorithm_command(builder_.recovery_turn_right(timestamp_s));
    }

private:
    AlgorithmMotionFacadeResult send_algorithm_command(const AlgorithmMotionCommand &command) {
        AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;
        if (!transport_) {
            result.error = "software_motion_transport_missing";
            return result;
        }

        auto converted = adapter_.to_software_command(command);
        if (!converted.ok) {
            result.error = converted.error;
            return result;
        }
        result.software_command = converted.command;

        result.send_result = transport_->send_command(converted.command);
        if (!result.send_result.ok) {
            result.error = "software_motion_send_failed";
            return result;
        }
        if (!result.send_result.accepted) {
            result.error = "software_motion_rejected";
            return result;
        }

        result.ok = true;
        result.accepted = true;
        return result;
    }

    std::shared_ptr<SoftwareMotionCommandTransport> transport_;
    AlgorithmMotionCommandBuilder builder_;
    AlgorithmMotionCommandAdapter adapter_;
};

} // namespace robot_slamd
