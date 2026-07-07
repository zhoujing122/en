#pragma once
#include "robot_slamd/software_motion/software_motion_shadow_adapter.hpp"

#include <memory>
#include <utility>

namespace robot_slamd {

enum class SoftwareTransportAcceptanceCase {
    StopIdempotent,
    EmergencyStopPriority,
    TurnLeftShadow,
    TurnRightShadow,
    ForwardDisabled,
    BackwardDisabled,
    InvalidSpeedRejected,
    InvalidTtlRejected,
    TransportRejectPropagates,
};

struct SoftwareTransportAcceptanceResult {
    bool ok = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
};

class SoftwareTransportAcceptanceRunner {
public:
    explicit SoftwareTransportAcceptanceRunner(
        std::shared_ptr<SoftwareMotionCommandTransport> transport)
        : transport_(std::move(transport)) {}

    SoftwareTransportAcceptanceResult run_shadow_acceptance(double start_time_s) {
        SoftwareTransportAcceptanceResult result;
        if (!transport_) {
            result.failed.push_back("transport_missing");
            return result;
        }

        SoftwareMotionTransportContractOptions options;
        options.allow_rotation_commands = true;
        options.allow_translation_commands = false;
        options.allow_emergency_stop = true;
        options.max_speed_normalized = 0.05;
        options.max_ttl_s = 0.50;
        options.max_command_age_s = 0.50;
        SoftwareMotionShadowAdapter adapter(
            transport_,
            SoftwareMotionTransportContractChecker(options));

        expect_accept(result, adapter, stop(start_time_s, 1), "stop_idempotent_first");
        expect_accept(result, adapter, stop(start_time_s + 0.01, 2), "stop_idempotent_second");
        expect_accept(result, adapter, emergency_stop(start_time_s + 0.02, 3), "emergency_stop_priority");
        expect_accept(result,
                      adapter,
                      motion(SoftwareMotionDirection::TurnLeft,
                             start_time_s + 0.03,
                             0.03,
                             4,
                             SoftwareMotionCommandSource::ManualTest,
                             "direction_probe_left"),
                      "turn_left_shadow");
        expect_accept(result,
                      adapter,
                      motion(SoftwareMotionDirection::TurnRight,
                             start_time_s + 0.04,
                             0.03,
                             5,
                             SoftwareMotionCommandSource::ManualTest,
                             "direction_probe_right"),
                      "turn_right_shadow");
        expect_reject(result,
                      adapter,
                      motion(SoftwareMotionDirection::Forward,
                             start_time_s + 0.05,
                             0.03,
                             6,
                             SoftwareMotionCommandSource::ManualTest,
                             "forward_disabled"),
                      "forward_disabled");
        expect_reject(result,
                      adapter,
                      motion(SoftwareMotionDirection::Backward,
                             start_time_s + 0.06,
                             0.03,
                             7,
                             SoftwareMotionCommandSource::ManualTest,
                             "backward_disabled"),
                      "backward_disabled");
        expect_reject(result,
                      adapter,
                      motion(SoftwareMotionDirection::TurnLeft,
                             start_time_s + 0.07,
                             0.06,
                             8,
                             SoftwareMotionCommandSource::ManualTest,
                             "invalid_speed"),
                      "invalid_speed_rejected");

        auto invalid_ttl = motion(SoftwareMotionDirection::TurnLeft,
                                  start_time_s + 0.08,
                                  0.03,
                                  9,
                                  SoftwareMotionCommandSource::ManualTest,
                                  "invalid_ttl");
        invalid_ttl.ttl_s = 0.0;
        expect_reject(result, adapter, invalid_ttl, "invalid_ttl_rejected");

        result.ok = result.failed.empty();
        return result;
    }

private:
    static SoftwareMotionCommand stop(double timestamp_s, uint64_t sequence) {
        SoftwareMotionCommand command;
        command.direction = SoftwareMotionDirection::Stop;
        command.speed_normalized = 0.0;
        command.timestamp_s = timestamp_s;
        command.ttl_s = 0.30;
        command.source = SoftwareMotionCommandSource::SafetyStop;
        command.reason = "safety_stop";
        command.sequence = sequence;
        return command;
    }

    static SoftwareMotionCommand emergency_stop(double timestamp_s, uint64_t sequence) {
        SoftwareMotionCommand command = stop(timestamp_s, sequence);
        command.direction = SoftwareMotionDirection::EmergencyStop;
        command.reason = "emergency_stop";
        return command;
    }

    static SoftwareMotionCommand motion(SoftwareMotionDirection direction,
                                        double timestamp_s,
                                        double speed,
                                        uint64_t sequence,
                                        SoftwareMotionCommandSource source,
                                        const std::string &reason) {
        SoftwareMotionCommand command;
        command.direction = direction;
        command.speed_normalized = speed;
        command.timestamp_s = timestamp_s;
        command.ttl_s = 0.30;
        command.source = source;
        command.reason = reason;
        command.sequence = sequence;
        return command;
    }

    static void expect_accept(SoftwareTransportAcceptanceResult &result,
                              SoftwareMotionShadowAdapter &adapter,
                              const SoftwareMotionCommand &command,
                              const std::string &name) {
        auto send_result = adapter.send_command(command);
        if (send_result.ok && send_result.accepted) {
            result.passed.push_back(name);
        } else {
            result.failed.push_back(name);
        }
    }

    static void expect_reject(SoftwareTransportAcceptanceResult &result,
                              SoftwareMotionShadowAdapter &adapter,
                              const SoftwareMotionCommand &command,
                              const std::string &name) {
        auto send_result = adapter.send_command(command);
        if (send_result.ok && !send_result.accepted && !send_result.error.empty()) {
            result.passed.push_back(name);
        } else {
            result.failed.push_back(name);
        }
    }

    std::shared_ptr<SoftwareMotionCommandTransport> transport_;
};

} // namespace robot_slamd
