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
            record_fail(result, "transport_missing");
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

        // Stop idempotent.
        expect_accept(result,
                      adapter,
                      make_stop(start_time_s, 1),
                      "stop_idempotent_first");
        expect_accept(result,
                      adapter,
                      make_stop(start_time_s + 0.01, 2),
                      "stop_idempotent_second");

        // Emergency stop.
        expect_accept(result,
                      adapter,
                      make_emergency_stop(start_time_s + 0.02, 3),
                      "emergency_stop_priority");

        // Turn left shadow.
        expect_accept(result,
                      adapter,
                      make_turn_left(start_time_s + 0.03, 4),
                      "turn_left_shadow");

        // Turn right shadow.
        expect_accept(result,
                      adapter,
                      make_turn_right(start_time_s + 0.04, 5),
                      "turn_right_shadow");

        // Forward disabled.
        expect_reject(result,
                      adapter,
                      make_forward(start_time_s + 0.05, 6),
                      "forward_disabled");

        // Backward disabled.
        expect_reject(result,
                      adapter,
                      make_backward(start_time_s + 0.06, 7),
                      "backward_disabled");

        // Invalid speed rejected.
        auto invalid_speed = make_turn_left(start_time_s + 0.07, 8);
        invalid_speed.speed_normalized = 0.06;
        invalid_speed.reason = "invalid_speed";
        expect_reject(result,
                      adapter,
                      invalid_speed,
                      "invalid_speed_rejected");

        // Invalid ttl rejected.
        auto invalid_ttl = make_turn_left(start_time_s + 0.08, 9);
        invalid_ttl.ttl_s = 0.0;
        invalid_ttl.reason = "invalid_ttl";
        expect_reject(result, adapter, invalid_ttl, "invalid_ttl_rejected");

        // Transport reject propagates through expect_accept failures.
        result.ok = result.failed.empty();
        return result;
    }

private:
    static SoftwareMotionCommand make_stop(double timestamp_s, uint64_t sequence) {
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

    static SoftwareMotionCommand make_emergency_stop(double timestamp_s,
                                                     uint64_t sequence) {
        SoftwareMotionCommand command = make_stop(timestamp_s, sequence);
        command.direction = SoftwareMotionDirection::EmergencyStop;
        command.reason = "emergency_stop";
        return command;
    }

    static SoftwareMotionCommand make_turn_left(double timestamp_s,
                                                uint64_t sequence) {
        return make_motion(SoftwareMotionDirection::TurnLeft,
                           timestamp_s,
                           0.03,
                           sequence,
                           SoftwareMotionCommandSource::ManualTest,
                           "direction_probe_left");
    }

    static SoftwareMotionCommand make_turn_right(double timestamp_s,
                                                 uint64_t sequence) {
        return make_motion(SoftwareMotionDirection::TurnRight,
                           timestamp_s,
                           0.03,
                           sequence,
                           SoftwareMotionCommandSource::ManualTest,
                           "direction_probe_right");
    }

    static SoftwareMotionCommand make_forward(double timestamp_s,
                                              uint64_t sequence) {
        return make_motion(SoftwareMotionDirection::Forward,
                           timestamp_s,
                           0.03,
                           sequence,
                           SoftwareMotionCommandSource::ManualTest,
                           "forward_disabled");
    }

    static SoftwareMotionCommand make_backward(double timestamp_s,
                                               uint64_t sequence) {
        return make_motion(SoftwareMotionDirection::Backward,
                           timestamp_s,
                           0.03,
                           sequence,
                           SoftwareMotionCommandSource::ManualTest,
                           "backward_disabled");
    }

    static SoftwareMotionCommand make_motion(SoftwareMotionDirection direction,
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

    static void record_pass(SoftwareTransportAcceptanceResult &result,
                            const std::string &name) {
        result.passed.push_back(name);
    }

    static void record_fail(SoftwareTransportAcceptanceResult &result,
                            const std::string &name) {
        result.failed.push_back(name);
    }

    static void expect_accept(SoftwareTransportAcceptanceResult &result,
                              SoftwareMotionShadowAdapter &adapter,
                              const SoftwareMotionCommand &command,
                              const std::string &name) {
        auto send_result = adapter.send_command(command);
        if (send_result.ok && send_result.accepted) {
            record_pass(result, name);
        } else {
            record_fail(result, name);
        }
    }

    static void expect_reject(SoftwareTransportAcceptanceResult &result,
                              SoftwareMotionShadowAdapter &adapter,
                              const SoftwareMotionCommand &command,
                              const std::string &name) {
        auto send_result = adapter.send_command(command);
        if (send_result.ok && !send_result.accepted && !send_result.error.empty()) {
            record_pass(result, name);
        } else {
            record_fail(result, name);
        }
    }

    std::shared_ptr<SoftwareMotionCommandTransport> transport_;
};

} // namespace robot_slamd
