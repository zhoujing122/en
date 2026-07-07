#include "robot_slamd/software_motion/loopback_software_motion_transport.hpp"
#include "robot_slamd/software_motion/software_motion_shadow_adapter.hpp"

#include <iostream>
#include <memory>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::SoftwareMotionCommand command(robot_slamd::SoftwareMotionDirection direction,
                                           double speed = 0.0,
                                           uint64_t sequence = 1) {
    robot_slamd::SoftwareMotionCommand c;
    c.direction = direction;
    c.speed_normalized = speed;
    c.timestamp_s = 2.0;
    c.ttl_s = 0.30;
    c.sequence = sequence;
    c.reason = "shadow_adapter_test";
    c.source = robot_slamd::SoftwareMotionCommandSource::ManualTest;
    if (direction == robot_slamd::SoftwareMotionDirection::Stop ||
        direction == robot_slamd::SoftwareMotionDirection::EmergencyStop) {
        c.source = robot_slamd::SoftwareMotionCommandSource::SafetyStop;
    }
    return c;
}

class BadResultTransport final : public robot_slamd::SoftwareMotionCommandTransport {
public:
    robot_slamd::SoftwareMotionSendResult send_command(
        const robot_slamd::SoftwareMotionCommand &command) override {
        send_count++;
        return {false, false, "", send_count, command.timestamp_s};
    }

    uint64_t send_count = 0;
};
} // namespace

int main() {
    using namespace robot_slamd;

    SoftwareMotionShadowAdapter missing(nullptr, SoftwareMotionTransportContractChecker());
    auto missing_result = missing.send_command(command(SoftwareMotionDirection::Stop));
    expect(!missing_result.ok && missing_result.error == "shadow_adapter_inner_missing",
           "missing inner fails closed");

    auto loopback = std::make_shared<LoopbackSoftwareMotionCommandTransport>();
    SoftwareMotionShadowAdapter adapter(loopback, SoftwareMotionTransportContractChecker());

    auto invalid = command(SoftwareMotionDirection::TurnLeft, 0.06, 1);
    auto invalid_result = adapter.send_command(invalid);
    expect(invalid_result.ok && !invalid_result.accepted && invalid_result.error == "speed_exceeds_limit",
           "invalid command rejected by checker");
    expect(loopback->send_count == 0, "invalid command does not call inner");

    auto stop_result = adapter.send_command(command(SoftwareMotionDirection::Stop));
    expect(stop_result.ok && stop_result.accepted, "valid STOP calls inner");
    expect(loopback->send_count == 1, "STOP increments inner count");

    auto turn_result = adapter.send_command(command(SoftwareMotionDirection::TurnLeft, 0.03, 2));
    expect(turn_result.ok && turn_result.accepted, "valid TURN_LEFT calls inner");
    expect(loopback->send_count == 2, "TURN_LEFT increments inner count");

    auto forward = adapter.send_command(command(SoftwareMotionDirection::Forward, 0.03, 3));
    expect(forward.ok && !forward.accepted && forward.error == "translation_commands_disabled",
           "forward disabled rejected by adapter");

    loopback->reject_non_stop = true;
    auto rejected = adapter.send_command(command(SoftwareMotionDirection::TurnRight, 0.03, 4));
    expect(rejected.ok && !rejected.accepted && rejected.error == "loopback_non_stop_rejected",
           "inner reject recorded");

    auto bad = std::make_shared<BadResultTransport>();
    SoftwareMotionShadowAdapter bad_adapter(bad, SoftwareMotionTransportContractChecker());
    auto bad_result = bad_adapter.send_command(command(SoftwareMotionDirection::Stop));
    expect(bad_result.ok && !bad_result.accepted && bad_result.error == "failed_result_requires_error",
           "send result contract failure recorded");

    expect(adapter.history().size() == 5, "history records all adapter attempts");
    adapter.clear_history();
    expect(adapter.history().empty(), "clear_history clears history");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
