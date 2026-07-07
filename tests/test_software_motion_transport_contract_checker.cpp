#include "robot_slamd/software_motion/software_motion_transport_contract_checker.hpp"

#include <iostream>
#include <limits>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::SoftwareMotionCommand command(robot_slamd::SoftwareMotionDirection direction,
                                           double timestamp_s,
                                           double speed,
                                           double ttl_s,
                                           uint64_t sequence,
                                           const std::string &reason = "contract_test") {
    robot_slamd::SoftwareMotionCommand c;
    c.direction = direction;
    c.timestamp_s = timestamp_s;
    c.speed_normalized = speed;
    c.ttl_s = ttl_s;
    c.sequence = sequence;
    c.reason = reason;
    c.source = robot_slamd::SoftwareMotionCommandSource::ManualTest;
    if (direction == robot_slamd::SoftwareMotionDirection::Stop ||
        direction == robot_slamd::SoftwareMotionDirection::EmergencyStop) {
        c.source = robot_slamd::SoftwareMotionCommandSource::SafetyStop;
    }
    return c;
}

void expect_error(const robot_slamd::SoftwareMotionTransportContractChecker &checker,
                  const robot_slamd::SoftwareMotionCommand &c,
                  double now_s,
                  const std::string &expected) {
    auto r = checker.check_command(c, now_s);
    expect(!r.ok && r.error == expected, expected.c_str());
}
} // namespace

int main() {
    using namespace robot_slamd;
    SoftwareMotionTransportContractOptions options;
    SoftwareMotionTransportContractChecker checker(options);

    expect(checker.check_command(command(SoftwareMotionDirection::Stop, 1.0, 0.0, 0.3, 0), 1.0).ok,
           "valid STOP passes");
    expect_error(checker,
                 command(SoftwareMotionDirection::Stop, 1.0, 0.1, 0.3, 0),
                 1.0,
                 "stop_requires_zero_speed");
    expect_error(checker,
                 command(SoftwareMotionDirection::EmergencyStop, 1.0, 0.1, 0.3, 0),
                 1.0,
                 "emergency_stop_requires_zero_speed");
    expect_error(checker,
                 command(SoftwareMotionDirection::TurnLeft, 1.0, 0.03, 0.0, 1),
                 1.0,
                 "invalid_ttl");
    expect_error(checker,
                 command(SoftwareMotionDirection::TurnLeft, 1.0, 0.03, 0.51, 1),
                 1.0,
                 "ttl_too_large");
    expect_error(checker,
                 command(SoftwareMotionDirection::TurnLeft,
                         1.0,
                         std::numeric_limits<double>::quiet_NaN(),
                         0.3,
                         1),
                 1.0,
                 "non_finite_speed");
    expect_error(checker,
                 command(SoftwareMotionDirection::TurnLeft, 1.0, 1.1, 0.3, 1),
                 1.0,
                 "speed_out_of_range");
    expect_error(checker,
                 command(SoftwareMotionDirection::TurnLeft, 1.0, 0.06, 0.3, 1),
                 1.0,
                 "speed_exceeds_limit");
    expect_error(checker,
                 command(SoftwareMotionDirection::Forward, 1.0, 0.03, 0.3, 1),
                 1.0,
                 "translation_commands_disabled");
    expect_error(checker,
                 command(SoftwareMotionDirection::Backward, 1.0, 0.03, 0.3, 1),
                 1.0,
                 "translation_commands_disabled");

    SoftwareMotionTransportContractOptions no_rotation = options;
    no_rotation.allow_rotation_commands = false;
    SoftwareMotionTransportContractChecker rotation_checker(no_rotation);
    expect_error(rotation_checker,
                 command(SoftwareMotionDirection::TurnLeft, 1.0, 0.03, 0.3, 1),
                 1.0,
                 "rotation_commands_disabled");

    SoftwareMotionTransportContractOptions no_emergency = options;
    no_emergency.allow_emergency_stop = false;
    SoftwareMotionTransportContractChecker emergency_checker(no_emergency);
    expect_error(emergency_checker,
                 command(SoftwareMotionDirection::EmergencyStop, 1.0, 0.0, 0.3, 0),
                 1.0,
                 "emergency_stop_disabled");

    expect_error(checker,
                 command(SoftwareMotionDirection::TurnLeft, 1.0, 0.03, 0.3, 1),
                 1.6,
                 "command_too_old");
    expect_error(checker,
                 command(SoftwareMotionDirection::TurnLeft, 1.0, 0.03, 0.3, 1, ""),
                 1.0,
                 "reason_required");

    auto warning = checker.check_command(
        command(SoftwareMotionDirection::TurnLeft, 1.0, 0.03, 0.3, 0),
        1.0);
    expect(warning.ok, "sequence zero warning still ok");
    expect(!warning.warnings.empty() && warning.warnings[0] == "sequence_zero_for_motion",
           "sequence zero warning present");

    SoftwareMotionSendResult ok_result{true, true, "", 1, 1.0};
    expect(checker.check_send_result(command(SoftwareMotionDirection::Stop, 1.0, 0.0, 0.3, 0), ok_result, 1.0).ok,
           "accepted result passes");
    expect(checker.check_send_result(command(SoftwareMotionDirection::Stop, 1.0, 0.0, 0.3, 0), {false, false, "", 2, 1.0}, 1.0).error == "failed_result_requires_error",
           "failed result requires error");
    expect(checker.check_send_result(command(SoftwareMotionDirection::Stop, 1.0, 0.0, 0.3, 0), {true, false, "", 3, 1.0}, 1.0).error == "rejected_result_requires_error",
           "rejected result requires error");
    expect(checker.check_send_result(command(SoftwareMotionDirection::Stop, 1.0, 0.0, 0.3, 0), {true, true, "", 4, 0.9}, 1.0).error == "result_before_command",
           "result before command rejected");
    expect(checker.check_send_result(command(SoftwareMotionDirection::Stop, 1.0, 0.0, 0.3, 0), {true, true, "", 0, 1.0}, 1.0).error == "transport_sequence_regressed",
           "transport sequence regression rejected");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
