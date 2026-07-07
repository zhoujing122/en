#include "robot_slamd/software_motion/software_motion_command.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_error(const robot_slamd::SoftwareMotionCommand &c,
                  const char *error,
                  bool allow_translation = false,
                  bool allow_rotation = false) {
    auto r = robot_slamd::validate_software_motion_command(c, 0.10, allow_translation, allow_rotation, true);
    expect(!r.ok, "validation should fail");
    expect(r.error == error, error);
}

robot_slamd::SoftwareMotionCommand base(robot_slamd::SoftwareMotionDirection d) {
    robot_slamd::SoftwareMotionCommand c;
    c.direction = d;
    c.timestamp_s = 1.0;
    c.ttl_s = 0.3;
    c.source = robot_slamd::SoftwareMotionCommandSource::ActiveScan;
    c.speed_normalized = 0.05;
    return c;
}
} // namespace

int main() {
    using namespace robot_slamd;

    SoftwareMotionCommand c = base(SoftwareMotionDirection::Stop);
    c.speed_normalized = 0.0;
    auto r = validate_software_motion_command(c, 0.10, false, false, true);
    expect(r.ok, "stop zero should pass");

    c.speed_normalized = 0.1;
    expect_error(c, "stop_requires_zero_speed");

    c = base(SoftwareMotionDirection::EmergencyStop);
    c.speed_normalized = 0.0;
    r = validate_software_motion_command(c, 0.10, false, false, true);
    expect(r.ok, "emergency stop zero should pass");

    c.speed_normalized = 0.1;
    expect_error(c, "emergency_stop_requires_zero_speed");

    c = base(SoftwareMotionDirection::TurnLeft);
    expect_error(c, "rotation_commands_disabled", false, false);
    r = validate_software_motion_command(c, 0.10, false, true, true);
    expect(r.ok, "turn left should pass with rotation enabled");

    c = base(SoftwareMotionDirection::TurnRight);
    r = validate_software_motion_command(c, 0.10, false, true, true);
    expect(r.ok, "turn right should pass with rotation enabled");

    c = base(SoftwareMotionDirection::Forward);
    expect_error(c, "translation_commands_disabled", false, true);

    c = base(SoftwareMotionDirection::Backward);
    expect_error(c, "translation_commands_disabled", false, true);

    c = base(SoftwareMotionDirection::TurnLeft);
    c.speed_normalized = 1.2;
    expect_error(c, "speed_out_of_range", false, true);

    c = base(SoftwareMotionDirection::TurnLeft);
    c.speed_normalized = 0.2;
    expect_error(c, "speed_exceeds_limit", false, true);

    c = base(SoftwareMotionDirection::TurnLeft);
    c.timestamp_s = std::numeric_limits<double>::quiet_NaN();
    expect_error(c, "non_finite_timestamp", false, true);

    c = base(SoftwareMotionDirection::TurnLeft);
    c.ttl_s = 0.0;
    expect_error(c, "invalid_ttl", false, true);

    c = base(SoftwareMotionDirection::TurnLeft);
    c.source = SoftwareMotionCommandSource::Unknown;
    expect_error(c, "source_required_for_motion", false, true);

    expect(to_string(SoftwareMotionDirection::TurnLeft) == "TURN_LEFT", "direction string");
    expect(to_string(SoftwareMotionCommandSource::ActiveScan) == "ACTIVE_SCAN", "source string");
    expect(software_motion_direction_id(SoftwareMotionDirection::EmergencyStop) == 5, "direction id");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
