#include "robot_slamd/motion/algorithm_motion_command.hpp"

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

robot_slamd::AlgorithmMotionCommand base(robot_slamd::AlgorithmMotionKind kind,
                                         robot_slamd::AlgorithmMotionProfile profile) {
    robot_slamd::AlgorithmMotionCommand c;
    c.kind = kind;
    c.profile = profile;
    c.timestamp_s = 1.0;
    c.ttl_s = 0.3;
    c.duration_s = 0.2;
    c.speed_normalized = 0.02;
    c.reason = "test_motion";
    c.sequence = 1;
    return c;
}

robot_slamd::AlgorithmMotionValidationResult validate(const robot_slamd::AlgorithmMotionCommand &c,
                                                      bool allow_translation = false,
                                                      bool allow_rotation = true,
                                                      bool allow_recovery = false,
                                                      bool allow_manual = false) {
    return robot_slamd::validate_algorithm_motion_command(c,
                                                          0.03,
                                                          0.05,
                                                          0.05,
                                                          0.03,
                                                          0.30,
                                                          0.50,
                                                          0.50,
                                                          0.30,
                                                          allow_translation,
                                                          allow_rotation,
                                                          allow_recovery,
                                                          allow_manual);
}

void expect_error(robot_slamd::AlgorithmMotionCommand c, const char *error) {
    auto r = validate(c);
    expect(!r.ok, "validation should fail");
    expect(r.error == error, error);
}
} // namespace

int main() {
    using namespace robot_slamd;

    auto c = base(AlgorithmMotionKind::Stop, AlgorithmMotionProfile::Safety);
    c.speed_normalized = 0.0;
    c.duration_s = 0.0;
    auto r = validate(c);
    expect(r.ok, "stop zero passes");
    c.speed_normalized = 0.1;
    expect_error(c, "stop_requires_zero_speed");

    c = base(AlgorithmMotionKind::EmergencyStop, AlgorithmMotionProfile::Safety);
    c.speed_normalized = 0.0;
    c.duration_s = 0.0;
    r = validate(c);
    expect(r.ok, "emergency stop zero passes");
    c.speed_normalized = 0.1;
    expect_error(c, "emergency_stop_requires_zero_speed");

    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.timestamp_s = std::numeric_limits<double>::quiet_NaN();
    expect_error(c, "non_finite_timestamp");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.ttl_s = 0.0;
    expect_error(c, "invalid_ttl");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.duration_s = std::numeric_limits<double>::quiet_NaN();
    expect_error(c, "non_finite_duration");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.duration_s = -0.1;
    expect_error(c, "negative_duration");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.speed_normalized = std::numeric_limits<double>::quiet_NaN();
    expect_error(c, "non_finite_speed");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.speed_normalized = 1.1;
    expect_error(c, "speed_out_of_range");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.duration_s = 0.0;
    expect_error(c, "motion_requires_positive_duration");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.reason.clear();
    expect_error(c, "motion_reason_required");

    c = base(AlgorithmMotionKind::DirectionProbeLeft, AlgorithmMotionProfile::DirectionProbe);
    c.speed_normalized = 0.031;
    expect_error(c, "direction_probe_speed_too_high");
    c = base(AlgorithmMotionKind::DirectionProbeRight, AlgorithmMotionProfile::DirectionProbe);
    c.duration_s = 0.31;
    expect_error(c, "direction_probe_duration_too_long");
    c = base(AlgorithmMotionKind::ActiveScanTurnLeft, AlgorithmMotionProfile::ActiveScan);
    c.speed_normalized = 0.051;
    expect_error(c, "active_scan_speed_too_high");
    c = base(AlgorithmMotionKind::ActiveScanTurnRight, AlgorithmMotionProfile::ActiveScan);
    c.duration_s = 0.51;
    expect_error(c, "active_scan_duration_too_long");

    c = base(AlgorithmMotionKind::RecoveryForward, AlgorithmMotionProfile::Recovery);
    r = validate(c, true, true, false, false);
    expect(!r.ok && r.error == "recovery_commands_disabled", "recovery disabled");
    c = base(AlgorithmMotionKind::Forward, AlgorithmMotionProfile::ManualTest);
    r = validate(c, false, true, false, true);
    expect(!r.ok && r.error == "translation_commands_disabled", "translation disabled");
    c = base(AlgorithmMotionKind::TurnRight, AlgorithmMotionProfile::ManualTest);
    r = validate(c, false, false, false, true);
    expect(!r.ok && r.error == "rotation_commands_disabled", "rotation disabled");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    r = validate(c, false, true, false, false);
    expect(!r.ok && r.error == "manual_test_commands_disabled", "manual disabled");

    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.speed_normalized = 0.031;
    r = validate(c, false, true, false, true);
    expect(!r.ok && r.error == "manual_test_speed_too_high", "manual speed limit");
    c = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::ManualTest);
    c.duration_s = 0.31;
    r = validate(c, false, true, false, true);
    expect(!r.ok && r.error == "manual_test_duration_too_long", "manual duration limit");

    expect(to_string(AlgorithmMotionKind::ActiveScanTurnLeft) == "ACTIVE_SCAN_TURN_LEFT", "kind string");
    expect(to_string(AlgorithmMotionProfile::Recovery) == "RECOVERY", "profile string");
    expect(algorithm_motion_kind_id(AlgorithmMotionKind::RecoveryTurnRight) == 13, "kind id");
    expect(algorithm_motion_profile_id(AlgorithmMotionProfile::ManualTest) == 4, "profile id");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
