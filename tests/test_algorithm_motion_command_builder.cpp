#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"

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
void expect_near(double a, double b, const char *message) {
    expect(std::fabs(a - b) < 1e-12, message);
}
} // namespace

int main() {
    using namespace robot_slamd;
    AlgorithmMotionCommandBuilder builder;

    auto stop = builder.stop(1.0);
    expect(stop.kind == AlgorithmMotionKind::Stop, "stop kind");
    expect(stop.profile == AlgorithmMotionProfile::Safety, "stop safety profile");
    expect_near(stop.speed_normalized, 0.0, "stop speed zero");

    auto emergency = builder.emergency_stop(1.1);
    expect(emergency.kind == AlgorithmMotionKind::EmergencyStop, "emergency kind");
    expect_near(emergency.speed_normalized, 0.0, "emergency speed zero");

    auto dpl = builder.direction_probe_left(2.0);
    expect(dpl.kind == AlgorithmMotionKind::DirectionProbeLeft, "direction probe left kind");
    expect(dpl.profile == AlgorithmMotionProfile::DirectionProbe, "direction probe profile");
    expect_near(dpl.speed_normalized, 0.03, "direction probe speed");
    expect_near(dpl.duration_s, 0.30, "direction probe duration");

    auto dpr = builder.direction_probe_right(2.1);
    expect(dpr.kind == AlgorithmMotionKind::DirectionProbeRight, "direction probe right kind");

    auto asl = builder.active_scan_turn_left(3.0);
    expect(asl.kind == AlgorithmMotionKind::ActiveScanTurnLeft, "active scan left kind");
    expect(asl.profile == AlgorithmMotionProfile::ActiveScan, "active scan profile");
    expect_near(asl.speed_normalized, 0.05, "active scan speed");
    expect_near(asl.duration_s, 0.50, "active scan duration");

    auto asr = builder.active_scan_turn_right(3.1);
    expect(asr.kind == AlgorithmMotionKind::ActiveScanTurnRight, "active scan right kind");

    auto rf = builder.recovery_forward(4.0);
    expect(rf.kind == AlgorithmMotionKind::RecoveryForward, "recovery forward kind");
    expect(rf.profile == AlgorithmMotionProfile::Recovery, "recovery profile");
    auto rb = builder.recovery_backward(4.1);
    expect(rb.kind == AlgorithmMotionKind::RecoveryBackward, "recovery backward kind");
    expect(builder.recovery_turn_left(4.2).kind == AlgorithmMotionKind::RecoveryTurnLeft, "recovery turn left");
    expect(builder.recovery_turn_right(4.3).kind == AlgorithmMotionKind::RecoveryTurnRight, "recovery turn right");

    auto m1 = builder.manual_turn_left(5.0);
    auto m2 = builder.manual_turn_right(5.1);
    auto m3 = builder.manual_forward(5.2);
    auto m4 = builder.manual_backward(5.3);
    expect(m1.profile == AlgorithmMotionProfile::ManualTest, "manual profile");
    expect(m2.sequence == m1.sequence + 1, "sequence increments 1");
    expect(m3.sequence == m2.sequence + 1, "sequence increments 2");
    expect(m4.sequence == m3.sequence + 1, "sequence increments 3");
    expect_near(m4.ttl_s, 0.30, "default ttl");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
