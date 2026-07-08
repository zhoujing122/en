#include "robot_slamd/autonomy/autonomous_slam_policy.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
} // namespace

int main() {
    using namespace robot_slamd;

    AlgorithmMotionCommandBuilder builder;
    AutonomousSlamPolicy policy;

    RobotSlamMapQuality good;
    good.good_enough = true;
    auto good_decision = policy.decide(good, builder, 10.0);
    expect(good_decision.completed, "good quality completes");
    expect(good_decision.should_stop, "good quality stops");
    expect(good_decision.command.kind == AlgorithmMotionKind::Stop, "good quality stop command");

    RobotSlamMapQuality poor;
    poor.good_enough = false;
    auto poor_decision = policy.decide(poor, builder, 11.0);
    expect(poor_decision.should_send_motion, "poor quality sends motion");
    expect(poor_decision.command.kind == AlgorithmMotionKind::ActiveScanTurnLeft,
           "poor quality prefers left");
    expect(poor_decision.command.speed_normalized <= 0.05, "active scan speed capped");
    expect(poor_decision.command.duration_s <= 0.50, "active scan duration capped");
    expect(policy.active_scan_command_count() == 1, "active scan count increments");

    AutonomousSlamPolicyOptions right_options;
    right_options.prefer_left_turn = false;
    AutonomousSlamPolicy right_policy(right_options);
    auto right_decision = right_policy.decide(poor, builder, 12.0);
    expect(right_decision.command.kind == AlgorithmMotionKind::ActiveScanTurnRight,
           "right preference respected");

    AutonomousSlamPolicyOptions limited_options;
    limited_options.max_active_scan_commands = 1;
    AutonomousSlamPolicy limited(limited_options);
    limited.decide(poor, builder, 13.0);
    auto limited_decision = limited.decide(poor, builder, 14.0);
    expect(limited_decision.should_stop, "max active scan stops");
    expect(limited_decision.reason == "max_active_scan_commands_reached",
           "max active scan reason stable");
    expect(limited_decision.command.kind == AlgorithmMotionKind::Stop,
           "max active scan does not move");

    expect(poor_decision.command.kind != AlgorithmMotionKind::Forward &&
               poor_decision.command.kind != AlgorithmMotionKind::Backward,
           "policy does not generate translation");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
