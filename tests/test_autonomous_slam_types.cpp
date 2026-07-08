#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

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

    expect(to_string(AutonomousSlamState::Idle) == "idle", "Idle string");
    expect(to_string(AutonomousSlamState::Fault) == "fault", "Fault string");
    expect(autonomous_slam_state_id(AutonomousSlamState::Idle) == 0, "Idle id stable");
    expect(autonomous_slam_state_id(AutonomousSlamState::Fault) == 9, "Fault id stable");
    expect(autonomous_slam_fault_id(AutonomousSlamFault::None) == 0, "None fault id stable");
    expect(autonomous_slam_fault_id(AutonomousSlamFault::SafetyGateBlocked) == 6,
           "SafetyGateBlocked id stable");

    RobotSlamSensorSnapshot snapshot;
    expect(!snapshot.has_tof, "default snapshot has no tof");
    RobotSlamMapQuality quality;
    expect(!quality.good_enough, "default quality not good enough");
    RobotSlamMotionFeedback feedback;
    expect(feedback.command_settled, "default feedback settled");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
