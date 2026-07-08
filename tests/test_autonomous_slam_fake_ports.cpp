#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"
#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"

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

    FakeRobotSlamSensorPort sensor;
    sensor.snapshot.has_tof = false;
    expect(!sensor.latest_snapshot(1.0).has_tof, "fake sensor can simulate missing tof");
    sensor.snapshot.has_imu = false;
    sensor.snapshot.has_wheel = false;
    auto missing_odom = sensor.latest_snapshot(1.1);
    expect(!missing_odom.has_imu && !missing_odom.has_wheel,
           "fake sensor can simulate missing imu and wheel");

    FakeRobotSlamMotionPort motion;
    AlgorithmMotionCommandBuilder builder;
    motion.transport_fail_next_command = true;
    auto failed = motion.send_algorithm_command(builder.active_scan_turn_left(2.0));
    expect(!failed.ok && failed.error == "fake_motion_transport_failed",
           "fake motion transport fail next command");
    motion.feedback.fault = true;
    motion.feedback.fault_reason = "fake_feedback_fault";
    expect(motion.latest_feedback(2.1).fault, "fake motion feedback fault");

    FakeRobotSlamMapPort map;
    expect(map.save_map("/tmp/fake_autonomous_slam_map"), "fake map save succeeds");
    expect(map.saved_map_path == "/tmp/fake_autonomous_slam_map",
           "fake map records save path");
    expect(map.save_count == 1, "fake map save count");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
