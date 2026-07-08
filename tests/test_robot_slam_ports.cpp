#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"
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
    expect(sensor.ready(), "fake sensor ready");
    auto snapshot = sensor.latest_snapshot(10.0);
    expect(snapshot.has_tof && snapshot.has_imu && snapshot.has_wheel, "snapshot has required data");
    sensor.ready_flag = false;
    expect(sensor.status() == "fake_sensor_not_ready", "sensor not ready status");

    FakeRobotSlamMotionPort motion;
    AlgorithmMotionCommandBuilder builder;
    auto command = builder.active_scan_turn_left(10.0);
    auto sent = motion.send_algorithm_command(command);
    expect(sent.ok && sent.accepted, "fake motion accepts command");
    expect(motion.sent_commands.size() == 1, "fake motion records command");
    motion.reject_next_command = true;
    auto rejected = motion.send_algorithm_command(builder.active_scan_turn_right(11.0));
    expect(!rejected.ok && rejected.error == "fake_motion_rejected", "fake motion reject next");

    FakeRobotSlamMapPort map(2);
    expect(!map.latest_quality(10.0).good_enough, "map initially poor");
    map.integrate_sensor_snapshot(snapshot, 10.0);
    expect(!map.latest_quality(10.0).good_enough, "map still poor after one scan");
    map.integrate_sensor_snapshot(snapshot, 10.1);
    expect(map.latest_quality(10.1).good_enough, "map good after configured scans");

    FixedStepTimePort time(100.0, 0.25);
    expect(time.now_s() == 100.0, "time starts at configured value");
    time.advance();
    expect(time.now_s() == 100.25, "time advances deterministically");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
