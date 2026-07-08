#include "robot_slamd/autonomy/adapters/replay_robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/adapters/replay_robot_slam_sensor_port.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::RobotSlamSensorSnapshot snapshot_with_range(double range) {
    robot_slamd::RobotSlamSensorSnapshot snapshot;
    snapshot.has_tof = true;
    snapshot.tof.timestamp_s = 100.0;
    snapshot.tof.ranges_m = {range, range, range};
    snapshot.tof.angle_increment_rad = 0.1;
    snapshot.tof.max_range_m = 3.0;
    snapshot.tof.frame_id = "replay";
    snapshot.has_imu = true;
    snapshot.imu.timestamp_s = 100.0;
    return snapshot;
}

robot_slamd::RobotSlamMapQuality quality_with_count(int count) {
    robot_slamd::RobotSlamMapQuality quality;
    quality.good_enough = count > 1;
    quality.coverage_ratio = 0.1 * count;
    quality.yaw_coverage_ratio = 0.1 * count;
    quality.valid_scan_count = count;
    quality.reason = "replay_quality";
    return quality;
}
} // namespace

int main() {
    using namespace robot_slamd;

    ReplayRobotSlamSensorPort sensor({snapshot_with_range(0.5), snapshot_with_range(0.8)});
    expect(sensor.ready(), "replay sensor ready");
    expect(sensor.latest_snapshot(100.0).tof.ranges_m.front() == 0.5, "first replay sensor frame");
    expect(sensor.latest_snapshot(100.1).tof.ranges_m.front() == 0.8, "second replay sensor frame");
    expect(sensor.latest_snapshot(100.2).tof.ranges_m.front() == 0.8, "replay sensor holds last frame");
    sensor.set_ready(false);
    expect(!sensor.ready(), "replay sensor set ready false");

    ReplayRobotSlamMapPort map({quality_with_count(1), quality_with_count(2)});
    expect(map.ready(), "replay map ready");
    RobotSlamSensorSnapshot snapshot = snapshot_with_range(0.6);
    expect(map.integrate_sensor_snapshot(snapshot, 101.0), "replay map integrate succeeds");
    expect(map.integrate_count() == 1, "replay map integrate count");
    expect(map.latest_quality(101.0).valid_scan_count == 1, "first replay map quality");
    expect(map.latest_quality(101.1).valid_scan_count == 2, "second replay map quality");
    expect(map.latest_quality(101.2).valid_scan_count == 2, "replay map holds last quality");
    expect(map.save_map("/tmp/should_not_write_map"), "replay map save records path");
    expect(map.last_saved_path() == "/tmp/should_not_write_map", "replay map save path recorded");
    map.set_ready(false);
    expect(!map.ready(), "replay map set ready false");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
