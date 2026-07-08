#include "robot_slamd/autonomy/adapters/replay_robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/adapters/replay_robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_acceptance_runner.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"

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

robot_slamd::RobotSlamSensorSnapshot valid_snapshot(double now_s) {
    robot_slamd::RobotSlamSensorSnapshot snapshot;
    snapshot.has_tof = true;
    snapshot.tof.timestamp_s = now_s;
    snapshot.tof.ranges_m = {0.5, 0.6, 0.7};
    snapshot.tof.angle_increment_rad = 0.1;
    snapshot.tof.max_range_m = 3.0;
    snapshot.tof.frame_id = "front_tof";
    snapshot.has_imu = true;
    snapshot.imu.timestamp_s = now_s;
    snapshot.imu.yaw_rate_rad_s = 0.0;
    snapshot.imu.az_mps2 = 9.8;
    return snapshot;
}

robot_slamd::RobotSlamMapQuality valid_quality() {
    robot_slamd::RobotSlamMapQuality quality;
    quality.good_enough = false;
    quality.coverage_ratio = 0.2;
    quality.yaw_coverage_ratio = 0.3;
    quality.valid_scan_count = 3;
    quality.reason = "map_quality_poor";
    return quality;
}
} // namespace

int main() {
    using namespace robot_slamd;

    auto sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{valid_snapshot(100.0)});
    auto motion = std::make_shared<FakeRobotSlamMotionPort>();
    auto map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{valid_quality()});
    auto time = std::make_shared<FixedStepTimePort>(100.0, 0.1);
    RealAdapterAcceptanceRunner runner(sensor, motion, map, time);
    auto result = runner.run_once();
    expect(result.ok, "valid replay acceptance passes");
    expect(result.passed.size() == 3, "all three cases pass");
    expect(motion->sent_commands.empty(), "runner does not send motion");
    expect(result.readiness_report.readiness == RealAdapterReadiness::ShadowReady, "runner only shadow ready");

    sensor->set_ready(false);
    auto not_ready = runner.run_once();
    expect(!not_ready.ok, "sensor not ready fails");
    expect(!not_ready.failed.empty() && not_ready.failed.front() == "readiness_ports", "readiness_ports failed");

    auto invalid_sensor = valid_snapshot(100.0);
    invalid_sensor.tof.frame_id.clear();
    auto bad_sensor_port = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{invalid_sensor});
    RealAdapterAcceptanceRunner bad_sensor(bad_sensor_port, motion, map, time);
    auto bad_sensor_result = bad_sensor.run_once();
    expect(!bad_sensor_result.ok, "invalid sensor fails");
    expect(bad_sensor_result.failed.size() >= 1, "invalid sensor records failure");

    RobotSlamMapQuality invalid_quality = valid_quality();
    invalid_quality.coverage_ratio = 2.0;
    auto bad_map_port = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{invalid_quality});
    RealAdapterAcceptanceRunner bad_map(sensor, motion, bad_map_port, time);
    auto bad_map_result = bad_map.run_once();
    expect(!bad_map_result.ok, "invalid map fails");
    expect(bad_map_result.failed.back() == "map_quality_contract", "map quality contract failed");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
