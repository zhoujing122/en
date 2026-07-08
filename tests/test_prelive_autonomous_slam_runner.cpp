#include "robot_slamd/autonomy/adapters/replay_robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/adapters/replay_robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"
#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_runner.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool contains(const std::vector<std::string> &values, const std::string &needle) {
    for (const auto &value : values) {
        if (value == needle) {
            return true;
        }
    }
    return false;
}
} // namespace

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

robot_slamd::RobotSlamMapQuality good_quality() {
    robot_slamd::RobotSlamMapQuality quality;
    quality.good_enough = true;
    quality.coverage_ratio = 0.8;
    quality.yaw_coverage_ratio = 0.8;
    quality.valid_scan_count = 5;
    quality.reason = "map_quality_good";
    return quality;
}

robot_slamd::RobotSlamMapQuality poor_quality() {
    robot_slamd::RobotSlamMapQuality quality;
    quality.good_enough = false;
    quality.coverage_ratio = 0.1;
    quality.yaw_coverage_ratio = 0.1;
    quality.valid_scan_count = 1;
    quality.reason = "map_quality_poor";
    return quality;
}

robot_slamd::PreLiveIntegrationOptions enabled_options() {
    robot_slamd::PreLiveIntegrationOptions options;
    options.enabled = true;
    options.max_iterations = 12;
    return options;
}

int main() {
    using namespace robot_slamd;

    auto disabled_sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{valid_snapshot(100.0)});
    auto disabled_motion = std::make_shared<FakeRobotSlamMotionPort>();
    auto disabled_map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{good_quality()});
    auto disabled_time = std::make_shared<FixedStepTimePort>(100.0, 0.1);
    PreLiveAutonomousSlamRunner disabled_runner(disabled_sensor, disabled_motion, disabled_map, disabled_time);
    auto report = disabled_runner.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ConfigDisabled, "disabled blocks");

    auto sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{valid_snapshot(100.0)});
    auto motion = std::make_shared<FakeRobotSlamMotionPort>();
    auto map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{good_quality()});
    auto time = std::make_shared<FixedStepTimePort>(100.0, 0.1);
    PreLiveAutonomousSlamRunner runner(sensor, motion, map, time, {}, {}, {}, {}, enabled_options());
    report = runner.run();
    expect(report.ok, "valid prelive run passes");
    expect(report.stage == PreLiveIntegrationStage::Passed, "valid run passed stage");
    expect(report.counters.adapter_acceptance_run_count == 1, "acceptance run counted");
    expect(report.counters.stop_command_count > 0, "stop command recorded");
    expect(!report.algorithm_commands.empty(), "algorithm commands recorded");
    expect(!report.software_commands.empty(), "software commands recorded");
    expect(report.summary == "prelive_autonomous_slam_passed", "passed summary");

    auto not_ready_sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{valid_snapshot(100.0)}, false);
    PreLiveAutonomousSlamRunner readiness_runner(not_ready_sensor, motion, map, time, {}, {}, {}, {}, enabled_options());
    report = readiness_runner.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ReadinessFailed, "readiness failure blocks");

    auto invalid_snapshot = valid_snapshot(100.0);
    invalid_snapshot.tof.frame_id.clear();
    auto bad_sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{invalid_snapshot});
    PreLiveAutonomousSlamRunner contract_runner(bad_sensor, motion, map, time, {}, {}, {}, {}, enabled_options());
    report = contract_runner.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ContractFailed, "contract failure blocks");

    RobotSlamMapQuality bad_quality = good_quality();
    bad_quality.coverage_ratio = 2.0;
    auto bad_map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{bad_quality});
    PreLiveAutonomousSlamRunner bad_map_runner(sensor, motion, bad_map, time, {}, {}, {}, {}, enabled_options());
    report = bad_map_runner.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ContractFailed, "map contract failure blocks");

    auto no_accept_sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{valid_snapshot(100.0)}, false);
    PreLiveIntegrationOptions options = enabled_options();
    options.require_adapter_acceptance = true;
    PreLiveAutonomousSlamRunner acceptance_runner(no_accept_sensor, motion, map, time, {}, {}, {}, {}, options);
    report = acceptance_runner.run();
    expect(!report.ok, "acceptance blocked by readiness before run");

    auto reject_motion = std::make_shared<FakeRobotSlamMotionPort>();
    reject_motion->reject_next_command = true;
    auto poor_map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{poor_quality()});
    options = enabled_options();
    options.require_adapter_acceptance = false;
    PreLiveAutonomousSlamRunner reject_runner(sensor, reject_motion, poor_map, time, {}, {}, {}, {}, options);
    report = reject_runner.run();
    expect(!report.ok && report.block_reason != PreLiveBlockReason::None, "motion reject blocks safely");
    expect(report.counters.motion_reject_count > 0 || report.counters.coordinator_fault_count > 0, "motion reject counted");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
