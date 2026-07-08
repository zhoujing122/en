#include "robot_slamd/autonomy/adapters/replay_robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/adapters/replay_robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"
#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_gate.hpp"
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
    snapshot.imu.az_mps2 = 9.8;
    return snapshot;
}

robot_slamd::RobotSlamMapQuality quality(bool good) {
    robot_slamd::RobotSlamMapQuality q;
    q.good_enough = good;
    q.coverage_ratio = good ? 0.8 : 0.1;
    q.yaw_coverage_ratio = good ? 0.8 : 0.1;
    q.valid_scan_count = good ? 5 : 1;
    q.reason = good ? "map_quality_good" : "map_quality_poor";
    return q;
}

robot_slamd::PreLiveIntegrationOptions options() {
    robot_slamd::PreLiveIntegrationOptions o;
    o.enabled = true;
    o.max_iterations = 8;
    return o;
}

int main() {
    using namespace robot_slamd;

    auto sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{valid_snapshot(100.0)});
    auto motion = std::make_shared<FakeRobotSlamMotionPort>();
    auto map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{quality(true)});
    auto time = std::make_shared<FixedStepTimePort>(100.0, 0.1);

    PreLiveAutonomousSlamRunner sensor_null(nullptr, motion, map, time, {}, {}, {}, {}, options());
    auto report = sensor_null.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ReadinessFailed, "sensor null readiness failed");

    PreLiveAutonomousSlamRunner motion_null(sensor, nullptr, map, time, {}, {}, {}, {}, options());
    report = motion_null.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ReadinessFailed, "motion null readiness failed");

    PreLiveAutonomousSlamRunner map_null(sensor, motion, nullptr, time, {}, {}, {}, {}, options());
    report = map_null.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ReadinessFailed, "map null readiness failed");

    auto invalid_tof = valid_snapshot(100.0);
    invalid_tof.tof.ranges_m.clear();
    auto bad_sensor = std::make_shared<ReplayRobotSlamSensorPort>(std::vector<RobotSlamSensorSnapshot>{invalid_tof});
    PreLiveAutonomousSlamRunner invalid_tof_runner(bad_sensor, motion, map, time, {}, {}, {}, {}, options());
    report = invalid_tof_runner.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ContractFailed, "invalid tof contract failed");

    auto invalid_map_quality = quality(true);
    invalid_map_quality.yaw_coverage_ratio = -0.1;
    auto bad_map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{invalid_map_quality});
    PreLiveAutonomousSlamRunner invalid_map_runner(sensor, motion, bad_map, time, {}, {}, {}, {}, options());
    report = invalid_map_runner.run();
    expect(!report.ok && report.block_reason == PreLiveBlockReason::ContractFailed, "invalid map contract failed");

    auto reject_motion = std::make_shared<FakeRobotSlamMotionPort>();
    reject_motion->reject_next_command = true;
    auto poor_map = std::make_shared<ReplayRobotSlamMapPort>(std::vector<RobotSlamMapQuality>{quality(false)});
    auto reject_options = options();
    reject_options.require_adapter_acceptance = false;
    PreLiveAutonomousSlamRunner reject_runner(sensor, reject_motion, poor_map, time, {}, {}, {}, {}, reject_options);
    report = reject_runner.run();
    expect(!report.ok, "motion reject blocked or faulted");

    auto low_iter_options = options();
    low_iter_options.max_iterations = 1;
    PreLiveAutonomousSlamRunner low_iter_runner(sensor, motion, poor_map, time, {}, {}, {}, {}, low_iter_options);
    report = low_iter_runner.run();
    expect(!report.ok, "max iteration low blocks safely");

    PreLiveIntegrationReport manual_report;
    manual_report.ok = true;
    manual_report.counters.adapter_acceptance_run_count = 1;
    manual_report.final_state = AutonomousSlamState::Mapping;
    PreLiveAutonomousSlamGate gate;
    auto gate_result = gate.evaluate(manual_report);
    expect(!gate_result.pass && contains(gate_result.failed, "stop_command_not_seen"), "missing stop blocked");

    manual_report.counters.stop_command_count = 1;
    AlgorithmMotionCommand forward;
    forward.kind = AlgorithmMotionKind::Forward;
    forward.reason = "forbidden";
    manual_report.algorithm_commands.push_back(forward);
    gate_result = gate.evaluate(manual_report);
    expect(!gate_result.pass && contains(gate_result.failed, "forward_backward_command_forbidden"), "forward forbidden blocked");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
