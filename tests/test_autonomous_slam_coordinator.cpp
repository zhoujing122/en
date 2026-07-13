#include "robot_slamd/autonomy/autonomous_slam_coordinator.hpp"
#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"

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

robot_slamd::AutonomousSlamCoordinatorOptions enabled_options() {
    robot_slamd::AutonomousSlamCoordinatorOptions options;
    options.enabled = true;
    options.max_iterations = 50;
    return options;
}

robot_slamd::AutonomousSlamStepInput input(double now_s) {
    robot_slamd::AutonomousSlamStepInput in;
    in.now_s = now_s;
    return in;
}

class CountingSensorPort final : public robot_slamd::RobotSlamSensorPort {
public:
    bool ready() const override { return true; }
    robot_slamd::RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        read_count++;
        snapshot.tof.timestamp_s = now_s;
        snapshot.imu.timestamp_s = now_s;
        snapshot.wheel.timestamp_s = now_s;
        return snapshot;
    }
    std::string status() const override { return "counting_sensor"; }
    int read_count = 0;
    robot_slamd::RobotSlamSensorSnapshot snapshot;
};

robot_slamd::RobotSlamSensorSnapshot valid_legacy_snapshot() {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_tof = true;
    s.has_imu = true;
    s.has_wheel = true;
    s.tof.ranges_m = {0.5};
    s.tof.timestamp_s = 1.0;
    s.tof.source = "legacy_scalar_tof_projection:test";
    s.imu.timestamp_s = 1.0;
    s.wheel.timestamp_s = 1.0;
    s.wheel.valid = true;
    return s;
}
} // namespace

int main() {
    using namespace robot_slamd;

    {
        auto sensor = std::make_shared<FakeRobotSlamSensorPort>();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(1);
        AutonomousSlamCoordinator coordinator(sensor, motion, map);
        auto out = coordinator.step(input(1.0));
        expect(out.state == AutonomousSlamState::Idle, "disabled coordinator stays idle");
        expect(motion->sent_commands.empty(), "disabled coordinator sends nothing");
    }

    {
        auto sensor = std::make_shared<FakeRobotSlamSensorPort>();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(1);
        AutonomousSlamCoordinator coordinator(sensor, motion, map, enabled_options());
        auto in = input(10.0);
        in.start_requested = true;
        expect(coordinator.step(in).state == AutonomousSlamState::WaitingForSensors,
               "start enters waiting");
        expect(coordinator.step(input(10.1)).state == AutonomousSlamState::Initializing,
               "ready sensors enter initializing");
        expect(coordinator.step(input(10.2)).state == AutonomousSlamState::Mapping,
               "initialization enters mapping");
        auto out = coordinator.step(input(10.3));
        expect(out.state == AutonomousSlamState::Completed, "good map completes");
        expect(out.completed, "completed flag set");
        expect(!motion->sent_commands.empty() &&
                   motion->sent_commands.back().kind == AlgorithmMotionKind::Stop,
               "completed sends stop");
    }

    {
        auto sensor = std::make_shared<FakeRobotSlamSensorPort>();
        sensor->ready_flag = false;
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(5);
        AutonomousSlamCoordinator coordinator(sensor, motion, map, enabled_options());
        auto in = input(20.0);
        in.start_requested = true;
        coordinator.step(in);
        auto out = coordinator.step(input(20.1));
        expect(out.state == AutonomousSlamState::WaitingForSensors,
               "not ready sensors keep waiting");
    }

    {
        auto sensor = std::make_shared<FakeRobotSlamSensorPort>();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(5);
        AutonomousSlamCoordinator coordinator(sensor, motion, map, enabled_options());
        auto in = input(30.0);
        in.start_requested = true;
        coordinator.step(in);
        coordinator.step(input(30.1));
        coordinator.step(input(30.2));
        expect(coordinator.step(input(30.3)).state == AutonomousSlamState::NeedActiveScan,
               "poor map needs active scan");
        auto need = coordinator.step(input(30.4));
        expect(need.state == AutonomousSlamState::SendingMotionCommand,
               "policy queues motion command");
        expect(need.algorithm_command.kind == AlgorithmMotionKind::ActiveScanTurnLeft,
               "coordinator uses active scan left");
        auto sent = coordinator.step(input(30.5));
        expect(sent.state == AutonomousSlamState::WaitingMotionSettle,
               "accepted motion waits settle");
        expect(sent.command_sent, "accepted motion command sent");
        expect(coordinator.step(input(30.6)).state == AutonomousSlamState::IntegratingScan,
               "settled motion integrates scan");
        expect(coordinator.step(input(30.7)).state == AutonomousSlamState::Mapping,
               "integrating returns to mapping");
        for (const auto &command : motion->sent_commands) {
            expect(command.kind != AlgorithmMotionKind::Forward &&
                       command.kind != AlgorithmMotionKind::Backward,
                   "coordinator never generated translation");
        }
    }

    {
        auto sensor = std::make_shared<FakeRobotSlamSensorPort>();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(5);
        AutonomousSlamCoordinator coordinator(sensor, motion, map, enabled_options());
        auto in = input(40.0);
        in.start_requested = true;
        coordinator.step(in);
        coordinator.step(input(40.1));
        coordinator.step(input(40.2));
        coordinator.step(input(40.3));
        coordinator.step(input(40.4));
        motion->reject_next_command = true;
        auto rejected = coordinator.step(input(40.5));
        expect(rejected.state == AutonomousSlamState::Fault, "motion rejected faults");
        expect(rejected.fault == AutonomousSlamFault::MotionRejected,
               "motion rejected fault reason");
    }

    {
        auto sensor = std::make_shared<FakeRobotSlamSensorPort>();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(5);
        AutonomousSlamCoordinatorOptions options = enabled_options();
        options.max_iterations = 1;
        AutonomousSlamCoordinator coordinator(sensor, motion, map, options);
        auto in = input(50.0);
        in.start_requested = true;
        coordinator.step(in);
        auto out = coordinator.step(input(50.1));
        expect(out.state == AutonomousSlamState::Fault, "max iterations faults");
        expect(out.fault == AutonomousSlamFault::MapQualityStuck,
               "max iterations uses map quality stuck");
    }

    {
        auto sensor = std::make_shared<FakeRobotSlamSensorPort>();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(5);
        AutonomousSlamCoordinator coordinator(sensor, motion, map, enabled_options());
        auto in = input(60.0);
        in.start_requested = true;
        coordinator.step(in);
        auto stop = input(60.1);
        stop.stop_requested = true;
        auto out = coordinator.step(stop);
        expect(out.state == AutonomousSlamState::Idle, "stop returns idle");
        expect(!motion->sent_commands.empty() &&
                   motion->sent_commands.back().kind == AlgorithmMotionKind::Stop,
               "stop request sends stop");
    }



    {
        auto sensor = std::make_shared<CountingSensorPort>();
        sensor->snapshot = valid_legacy_snapshot();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(1);
        AutonomousSlamCoordinator coordinator(sensor, motion, map, enabled_options());
        auto start = input(70.0);
        start.start_requested = true;
        coordinator.step(start);
        coordinator.step(input(70.1));
        auto explicit_multi = input(70.2);
        explicit_multi.sensors.has_multi_tof = true;
        explicit_multi.sensors.multi_tof.has_front = true;
        auto out = coordinator.step(explicit_multi);
        expect(sensor->read_count == 0,
               "explicit multi-ToF input does not consume sensor port");
        expect(out.state == AutonomousSlamState::Fault,
               "multi-ToF without legacy projection fails initialization");
        expect(out.fault == AutonomousSlamFault::InitializationFailed,
               "multi-ToF only init failure explicit");
    }

    {
        auto sensor = std::make_shared<CountingSensorPort>();
        sensor->snapshot = valid_legacy_snapshot();
        auto motion = std::make_shared<FakeRobotSlamMotionPort>();
        auto map = std::make_shared<FakeRobotSlamMapPort>(1);
        AutonomousSlamCoordinator coordinator(sensor, motion, map, enabled_options());
        auto start = input(80.0);
        start.start_requested = true;
        coordinator.step(start);
        coordinator.step(input(80.1));
        auto out = coordinator.step(input(80.2));
        expect(sensor->read_count == 1,
               "empty input keeps existing sensor port read behavior");
        expect(out.state == AutonomousSlamState::Mapping,
               "empty input can initialize from sensor port");
    }

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
