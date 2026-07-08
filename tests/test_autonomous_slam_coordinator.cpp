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

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
