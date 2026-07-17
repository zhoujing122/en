#include "robot_slamd/app/robot_slam_motion_boundary.hpp"
#include "robot_slamd/simulation/application/simulation_robot_slam_runner.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

namespace {
int failures = 0;
void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
robot_slamd::AlgorithmMotionCommand make_command(
    robot_slamd::AlgorithmMotionKind kind, double timestamp, std::uint64_t sequence) {
    robot_slamd::AlgorithmMotionCommand command;
    command.kind = kind;
    command.profile = robot_slamd::AlgorithmMotionProfile::ManualTest;
    command.speed_normalized = robot_slamd::is_algorithm_stop_kind(kind) ? 0.0 : 0.10;
    command.duration_s = robot_slamd::is_algorithm_stop_kind(kind) ? 0.0 : 0.10;
    command.timestamp_s = timestamp;
    command.ttl_s = 0.30;
    command.reason = robot_slamd::is_algorithm_stop_kind(kind)
        ? "motion_boundary_test_stop" : "motion_boundary_test_command";
    command.sequence = sequence;
    return command;
}
class RejectingMotionPort final : public robot_slamd::RobotSlamMotionPort {
public:
    bool ready() const override { return true; }
    robot_slamd::AlgorithmMotionFacadeResult send_algorithm_command(
        const robot_slamd::AlgorithmMotionCommand &command) override {
        calls++;
        if (robot_slamd::is_algorithm_stop_kind(command.kind)) stops++;
        robot_slamd::AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;
        result.error = "transport_rejected";
        return result;
    }
    robot_slamd::RobotSlamMotionFeedback latest_feedback(double now) override {
        robot_slamd::RobotSlamMotionFeedback result;
        result.command_settled = true;
        result.timestamp_s = now;
        return result;
    }
    std::string status() const override { return "rejecting_test_transport"; }
    int calls = 0;
    int stops = 0;
};
robot_slamd::RobotSlamSensorSnapshot safe_snapshot() {
    robot_slamd::RobotSlamSensorSnapshot snapshot;
    snapshot.has_wheel = true;
    snapshot.wheel.valid = true;
    snapshot.wheel.timestamp_s = 0.0;
    snapshot.has_multi_tof = true;
    snapshot.multi_tof.has_front = true;
    snapshot.multi_tof.front.protocol_valid = true;
    snapshot.multi_tof.front.usable_for_mapping = true;
    snapshot.multi_tof.front.distance_m = 1.0;
    snapshot.multi_tof.front.effective_timestamp_s = 0.0;
    return snapshot;
}
}

int main() {
    using namespace robot_slamd;
    Config config;
    config.runtime_sensor_source = "simulation";
    config.runtime_operation = "exploration";
    config.sparse_slam_planar_tof_extrinsics_configured = true;
    config.sparse_slam_front_tof_x_m = 0.04;
    config.sparse_slam_left_tof_y_m = 0.04;
    config.sparse_slam_right_tof_y_m = -0.04;
    config.sparse_slam_left_tof_yaw_rad = kPi / 2.0;
    config.sparse_slam_right_tof_yaw_rad = -kPi / 2.0;
    config.exploration_simulation_fixed_dt_s = 0.05;
    config.exploration_simulation_initial_x_m = -0.75;
    config.exploration_simulation_initial_y_m = 0.0;
    config.exploration_simulation_initial_yaw_rad = 0.0;
    config.motion_execution_command_stale_timeout_s = 0.50;
    config.motion_execution_deadman_timeout_s = 0.30;
    config.motion_execution_max_encoder_age_s = 0.50;
    validate_config(config);

    auto simulation = build_simulation_robot_slam_adapter(config, OperationMode::Exploration);
    expect(simulation.ok && simulation.application && simulation.motion,
           "simulation composition exposes Application and endpoint");
    if (simulation.ok && simulation.application) {
        auto &application = *simulation.application;
        expect(application.initialize().ok, "simulation Application initializes");
        const auto send = [&](AlgorithmMotionKind kind, std::uint64_t sequence) {
            if (sequence == 1) {
                expect(simulation.plant->step(0.05, simulation.clock->now_s() + 0.05,
                                              simulation.world.get()),
                       "simulation advances before first canonical sample");
                expect(simulation.clock->advance(0.05), "simulation clock starts");
            }
            const double now = simulation.clock->now_s();
            expect(application.step(now).core_step_executed,
                   "Application/Core remains the only step path");
            const auto sent = application.motion_adapter()->send_algorithm_command(
                make_command(kind, now, sequence));
            expect(sent.ok && sent.accepted, "simulation command passes Application boundary");
            const auto stopped = application.motion_adapter()->send_algorithm_command(
                make_command(AlgorithmMotionKind::Stop, now, sequence + 100));
            expect(stopped.ok && stopped.accepted, "simulation Stop passes safety/write");
            expect(simulation.plant->step(0.05, now + 0.05, simulation.world.get()),
                   "plant advances after transport dispatch");
            expect(simulation.clock->advance(0.05), "simulation clock advances");
        };
        send(AlgorithmMotionKind::TurnLeft, 1);
        send(AlgorithmMotionKind::TurnRight, 2);
        send(AlgorithmMotionKind::Forward, 3);
        expect(simulation.motion->turn_left_count() > 0, "TurnLeft reaches SimMotionPort");
        expect(simulation.motion->turn_right_count() > 0, "TurnRight reaches SimMotionPort");
        expect(simulation.motion->forward_count() > 0, "Forward reaches SimMotionPort");
        const double now = simulation.clock->now_s();
        expect(application.step(now).core_step_executed, "stale test refreshes context");
        const auto stale = application.motion_adapter()->send_algorithm_command(
            make_command(AlgorithmMotionKind::Forward, now - 1.0, 20));
        expect(!stale.accepted && stale.error == "command_stale",
               "stale command is rejected before transport");
        expect(!application.report().motion.command_speed_used_as_odometry,
               "command speed is not an odometry input");
    }

    RobotSlamMotionBoundary replay(config, SensorSource::Replay, OperationMode::Mapping, nullptr);
    const auto replay_motion = replay.send_algorithm_command(
        make_command(AlgorithmMotionKind::Forward, 0.0, 1));
    expect(!replay_motion.accepted && replay_motion.error == "replay_motion_disabled",
           "Replay motion is explicit no-motion fail-closed");
    const auto replay_stop = replay.send_algorithm_command(
        make_command(AlgorithmMotionKind::Stop, 0.0, 2));
    expect(replay_stop.ok && replay_stop.accepted, "Replay Stop uses no-motion transport");

    RobotSlamMotionBoundary real(config, SensorSource::Real, OperationMode::Exploration, nullptr);
    const auto real_motion = real.send_algorithm_command(
        make_command(AlgorithmMotionKind::Forward, 0.0, 1));
    expect(!real.ready() && !real_motion.accepted &&
               real_motion.error == "real_motion_unavailable_fail_closed",
           "Real motion is fail-closed without hardware transport");

    auto rejecting = std::make_shared<RejectingMotionPort>();
    RobotSlamMotionBoundary boundary(config, SensorSource::Simulation,
                                     OperationMode::Exploration, rejecting);
    SparseSlamRuntimeCore core(config);
    RobotSlamMotionFeedback feedback;
    feedback.command_settled = true;
    boundary.update_context(safe_snapshot(), 0.0, feedback, core);
    const auto rejected = boundary.send_algorithm_command(
        make_command(AlgorithmMotionKind::TurnLeft, 0.0, 5));
    expect(!rejected.accepted && boundary.report().transport_reject_count > 0 &&
               rejecting->stops > 0,
           "transport rejection causes Stop through the same boundary");
    expect(failures == 0, "unified motion boundary acceptance");
    return failures == 0 ? 0 : 1;
}
