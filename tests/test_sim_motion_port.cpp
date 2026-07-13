#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    auto plant = std::make_shared<SimRobotPlant>();
    RobotPose2D pose;
    plant->reset(pose, 100.0);
    SimMotionPortConfig config;
    config.allow_translation = false;
    SimMotionPort motion(plant, config);
    AlgorithmMotionCommandBuilder builder;

    motion.update(100.0);
    auto left = motion.send_algorithm_command(builder.active_scan_turn_left(100.0));
    expect(left.ok && left.accepted, "active scan accepted");
    expect(motion.command_active(), "command active after send");
    expect(!motion.command_settled(), "not settled immediately");
    for (int i = 0; i < 10; ++i) {
        const double now = 100.0 + i * 0.02;
        motion.update(now);
        plant->step(0.02, now + 0.02);
    }
    expect(plant->state().pose.yaw_rad > 0.0, "left turn changes yaw");
    expect(plant->state().left_wheel_position_rad < 0.0, "left turn wheel sign");
    expect(plant->state().right_wheel_position_rad > 0.0, "left turn wheel sign right");

    motion.update(101.0);
    auto stop = motion.send_algorithm_command(builder.stop(101.0, "test_stop"));
    expect(stop.ok && stop.accepted, "stop accepted");
    for (int i = 0; i < 100; ++i) {
        const double now = 101.0 + i * 0.02;
        motion.update(now);
        plant->step(0.02, now + 0.02);
    }
    expect(!motion.command_active(), "stop clears active");
    expect(motion.command_settled(), "stop eventually settles");

    auto forward_rejected = motion.send_algorithm_command(builder.manual_forward(103.5));
    expect(!forward_rejected.ok && !forward_rejected.accepted,
           "forward rejected when simulation translation disabled");

    SimMotionPortConfig translation;
    translation.allow_translation = true;
    auto plant2 = std::make_shared<SimRobotPlant>();
    plant2->reset(pose, 0.0);
    SimMotionPort sim_translation(plant2, translation);
    sim_translation.update(0.0);
    auto forward = sim_translation.send_algorithm_command(builder.manual_forward(0.0));
    expect(forward.ok && forward.accepted, "forward accepted when simulation enabled");
    for (int i = 0; i < 100; ++i) {
        sim_translation.update(i * 0.02);
        plant2->step(0.02, i * 0.02 + 0.02);
    }
    expect(plant2->state().pose.x_m > 0.0, "simulation forward changes x");

    auto plant_backward = std::make_shared<SimRobotPlant>();
    plant_backward->reset(pose, 0.0);
    SimMotionPort sim_backward(plant_backward, translation);
    sim_backward.update(0.0);
    auto backward = sim_backward.send_algorithm_command(builder.manual_backward(0.0));
    expect(backward.ok && backward.accepted, "backward accepted when simulation enabled");
    for (int i = 0; i < 100; ++i) {
        sim_backward.update(i * 0.02);
        plant_backward->step(0.02, i * 0.02 + 0.02);
    }
    expect(plant_backward->state().pose.x_m < 0.0, "simulation backward changes x negative");

    auto plant3 = std::make_shared<SimRobotPlant>();
    plant3->reset(pose, 0.0);
    SimMotionPort emergency(plant3, translation);
    emergency.update(0.0);
    expect(emergency.send_algorithm_command(builder.manual_forward(0.0)).accepted,
           "emergency setup forward");
    emergency.update(0.1);
    auto estop = emergency.send_algorithm_command(builder.emergency_stop(0.1));
    expect(estop.ok && estop.accepted, "emergency stop accepted");
    auto after_estop = emergency.send_algorithm_command(builder.manual_forward(0.2));
    expect(!after_estop.accepted, "ordinary command blocked after emergency stop");

    auto stale = emergency.send_algorithm_command(builder.active_scan_turn_left(-10.0));
    expect(!stale.accepted, "stale command rejected");
    return failures ? 1 : 0;
}
