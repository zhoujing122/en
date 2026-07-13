#include "robot_slamd/simulation/robot/sim_robot_plant.hpp"

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
    SimRobotPlant plant;
    RobotPose2D pose;
    expect(plant.reset(pose, 0.0), "reset valid");
    expect(plant.step(0.02, 0.02), "zero step valid");
    expect(std::fabs(plant.state().pose.x_m) < 1e-12, "no command x static");
    expect(std::fabs(plant.state().left_wheel_position_rad) < 1e-12, "no command wheel static");

    expect(plant.set_target_velocity(0.10, 0.0), "set forward target");
    for (int i = 0; i < 100; ++i) {
        expect(plant.step(0.02, 0.04 + i * 0.02), "forward step");
    }
    expect(plant.state().pose.x_m > 0.05, "forward changes x");
    expect(plant.state().left_wheel_position_rad > 0.0, "forward left wheel positive");
    expect(plant.state().right_wheel_position_rad > 0.0, "forward right wheel positive");
    const double forward_left = plant.state().left_wheel_position_rad;

    plant.reset(pose, 0.0);
    expect(plant.set_target_velocity(0.0, 0.8), "set left turn target");
    for (int i = 0; i < 100; ++i) {
        expect(plant.step(0.02, i * 0.02), "turn step");
    }
    expect(plant.state().pose.yaw_rad > 0.1, "left turn yaw increases");
    expect(plant.state().left_wheel_position_rad < 0.0, "left turn left wheel negative");
    expect(plant.state().right_wheel_position_rad > 0.0, "left turn right wheel positive");

    plant.reset(pose, 0.0);
    expect(plant.set_target_velocity(0.0, -0.8), "set right turn target");
    for (int i = 0; i < 100; ++i) {
        expect(plant.step(0.02, i * 0.02), "right step");
    }
    expect(plant.state().pose.yaw_rad < -0.1, "right turn yaw decreases");

    plant.reset(pose, 0.0);
    expect(plant.set_target_velocity(-0.10, 0.0), "set backward target");
    for (int i = 0; i < 100; ++i) {
        expect(plant.step(0.02, i * 0.02), "backward step");
    }
    expect(plant.state().pose.x_m < -0.05, "backward changes x negative");
    expect(plant.state().left_wheel_position_rad < 0.0, "backward wheel sign");
    expect(std::fabs(plant.state().left_wheel_position_rad) > 0.1 * std::fabs(forward_left),
           "backward wheel changed");

    SimRobotPlantConfig bad;
    bad.wheel_base_m = 0.0;
    SimRobotPlant invalid(bad);
    expect(!invalid.valid(), "invalid wheel base rejected");
    expect(!plant.step(0.0, 1.0), "invalid dt rejected");

    FakeWorld2D world;
    world.add_segment(0.20, -1.0, 0.20, 1.0, 10);
    SimRobotPlantConfig colliding_config;
    colliding_config.collision_check_enabled = true;
    colliding_config.robot_radius_m = 0.05;
    SimRobotPlant collision_plant(colliding_config);
    collision_plant.reset(pose, 0.0);
    collision_plant.set_target_velocity(0.20, 0.0);
    for (int i = 0; i < 200; ++i) {
        collision_plant.step(0.01, i * 0.01, &world);
    }
    expect(collision_plant.state().pose.x_m < 0.20, "collision prevents crossing wall");
    expect(collision_plant.state().collision, "collision reported");
    return failures ? 1 : 0;
}
