#include "robot_slamd/simulation/world/fake_world_2d.hpp"

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
    FakeWorld2D world;
    expect(world.add_axis_aligned_room(-1.0, -1.0, 1.0, 1.0), "room added");
    auto front = world.raycast(0.0, 0.0, 0.0, 0.02, 12.0);
    expect(front.hit, "front wall hit");
    expect(std::fabs(front.distance_m - 1.0) < 1e-9, "front wall distance");
    auto left = world.raycast(0.0, 0.0, kSimPi * 0.5, 0.02, 12.0);
    expect(left.hit, "left wall hit");
    expect(std::fabs(left.distance_m - 1.0) < 1e-9, "left wall distance");
    auto back_range = world.raycast(0.9, 0.0, kSimPi, 0.02, 0.5);
    expect(!back_range.hit, "max range outside no hit");
    expect(world.add_segment(0.4, -0.2, 0.4, 0.2, 99), "obstacle segment");
    auto nearest = world.raycast(0.0, 0.0, 0.0, 0.02, 12.0);
    expect(nearest.hit && nearest.segment_id == 99, "nearest hit chosen");
    auto parallel = world.raycast(0.0, 0.5, 0.0, 0.02, 0.4);
    expect(!parallel.hit, "parallel or out-of-range no hit");
    auto endpoint = world.raycast(0.0, 0.0, std::atan2(1.0, 1.0), 0.02, 2.0);
    expect(endpoint.hit, "endpoint hit");
    expect(world.circle_collides(1.0, 0.0, 0.05), "circle collides with wall");
    expect(!world.circle_collides(0.0, 0.0, 0.05), "circle free inside room");
    return failures ? 1 : 0;
}
