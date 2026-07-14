#include "robot_slamd/exploration/safe_waypoint_tracker.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>

namespace {
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << "\n";
        std::exit(1);
    }
}
}

int main() {
    using namespace robot_slamd;
    auto plant = std::make_shared<SimRobotPlant>();
    expect(plant->reset({}, 0.0), "plant reset");
    SimMotionPortConfig motion_config;
    motion_config.allow_translation = true;
    SimMotionPort motion(plant, motion_config);
    SafeWaypointTracker tracker;
    expect(tracker.set_path({RobotPose2D{}, RobotPose2D{0.0, 0.35, 0.0}},
                            RobotPose2D{}),
           "tracker accepts path");

    bool reached = false;
    double now = 0.0;
    for (int i = 0; i < 600 && !reached; ++i) {
        motion.update(now);
        SafeWaypointTrackerInput input;
        input.now_s = now;
        input.estimated_map_pose = plant->state().pose;
        input.motion_feedback = motion.latest_feedback(now);
        const auto result = tracker.step(input, motion);
        expect(result.ok, "tracker step succeeds");
        reached = result.goal_reached;
        expect(plant->step(0.05, now + 0.05), "plant physics step");
        now += 0.05;
    }
    expect(reached, "estimated-pose tracking reaches waypoint");
    expect(motion.turn_left_count() > 0 && motion.forward_count() > 0,
           "tracker maps alignment and following to motion port");
    expect(tracker.report().commanded_stops >= 2,
           "state changes issue stop first");
    expect(tracker.report().estimated_travel_distance_m > 0.2,
           "estimated travel accumulated");

    SafeWaypointTracker obstacle_tracker;
    expect(obstacle_tracker.set_path({plant->state().pose,
                                      RobotPose2D{1.0, 0.35, 0.0}},
                                     plant->state().pose),
           "obstacle path accepted");
    motion.update(now);
    SafeWaypointTrackerInput blocked;
    blocked.now_s = now;
    blocked.estimated_map_pose = plant->state().pose;
    blocked.motion_feedback = motion.latest_feedback(now);
    blocked.front_hit = true;
    blocked.front_distance_m = 0.10;
    const auto stopped = obstacle_tracker.step(blocked, motion);
    expect(stopped.ok && stopped.command_sent && stopped.replan_required,
           "front obstacle causes stop and replan");
    expect(stopped.command.kind == AlgorithmMotionKind::Stop &&
           obstacle_tracker.report().obstacle_stops == 1,
           "obstacle stop is explicit");
    return 0;
}
