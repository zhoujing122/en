#include "robot_slamd/exploration/safe_waypoint_tracker.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"
#include "sparse_navigation_test_helpers.hpp"

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

    auto obstacle_plant = std::make_shared<SimRobotPlant>();
    expect(obstacle_plant->reset({}, now), "obstacle plant reset");
    SimMotionPort obstacle_motion(obstacle_plant, motion_config);
    SafeWaypointTrackerConfig obstacle_config;
    obstacle_config.obstacle_confirmation_frames = 2;
    SafeWaypointTracker obstacle_tracker(obstacle_config);
    expect(obstacle_tracker.set_path({RobotPose2D{},
                                      RobotPose2D{1.0, 0.0, 0.0}},
                                     RobotPose2D{}),
           "obstacle path accepted");
    auto obstacle_step = [&](bool hit, double distance_m) {
        obstacle_motion.update(now);
        SafeWaypointTrackerInput input;
        input.now_s = now;
        input.estimated_map_pose = obstacle_plant->state().pose;
        input.motion_feedback = obstacle_motion.latest_feedback(now);
        input.front_hit = hit;
        input.front_distance_m = distance_m;
        const auto result = obstacle_tracker.step(input, obstacle_motion);
        now += 0.05;
        return result;
    };
    expect(obstacle_step(false, 0.0).command.kind == AlgorithmMotionKind::Stop,
           "path transition first sends stop");
    bool forward_started = false;
    for (int i = 0; i < 10 && !forward_started; ++i) {
        const auto step = obstacle_step(false, 0.0);
        forward_started = step.command_sent &&
            step.command.kind == AlgorithmMotionKind::Forward;
    }
    expect(forward_started, "aligned path starts forward command");
    const auto first_hit = obstacle_step(true, 0.18);
    expect(first_hit.ok && !first_hit.path_validation_required,
           "single normal hit does not abandon path");
    const auto confirmed = obstacle_step(true, 0.18);
    expect(confirmed.ok && confirmed.command_sent &&
               confirmed.path_validation_required &&
               !confirmed.replan_required &&
               confirmed.command.kind == AlgorithmMotionKind::Stop,
           "confirmed normal hit stops for map path validation");

    SafeWaypointTracker turning_tracker(obstacle_config);
    expect(turning_tracker.set_path(
               {RobotPose2D{}, RobotPose2D{0.0, 1.0, 0.0}}, RobotPose2D{}),
           "turning path accepted");
    obstacle_motion.update(now);
    SafeWaypointTrackerInput turn_input;
    turn_input.now_s = now;
    turn_input.estimated_map_pose = {};
    turn_input.motion_feedback = obstacle_motion.latest_feedback(now);
    turn_input.front_hit = true;
    turn_input.front_distance_m = 0.18;
    const auto turning = turning_tracker.step(turn_input, obstacle_motion);
    expect(turning.ok && !turning.path_validation_required,
           "front obstacle does not reject a lateral goal while turning");

    SafeWaypointTrackerConfig stuck_config;
    stuck_config.forward_segment_duration_s = 0.05;
    stuck_config.maximum_segments_without_progress = 2;
    stuck_config.maximum_no_progress_duration_s = 10.0;
    SafeWaypointTracker stuck_tracker(stuck_config);
    auto stuck_plant = std::make_shared<SimRobotPlant>();
    expect(stuck_plant->reset({}, now), "stuck plant reset");
    SimMotionPort stuck_motion(stuck_plant, motion_config);
    expect(stuck_tracker.set_path(
               {RobotPose2D{}, RobotPose2D{1.0, 0.0, 0.0}}, RobotPose2D{}),
           "stuck path accepted");
    bool stuck_replan = false;
    for (int i = 0; i < 80 && !stuck_replan; ++i) {
        stuck_motion.update(now);
        SafeWaypointTrackerInput input;
        input.now_s = now;
        input.estimated_map_pose = {};
        input.motion_feedback = stuck_motion.latest_feedback(now);
        const auto step = stuck_tracker.step(input, stuck_motion);
        expect(step.ok, "stuck tracker remains valid");
        stuck_replan = step.replan_required;
        now += 0.05;
    }
    expect(stuck_replan && stuck_tracker.report().no_progress_replans == 1,
           "estimated-pose no progress stops and replans");

    NavigationGridViewConfig grid_config;
    grid_config.min_x_m = 0.0;
    grid_config.max_x_m = 5.0;
    grid_config.min_y_m = 0.0;
    grid_config.max_y_m = 3.0;
    grid_config.robot_radius_m = 0.0;
    grid_config.safety_margin_m = 0.0;
    std::vector<SparseOccupancyCell> path_cells;
    for (int x = 0; x < 5; ++x) {
        path_cells.push_back(x == 2
            ? sparse_navigation_test::occupied_cell(x, 1)
            : sparse_navigation_test::free_cell(x, 1));
    }
    NavigationGridView blocked_grid;
    expect(blocked_grid.build(
               sparse_navigation_test::snapshot_from_cells(path_cells),
               grid_config).ok,
           "blocked path grid builds");
    SafeWaypointTracker path_tracker;
    expect(path_tracker.set_path({RobotPose2D{0.5, 1.5, 0.0},
                                  RobotPose2D{4.5, 1.5, 0.0}},
                                 RobotPose2D{0.5, 1.5, 0.0}),
           "path validation tracker accepts route");
    expect(!path_tracker.remaining_path_is_traversable(
               blocked_grid, RobotPose2D{0.5, 1.5, 0.0}),
           "latest occupied cell invalidates remaining path");
    return 0;
}
