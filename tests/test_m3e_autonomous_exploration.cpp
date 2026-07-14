#include "m3e_exploration_test_helpers.hpp"

int main() {
    const auto report = m3e_test::run("/tmp/en_m3e_test_full");
    m3e_test::expect(report.ok, "formal exploration runner completes");
    m3e_test::expect(
        report.exploration.state ==
            robot_slamd::AutonomousExplorationState::Completed &&
            report.termination_reason == "no_reachable_frontier",
        "completion requires no reachable frontier");
    m3e_test::expect(report.exploration.selected_goals > 0 &&
                         report.exploration.completed_goals > 0,
                     "at least one reachable frontier completes");
    m3e_test::expect(report.exploration.expanded_astar_nodes > 0 &&
                         report.exploration.plan_count > 0 &&
                         report.exploration.replan_count > 0,
                     "bounded A-star and replanning execute");
    m3e_test::expect(report.ground_truth_travel_distance_m > 0.30 &&
                         report.exploration.estimated_travel_distance_m > 0.30 &&
                         report.exploration.commanded_forward_segments > 0,
                     "motion port drives plant while tracking estimated pose");
    m3e_test::expect(report.exploration.matcher_attempts > 0 &&
                         report.exploration.atomic_commit_successes > 0 &&
                         report.exploration.keyframe_count > 0,
                     "yaw matcher and atomic keyframe path execute");
    m3e_test::expect(report.exploration.known_cells_end >
                             report.exploration.known_cells_start &&
                         report.exploration.map_revision_end >
                             report.exploration.map_revision_start,
                     "continued sparse slam grows map and revision");
    m3e_test::expect(report.exploration.obstacle_stops > 0 &&
                         !report.collision,
                     "front obstacle causes safe stop and replan");
    m3e_test::expect(report.final_map_save_success &&
                         report.final_map_checksum != 0,
                     "completed exploration saves validated map artifact");
    return 0;
}
