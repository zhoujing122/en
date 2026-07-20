#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"

#include <set>

int main() {
    const auto report = robot_slamd::StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_rearm_progress", "nominal_rectangle");
    const std::set<std::uint64_t> command_ids(
        report.command_ids.begin(), report.command_ids.end());
    return report.ok && report.corner_rearm_count == 4 &&
           report.wall_segment_sequence == std::vector<std::uint64_t>{1, 2, 3, 4, 5} &&
           report.forward_steps_per_segment.size() == 5 &&
           report.odom_distance_per_segment_m.size() == 5 &&
           command_ids.size() == report.command_ids.size() &&
           report.corner_transition_ids.size() == 4 ? 0 : 1;
}
