#include "robot_slamd/replay/stop_go_replay_runner.hpp"
#include "robot_slamd/simulation/application/rectangle_mapping_comparison.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"

#include <filesystem>
#include <iostream>

int main() {
    using namespace robot_slamd;
    const std::filesystem::path root = "/tmp/p6_replay_comparison";
    const auto source = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), (root / "source").string(), "nominal_rectangle");
    const auto first = StopGoReplayRunner{}.run(
        p6_rectangle_test_config(), (root / "source" / "stop_go_mapping.replay").string(),
        (root / "one").string());
    const auto second = StopGoReplayRunner{}.run(
        p6_rectangle_test_config(), (root / "source" / "stop_go_mapping.replay").string(),
        (root / "two").string());
    const auto comparison = run_rectangle_mapping_comparison(
        p6_rectangle_test_config(), source, (root / "comparison").string());
    std::cout << "comparison stop=" << comparison.stop_go_termination
              << " frontier=" << comparison.frontier_termination
              << " frontier_steps=" << comparison.frontier_simulation_steps
              << " frontier_revision=" << comparison.frontier_map_revision
              << " frontier_cells=" << comparison.frontier_map_cells
              << " frontier_checksum=" << comparison.frontier_map_checksum << '\n';
    return source.ok && first.ok && second.ok &&
           first.final_map_checksum == source.final_map_checksum &&
           first.final_map_checksum == second.final_map_checksum &&
           first.wall_segment_sequence == second.wall_segment_sequence &&
           first.corner_transition_count == second.corner_transition_count &&
           first.termination_reason == second.termination_reason &&
           first.run_start_anchor.valid && second.run_start_anchor.valid &&
           first.run_start_anchor.initial_wall_model_signature_hash ==
               second.run_start_anchor.initial_wall_model_signature_hash &&
           first.total_odom_travel_distance_m == source.total_odom_travel_distance_m &&
           first.closure_confirmation_attempt_count ==
               source.closure_confirmation_attempt_count &&
           first.closure_confirmation_pass_count ==
               source.closure_confirmation_pass_count &&
           comparison.stop_go_completed && !comparison.frontier_termination.empty() &&
           comparison.stop_go_map_commits == source.map_commits &&
           comparison.frontier_map_commits == comparison.frontier_map_revision &&
           comparison.stop_go_map_checksum == source.final_map_checksum ? 0 : 1;
}
