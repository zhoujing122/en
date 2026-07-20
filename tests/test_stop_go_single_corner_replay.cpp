#include "robot_slamd/replay/stop_go_replay_runner.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

#include <filesystem>

int main() {
    using namespace robot_slamd;
    const std::string root = "/tmp/p5_single_corner_replay";
    std::filesystem::create_directories(root + "/source");
    std::filesystem::create_directories(root + "/one");
    std::filesystem::create_directories(root + "/two");
    const auto config = p5_single_corner_test_config();
    const auto source = StopGoStraightWallSimulationRunner{}.run_scenario(
        config, root + "/source", "nominal_single_corner");
    if (!source.ok || source.replay_records.empty()) return 1;
    const auto first = StopGoReplayRunner{}.run(config,
        root + "/source/stop_go_mapping.replay", root + "/one");
    const auto second = StopGoReplayRunner{}.run(config,
        root + "/source/stop_go_mapping.replay", root + "/two");
    return first.ok && second.ok && first.termination_reason == "single_corner_complete" &&
           first.final_map_checksum == source.final_map_checksum &&
           first.final_map_checksum == second.final_map_checksum &&
           first.command_ids == source.command_ids &&
           second.command_ids == source.command_ids &&
           first.wall_point_sequence_hash == second.wall_point_sequence_hash &&
           first.wall_model_sequence_hash == second.wall_model_sequence_hash &&
           first.wall_point_sequence_hash == source.wall_point_sequence_hash &&
           first.wall_model_sequence_hash == source.wall_model_sequence_hash &&
           first.corner_main_turn_command_count == 1 && second.corner_main_turn_command_count == 1 &&
           first.new_wall_segment_id == 2 && second.new_wall_segment_id == 2 ? 0 : 1;
}
