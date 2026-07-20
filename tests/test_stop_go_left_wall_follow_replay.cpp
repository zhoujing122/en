#include "robot_slamd/replay/stop_go_replay_runner.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_left_wall_test_helpers.hpp"

#include <filesystem>

int main() {
    using namespace robot_slamd;
    const std::string root = "/tmp/p4_left_wall_replay";
    std::filesystem::create_directories(root + "/source");
    std::filesystem::create_directories(root + "/one");
    std::filesystem::create_directories(root + "/two");
    const auto simulation = StopGoStraightWallSimulationRunner{}.run_scenario(
        p4_test_config(), root + "/source", "heading_toward_wall");
    if (!simulation.ok || simulation.replay_records.empty()) return 1;
    const std::string replay_path = root + "/source/stop_go_mapping.replay";
    const auto first = StopGoReplayRunner{}.run(
        p4_test_config(), replay_path, root + "/one");
    const auto second = StopGoReplayRunner{}.run(
        p4_test_config(), replay_path, root + "/two");
    return first.ok && second.ok && first.final_map_checksum != 0 &&
           first.final_map_checksum == simulation.final_map_checksum &&
           first.final_map_checksum == second.final_map_checksum &&
           first.wall_point_sequence_hash == second.wall_point_sequence_hash &&
           first.wall_model_sequence_hash == second.wall_model_sequence_hash &&
           first.map_writes_during_turn_or_verify == 0
               ? 0 : 1;
}
