#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_left_wall_test_helpers.hpp"

#include <filesystem>

int main() {
    const std::string root = "/tmp/p4_safety_epoch";
    std::filesystem::create_directories(root + "/loss");
    std::filesystem::create_directories(root + "/blocked");
    const auto loss = robot_slamd::StopGoStraightWallSimulationRunner{}
        .run_scenario(p4_test_config(), root + "/loss", "left_wall_loss");
    const auto blocked = robot_slamd::StopGoStraightWallSimulationRunner{}
        .run_scenario(p4_test_config(), root + "/blocked", "front_blocked");
    return !loss.ok && loss.termination_reason.find("left_wall") != std::string::npos &&
           loss.correction_command_count == 0 && !blocked.ok &&
           blocked.termination_reason.find("front_") != std::string::npos &&
           blocked.correction_command_count == 0
               ? 0 : 1;
}
