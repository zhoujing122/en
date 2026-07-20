#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_left_wall_test_helpers.hpp"

#include <filesystem>

int main() {
    const std::string dir = "/tmp/p4_wall_acquisition_gate";
    std::filesystem::create_directories(dir);
    auto config = p4_test_config();
    config.stop_go_max_steps = 2;
    config.stop_go_max_total_distance_mm = 40.0;
    const auto report = robot_slamd::StopGoStraightWallSimulationRunner{}
        .run_scenario(config, dir, "nominal");
    return report.ok && report.core_ready_observed && report.core_wait_ticks > 0 &&
           report.commands_submitted == report.forward_command_count &&
           report.wall_acquisition_steps > 0 && report.map_writes_while_moving == 0
               ? 0 : 1;
}
