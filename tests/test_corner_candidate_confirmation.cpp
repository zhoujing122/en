#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

#include <filesystem>

int main() {
    using namespace robot_slamd;
    const auto base = p5_single_corner_test_config();
    const auto normal = StopGoStraightWallSimulationRunner{}.run_scenario(
        base, "/tmp/p5_corner_candidate_normal", "nominal_single_corner");
    const auto anomalous = StopGoStraightWallSimulationRunner{}.run_scenario(
        base, "/tmp/p5_corner_candidate_false", "false_front_measurement");
    return normal.ok && normal.corner_candidate_count > 0 &&
           normal.corner_confirmation_count >= base.stop_go_corner_confirmation_required_hits &&
           normal.map_writes_during_corner_confirm == 0 &&
           anomalous.ok && anomalous.corner_confirmation_reject_count > 0 &&
           anomalous.corner_main_turn_command_count == 1 ? 0 : 1;
}
