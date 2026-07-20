#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    const auto config = p5_single_corner_test_config();
    const auto clearance = StopGoStraightWallSimulationRunner{}.run_scenario(
        config, "/tmp/p5_safety_clearance", "right_turn_clearance_blocked");
    const auto missing = StopGoStraightWallSimulationRunner{}.run_scenario(
        config, "/tmp/p5_safety_missing", "new_left_wall_missing");
    const auto verify = StopGoStraightWallSimulationRunner{}.run_scenario(
        config, "/tmp/p5_safety_verify", "corner_turn_verification_failed");
    return clearance.termination_reason == "turn_clearance_blocked" &&
           clearance.corner_main_turn_command_count == 0 &&
           missing.termination_reason == "new_left_wall_reacquisition_failed" &&
           verify.termination_reason == "corner_turn_verification_failed" &&
           verify.corner_main_turn_command_count == 1 ? 0 : 1;
}
