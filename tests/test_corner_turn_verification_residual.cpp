#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

#include <cmath>

int main() {
    using namespace robot_slamd;
    const auto under = StopGoStraightWallSimulationRunner{}.run_scenario(
        p5_single_corner_test_config(), "/tmp/p5_corner_under", "under_rotation");
    const auto over = StopGoStraightWallSimulationRunner{}.run_scenario(
        p5_single_corner_test_config(), "/tmp/p5_corner_over", "over_rotation");
    const auto failed = StopGoStraightWallSimulationRunner{}.run_scenario(
        p5_single_corner_test_config(), "/tmp/p5_corner_residual_failed", "corner_turn_verification_failed");
    return under.ok && under.corner_residual_right_count > 0 &&
           std::fabs(under.final_corner_turn_error_rad) <= 3.0 * M_PI / 180.0 &&
           over.ok && over.corner_residual_left_count > 0 &&
           std::fabs(over.final_corner_turn_error_rad) <= 3.0 * M_PI / 180.0 &&
           failed.termination_reason == "corner_turn_verification_failed" &&
           !failed.new_wall_model_valid && failed.map_writes_during_turn_verification == 0 ? 0 : 1;
}
