#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    const auto clearance = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_failure_clearance", "corner_clearance_failure");
    const auto missing = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_failure_wall", "new_wall_loss_mid_run");
    const auto turn = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_failure_turn", "corner_turn_verification_failed");
    const auto bounded = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_failure_closure", "closure_not_reached");
    const auto extra = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_failure_extra_corner",
        "unexpected_extra_corner_before_closure");
    auto runtime_limited_config = p6_rectangle_test_config();
    runtime_limited_config.stop_go_rectangle_maximum_runtime_s = 0.10;
    const auto runtime_limited = StopGoStraightWallSimulationRunner{}.run_scenario(
        runtime_limited_config, "/tmp/p6_failure_runtime", "nominal_rectangle");
    return clearance.termination_reason == "turn_clearance_blocked" &&
           clearance.corner_transition_count < 4 &&
           clearance.corner_main_turn_command_count == 1 &&
           missing.termination_reason == "new_left_wall_reacquisition_failed" &&
           missing.corner_transition_count == 0 &&
           turn.termination_reason == "corner_turn_verification_failed" &&
           turn.corner_transition_count == 0 &&
           bounded.termination_reason == "rectangle_closure_not_reached" &&
           bounded.corner_transition_count == 4 && !bounded.final_map_saved &&
           extra.termination_reason == "unexpected_extra_corner_before_closure" &&
           extra.corner_transition_count == 4 && !extra.final_map_saved &&
           runtime_limited.termination_reason == "rectangle_closure_not_reached" &&
           runtime_limited.corner_transition_count < 4 &&
           !runtime_limited.final_map_saved ? 0 : 1;
}
