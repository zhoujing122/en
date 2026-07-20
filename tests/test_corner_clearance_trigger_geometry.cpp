#include "robot_slamd/config/config.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

#include <cmath>
#include <stdexcept>

int main() {
    using namespace robot_slamd;
    auto config = p5_single_corner_test_config();
    const double minimum = std::max(0.0,
        config.stop_go_corner_robot_turning_sweep_radius_mm +
        config.stop_go_corner_turn_clearance_margin_mm -
        config.stop_go_corner_front_tof_forward_offset_mm) +
        config.stop_go_forward_step_mm + config.stop_go_corner_expected_forward_overrun_mm;
    if (config.stop_go_corner_trigger_distance_mm < minimum) return 1;
    config.stop_go_corner_trigger_distance_mm = minimum - 1.0;
    bool rejected = false;
    try { validate_config(config); } catch (const std::exception &) { rejected = true; }
    if (!rejected) return 1;
    config = p5_single_corner_test_config();
    const auto clear = StopGoStraightWallSimulationRunner{}.run_scenario(
        config, "/tmp/p5_corner_clearance_clear", "nominal_single_corner");
    const auto blocked = StopGoStraightWallSimulationRunner{}.run_scenario(
        config, "/tmp/p5_corner_clearance_blocked", "right_turn_clearance_blocked");
    return clear.ok && blocked.termination_reason == "turn_clearance_blocked" &&
           blocked.corner_main_turn_command_count == 0 &&
           blocked.corner_right_clearance_reject_count > 0 ? 0 : 1;
}
