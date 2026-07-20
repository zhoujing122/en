#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

#include <filesystem>

int main() {
    using namespace robot_slamd;
    auto valid_rectangle = p6_rectangle_test_config();
    bool valid_config_accepted = true;
    try {
        validate_config(valid_rectangle);
    } catch (const std::exception &) {
        valid_config_accepted = false;
    }
    auto real_rectangle = valid_rectangle;
    real_rectangle.runtime_sensor_source = "real";
    bool real_rejected = false;
    try {
        validate_config(real_rectangle);
    } catch (const std::exception &) {
        real_rejected = true;
    }
    auto invalid_confirmation = valid_rectangle;
    invalid_confirmation.stop_go_rectangle_closure_confirmation_samples = 2;
    invalid_confirmation.stop_go_rectangle_closure_confirmation_required_passes = 3;
    bool invalid_confirmation_rejected = false;
    try {
        validate_config(invalid_confirmation);
    } catch (const std::exception &) {
        invalid_confirmation_rejected = true;
    }
    const auto p5 = StopGoStraightWallSimulationRunner{}.run_scenario(
        p5_single_corner_test_config(), "/tmp/p6_compat_p5", "nominal_single_corner");
    const auto p6 = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_compat_rectangle", "nominal_rectangle");
    return valid_config_accepted && real_rejected && invalid_confirmation_rejected &&
           p5.ok && p5.termination_reason == "single_corner_complete" &&
           p5.corner_main_turn_command_count == 1 &&
           p6.ok && p6.corner_transition_count == 4 &&
           p6.wall_segment_sequence == std::vector<std::uint64_t>{1, 2, 3, 4, 5}
        ? 0 : 1;
}
