#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

#include <algorithm>

int main() {
    using namespace robot_slamd;
    const auto report = StopGoStraightWallSimulationRunner{}.run_scenario(
        p5_single_corner_test_config(), "/tmp/p5_fixed_right_turn", "nominal_single_corner");
    const auto main_state = std::find(report.state_trace.begin(), report.state_trace.end(),
                                      "ISSUE_CORNER_RIGHT_TURN");
    const auto wait_state = std::find(report.state_trace.begin(), report.state_trace.end(),
                                      "WAIT_CORNER_TURN_DONE");
    const auto verify_state = std::find(report.state_trace.begin(), report.state_trace.end(),
                                        "VERIFY_CORNER_TURN");
    return report.ok && report.corner_main_turn_command_count == 1 &&
           report.corner_right_turn_command_count == 1 &&
           report.corner_left_turn_command_count == 0 && report.corner_main_command_id != 0 &&
           main_state != report.state_trace.end() && wait_state != report.state_trace.end() &&
           verify_state != report.state_trace.end() &&
           report.map_writes_during_corner_turn == 0 ? 0 : 1;
}
