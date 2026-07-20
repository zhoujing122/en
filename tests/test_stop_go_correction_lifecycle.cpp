#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_left_wall_test_helpers.hpp"

#include <algorithm>
#include <filesystem>

int main() {
    const std::string dir = "/tmp/p4_correction_lifecycle";
    std::filesystem::create_directories(dir);
    const auto report = robot_slamd::StopGoStraightWallSimulationRunner{}
        .run_scenario(p4_test_config(), dir, "heading_toward_wall");
    const auto has = [&report](const char *state) {
        return std::find(report.state_trace.begin(), report.state_trace.end(), state) !=
               report.state_trace.end();
    };
    return report.ok && report.correction_command_count > 0 &&
           report.right_correction_count > 0 && report.left_correction_count == 0 &&
           report.post_turn_verification_count > 0 &&
           report.post_turn_verification_failure_count == 0 &&
           report.map_writes_during_turn_or_verify == 0 &&
           has("ISSUE_TURN_CORRECTION") && has("WAIT_TURN_DONE") &&
           has("WAIT_TURN_SETTLE") && has("VERIFY_AFTER_TURN")
               ? 0 : 1;
}
