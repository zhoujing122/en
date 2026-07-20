#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"

#include <cmath>

int main() {
    using namespace robot_slamd;
    const auto report = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_closure_confirmation", "nominal_rectangle");
    return report.ok && report.closure_candidate_count >= 1 &&
           report.closure_confirmation_attempt_count >= 3 &&
           report.closure_confirmation_pass_count >= 3 &&
           report.map_writes_during_closure_confirm == 0 &&
           report.estimated_closure_position_error_m <= 0.10 &&
           std::fabs(report.estimated_closure_yaw_error_rad) <= 5.0 * kPi / 180.0 &&
           !report.forced_pose_snap_used && !report.rectangle_geometry_snap_used &&
           report.controller_map_odom_write_attempt_count == 0 ? 0 : 1;
}
