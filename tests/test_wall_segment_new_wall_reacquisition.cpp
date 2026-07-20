#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    const auto report = StopGoStraightWallSimulationRunner{}.run_scenario(
        p5_single_corner_test_config(), "/tmp/p5_wall_segment_reacquisition", "nominal_single_corner");
    return report.ok && report.old_wall_segment_id == 1 && report.new_wall_segment_id == 2 &&
           report.wall_model_reset_due_to_corner_count == 1 && report.new_wall_model_valid &&
           report.new_wall_reacquisition_samples > 0 && report.post_corner_follow_steps >= 5 &&
           report.old_wall_point_hash != report.new_wall_point_hash &&
           report.old_wall_model_hash != report.new_wall_model_hash ? 0 : 1;
}
