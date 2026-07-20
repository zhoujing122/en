#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"

#include <cmath>

int main() {
    using namespace robot_slamd;
    const auto first = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_anchor_one", "nominal_rectangle");
    const auto second = StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_anchor_two", "nominal_rectangle");
    return first.ok && second.ok && first.run_start_anchor.valid &&
           first.run_start_anchor.start_wall_segment_id == 1 &&
           std::isfinite(first.run_start_anchor.initial_wall_line_offset_m) &&
           first.run_start_anchor.initial_wall_model_input_point_count >= 5 &&
           first.run_start_anchor.initial_wall_model_inlier_point_count >= 5 &&
           first.run_start_anchor.completed_forward_steps_at_start < first.completed_steps &&
           first.total_odom_travel_distance_m > 3.0 &&
           first.total_odom_travel_distance_m != first.completed_steps * 0.020 &&
           !first.command_target_used_as_odometry &&
           first.run_anchor_hash == second.run_anchor_hash &&
           first.command_ids == second.command_ids &&
           first.wall_segment_sequence == second.wall_segment_sequence &&
           first.wall_point_sequence_hash == second.wall_point_sequence_hash &&
           first.wall_model_sequence_hash == second.wall_model_sequence_hash &&
           first.final_map_checksum == second.final_map_checksum &&
           std::fabs(first.total_odom_travel_distance_m -
                     second.total_odom_travel_distance_m) < 1e-12 ? 0 : 1;
}
