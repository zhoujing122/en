#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"

#include <cmath>

int main() {
    const auto report = robot_slamd::StopGoStraightWallSimulationRunner{}.run_scenario(
        p6_rectangle_test_config(), "/tmp/p6_map_quality", "nominal_rectangle");
    return report.ok && report.final_map_saved && report.final_map_reload_verified &&
           report.final_map_checksum == report.final_map_reload_checksum &&
           report.map_cells > 0 && report.map_has_bounds &&
           report.map_min_x_cell <= report.map_max_x_cell &&
           report.map_min_y_cell <= report.map_max_y_cell &&
           report.map_occupied_cell_count > 0 && report.map_free_cell_count > 0 &&
           report.observable_wall_coverage_ratio >= 0.50 &&
           report.p95_wall_thickness_cells <= 2.0 &&
           report.ghost_occupied_cell_ratio <= 0.10 &&
           report.duplicate_wall_band_count == 0 && report.final_settle_complete &&
           report.ground_truth_final_position_error_m <= 0.10 &&
           std::fabs(report.ground_truth_final_yaw_error_rad) <=
               5.0 * robot_slamd::kPi / 180.0 &&
           report.final_mapping_commit_count == 1 &&
           report.commands_submitted_after_completion == 0 ? 0 : 1;
}
