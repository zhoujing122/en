#include "robot_slamd/mapping/stop_go_mapping_controller.hpp"

int main() {
    using namespace robot_slamd;
    StopGoMappingControllerConfig config;
    config.heading_deadband_rad = 1.0 * kPi / 180.0;
    config.distance_deadband_m = 0.01;
    config.distance_to_heading_gain_rad_per_m = 20.0 * kPi / 180.0;
    config.max_distance_bias_rad = 3.0 * kPi / 180.0;
    config.min_correction_deg = 1.0;
    config.max_correction_deg = 3.0;
    LeftWallModel wall;
    wall.valid = true;
    wall.wall_heading_rad = 0.0;
    wall.signed_base_to_wall_distance_m = 0.15;
    const auto toward = decide_left_wall_control(wall, {0.0, 0.0, 5.0 * kPi / 180.0}, 0.15, config);
    const auto away = decide_left_wall_control(wall, {0.0, 0.0, -5.0 * kPi / 180.0}, 0.15, config);
    wall.signed_base_to_wall_distance_m = 0.30;
    const auto far = decide_left_wall_control(wall, {0.0, 0.0, 0.0}, 0.15, config);
    wall.signed_base_to_wall_distance_m = 0.09;
    const auto close = decide_left_wall_control(wall, {0.0, 0.0, 0.0}, 0.15, config);
    return toward.action == LeftWallControlAction::Right &&
           away.action == LeftWallControlAction::Left &&
           far.action == LeftWallControlAction::Left &&
           close.action == LeftWallControlAction::Right &&
           toward.correction_amount_deg <= 3.0 && close.correction_amount_deg <= 3.0
               ? 0 : 1;
}
