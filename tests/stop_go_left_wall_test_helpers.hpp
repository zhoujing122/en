#pragma once

#include "robot_slamd/config/config.hpp"

#include <cmath>

inline robot_slamd::Config p4_test_config() {
    robot_slamd::Config c;
    c.runtime_sensor_source = "simulation";
    c.runtime_operation = "stop_go_wall_mapping";
    c.stop_go_mapping_enabled = true;
    c.stop_go_mapping_mode = "left_wall_follow";
    c.stop_go_max_steps = 40;
    c.stop_go_max_total_distance_mm = 800.0;
    c.stop_go_desired_distance_mode = "configured";
    c.stop_go_desired_base_to_wall_distance_mm = 150.0;
    c.stop_go_wall_window_max_points = 10;
    c.stop_go_wall_min_fit_points = 5;
    c.stop_go_wall_min_baseline_mm = 50.0;
    c.stop_go_wall_max_fit_rms_mm = 30.0;
    c.stop_go_wall_outlier_min_threshold_mm = 20.0;
    c.stop_go_wall_outlier_mad_scale = 3.0;
    c.stop_go_wall_acquisition_max_forward_steps = 6;
    c.stop_go_wall_max_stationary_resample_attempts = 3;
    c.stop_go_wall_max_consecutive_misses = 2;
    c.stop_go_wall_heading_deadband_deg = 1.0;
    c.stop_go_wall_distance_deadband_mm = 10.0;
    c.stop_go_wall_distance_to_heading_gain_deg_per_m = 20.0;
    c.stop_go_wall_max_distance_bias_deg = 3.0;
    c.stop_go_wall_min_correction_deg = 1.0;
    c.stop_go_wall_max_correction_deg = 3.0;
    c.stop_go_wall_min_allowed_distance_mm = 50.0;
    c.stop_go_wall_max_allowed_distance_mm = 500.0;
    c.stop_go_front_stop_threshold_mm = 200.0;
    c.stop_go_front_max_invalid_samples = 2;
    c.sparse_slam_planar_tof_extrinsics_configured = true;
    c.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    c.sparse_slam_front_tof_x_m = 0.10;
    c.sparse_slam_left_tof_y_m = 0.08;
    c.sparse_slam_left_tof_yaw_rad = 1.5707963267948966;
    c.sparse_slam_right_tof_y_m = -0.08;
    c.sparse_slam_right_tof_yaw_rad = -1.5707963267948966;
    c.exploration_simulation_initial_x_m = -1.2;
    c.exploration_simulation_initial_y_m = 0.45;
    c.exploration_simulation_initial_yaw_rad = 0.0;
    c.sparse_slam_map_run_id = "formal_p4_test";
    return c;
}
