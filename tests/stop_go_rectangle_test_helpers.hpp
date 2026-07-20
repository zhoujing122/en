#pragma once

#include "robot_slamd/config/config.hpp"

inline robot_slamd::Config p6_rectangle_test_config() {
    robot_slamd::Config c;
    c.runtime_sensor_source = "simulation";
    c.runtime_operation = "stop_go_wall_mapping";
    c.stop_go_mapping_enabled = true;
    c.stop_go_mapping_mode = "rectangle_loop";
    c.stop_go_forward_step_mm = 8.0;
    c.stop_go_max_steps = 500;
    c.stop_go_max_total_distance_mm = 6000.0;
    c.stop_go_motion_max_rpm = 12.0;
    c.stop_go_motion_timeout_ms = 6000;
    c.stop_go_desired_distance_mode = "configured";
    // The P5 trigger stops 0.45 m from each synthetic corner (front offset
    // plus trigger range); this Simulation-provisional target keeps every
    // rectangle wall at the same safe turning distance.
    c.stop_go_desired_base_to_wall_distance_mm = 400.0;
    c.stop_go_wall_window_max_points = 10;
    c.stop_go_wall_min_fit_points = 5;
    c.stop_go_wall_min_baseline_mm = 50.0;
    c.stop_go_wall_max_fit_rms_mm = 30.0;
    c.stop_go_wall_outlier_min_threshold_mm = 20.0;
    c.stop_go_wall_outlier_mad_scale = 3.0;
    c.stop_go_wall_acquisition_max_forward_steps = 12;
    c.stop_go_wall_max_stationary_resample_attempts = 3;
    c.stop_go_wall_max_consecutive_misses = 2;
    c.stop_go_wall_min_allowed_distance_mm = 50.0;
    c.stop_go_wall_max_allowed_distance_mm = 500.0;
    c.stop_go_front_stop_threshold_mm = 200.0;
    c.stop_go_front_max_invalid_samples = 2;
    c.stop_go_corner_maximum_transitions = 1;
    c.stop_go_corner_confirmation_resamples = 2;
    c.stop_go_corner_confirmation_required_hits = 2;
    c.stop_go_corner_robot_turning_sweep_radius_mm = 80.0;
    c.stop_go_corner_turn_clearance_margin_mm = 20.0;
    c.stop_go_corner_front_tof_forward_offset_mm = 100.0;
    c.stop_go_corner_expected_forward_overrun_mm = 10.0;
    c.stop_go_corner_trigger_distance_mm = 350.0;
    c.stop_go_corner_minimum_right_turn_clearance_mm = 200.0;
    c.stop_go_corner_right_clearance_resample_attempts = 2;
    c.stop_go_corner_main_turn_deg = 90.0;
    c.stop_go_corner_turn_acceptance_tolerance_deg = 3.0;
    c.stop_go_corner_max_residual_correction_deg = 8.0;
    c.stop_go_corner_max_residual_corrections = 2;
    c.stop_go_corner_min_residual_correction_deg = 1.0;
    c.stop_go_corner_post_turn_front_stop_threshold_mm = 200.0;
    c.stop_go_corner_post_turn_sensor_resample_attempts = 3;
    c.stop_go_corner_new_wall_min_distance_mm = 80.0;
    c.stop_go_corner_new_wall_max_distance_mm = 500.0;
    c.stop_go_corner_new_wall_stationary_resample_attempts = 3;
    c.stop_go_corner_new_wall_acquisition_max_forward_steps = 8;
    c.stop_go_corner_post_corner_required_follow_steps = 2;
    c.stop_go_rectangle_target_corner_transitions = 4;
    c.stop_go_rectangle_maximum_corner_transitions = 4;
    c.stop_go_rectangle_minimum_follow_steps_before_next_corner = 5;
    c.stop_go_rectangle_minimum_odom_distance_before_next_corner_mm = 100.0;
    c.stop_go_rectangle_corner_rearm_front_clearance_mm = 300.0;
    c.stop_go_rectangle_minimum_total_distance_before_closure_mm = 1000.0;
    c.stop_go_rectangle_minimum_steps_after_fourth_corner = 2;
    c.stop_go_rectangle_minimum_distance_after_fourth_corner_mm = 40.0;
    c.stop_go_rectangle_closure_position_tolerance_mm = 100.0;
    c.stop_go_rectangle_closure_yaw_tolerance_deg = 5.0;
    c.stop_go_rectangle_closure_wall_heading_tolerance_deg = 8.0;
    c.stop_go_rectangle_closure_wall_distance_tolerance_mm = 35.0;
    c.stop_go_rectangle_closure_wall_line_offset_tolerance_mm = 80.0;
    c.stop_go_rectangle_closure_confirmation_samples = 3;
    c.stop_go_rectangle_closure_confirmation_required_passes = 3;
    c.stop_go_rectangle_closure_confirmation_max_attempts = 5;
    c.stop_go_rectangle_maximum_post_fourth_corner_forward_steps = 30;
    c.stop_go_rectangle_maximum_post_fourth_corner_odom_distance_mm = 700.0;
    c.stop_go_rectangle_maximum_total_forward_steps = 500;
    c.stop_go_rectangle_maximum_total_odom_distance_mm = 6000.0;
    c.stop_go_rectangle_maximum_runtime_s = 600.0;
    c.stop_go_rectangle_map_quality_max_p95_wall_thickness_cells = 2.0;
    c.stop_go_rectangle_map_quality_max_ghost_occupied_ratio = 0.10;
    c.stop_go_rectangle_map_quality_max_duplicate_wall_bands = 0;
    c.stop_go_rectangle_map_quality_min_observable_wall_coverage_ratio = 0.50;
    c.sparse_slam_planar_tof_extrinsics_configured = true;
    c.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    c.sparse_slam_front_tof_x_m = 0.10;
    c.sparse_slam_left_tof_y_m = 0.08;
    c.sparse_slam_left_tof_yaw_rad = 1.5707963267948966;
    c.sparse_slam_right_tof_y_m = -0.08;
    c.sparse_slam_right_tof_yaw_rad = -1.5707963267948966;
    c.exploration_simulation_initial_x_m = -0.65;
    c.exploration_simulation_initial_y_m = 0.15;
    c.exploration_simulation_initial_yaw_rad = 0.0;
    c.sparse_slam_map_id = "formal_p6_rectangle";
    c.sparse_slam_map_run_id = "formal_p6_rectangle";
    return c;
}
