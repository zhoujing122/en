#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/mapping/left_wall_line_estimator.hpp"
#include "robot_slamd/autonomy/ports/relative_motion_step_port.hpp"
#include "robot_slamd/runtime/mapping_settle_gate.hpp"
#include "robot_slamd/sensors/stable_three_tof_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace robot_slamd {

enum class StopGoMappingControllerState {
    Idle,
    WaitingForCoreReady,
    InitialSettle,
    InitialSample,
    WallAcquisition,
    IssueForwardStep,
    WaitMotionDone,
    WaitMappingSettle,
    AcquireThreeTof,
    CommitMappingSample,
    IssueTurnCorrection,
    WaitTurnDone,
    WaitTurnSettle,
    VerifyAfterTurn,
    CornerCandidate,
    CornerConfirming,
    IssueCornerRightTurn,
    WaitCornerTurnDone,
    WaitCornerTurnSettle,
    VerifyCornerTurn,
    IssueCornerResidualCorrection,
    WaitCornerResidualDone,
    WaitCornerResidualSettle,
    PostCornerTurnSensorVerify,
    NewLeftWallReacquisition,
    CheckLimit,
    RectangleClosureCandidate,
    RectangleClosureConfirming,
    FinalSettle,
    FinalMappingCommit,
    Completed,
    SingleCornerComplete,
    WallLost,
    FrontBlocked,
    TurnClearanceBlocked,
    CornerTurnVerificationFailed,
    PostTurnFrontBlocked,
    NewLeftWallReacquisitionFailed,
    Fault,
    Estop
};

inline const char *to_string(StopGoMappingControllerState state) {
    switch (state) {
    case StopGoMappingControllerState::Idle: return "IDLE";
    case StopGoMappingControllerState::WaitingForCoreReady: return "WAITING_FOR_CORE_READY";
    case StopGoMappingControllerState::InitialSettle: return "INITIAL_SETTLE";
    case StopGoMappingControllerState::InitialSample: return "INITIAL_SAMPLE";
    case StopGoMappingControllerState::WallAcquisition: return "WALL_ACQUISITION";
    case StopGoMappingControllerState::IssueForwardStep: return "ISSUE_FORWARD_STEP";
    case StopGoMappingControllerState::WaitMotionDone: return "WAIT_MOTION_DONE";
    case StopGoMappingControllerState::WaitMappingSettle: return "WAIT_MAPPING_SETTLE";
    case StopGoMappingControllerState::AcquireThreeTof: return "ACQUIRE_THREE_TOF";
    case StopGoMappingControllerState::CommitMappingSample: return "COMMIT_MAPPING_SAMPLE";
    case StopGoMappingControllerState::IssueTurnCorrection: return "ISSUE_TURN_CORRECTION";
    case StopGoMappingControllerState::WaitTurnDone: return "WAIT_TURN_DONE";
    case StopGoMappingControllerState::WaitTurnSettle: return "WAIT_TURN_SETTLE";
    case StopGoMappingControllerState::VerifyAfterTurn: return "VERIFY_AFTER_TURN";
    case StopGoMappingControllerState::CornerCandidate: return "CORNER_CANDIDATE";
    case StopGoMappingControllerState::CornerConfirming: return "CORNER_CONFIRMING";
    case StopGoMappingControllerState::IssueCornerRightTurn: return "ISSUE_CORNER_RIGHT_TURN";
    case StopGoMappingControllerState::WaitCornerTurnDone: return "WAIT_CORNER_TURN_DONE";
    case StopGoMappingControllerState::WaitCornerTurnSettle: return "WAIT_CORNER_TURN_SETTLE";
    case StopGoMappingControllerState::VerifyCornerTurn: return "VERIFY_CORNER_TURN";
    case StopGoMappingControllerState::IssueCornerResidualCorrection: return "ISSUE_CORNER_RESIDUAL_CORRECTION";
    case StopGoMappingControllerState::WaitCornerResidualDone: return "WAIT_CORNER_RESIDUAL_DONE";
    case StopGoMappingControllerState::WaitCornerResidualSettle: return "WAIT_CORNER_RESIDUAL_SETTLE";
    case StopGoMappingControllerState::PostCornerTurnSensorVerify: return "POST_CORNER_TURN_SENSOR_VERIFY";
    case StopGoMappingControllerState::NewLeftWallReacquisition: return "NEW_LEFT_WALL_REACQUISITION";
    case StopGoMappingControllerState::CheckLimit: return "CHECK_LIMIT";
    case StopGoMappingControllerState::RectangleClosureCandidate: return "RECTANGLE_CLOSURE_CANDIDATE";
    case StopGoMappingControllerState::RectangleClosureConfirming: return "RECTANGLE_CLOSURE_CONFIRMING";
    case StopGoMappingControllerState::FinalSettle: return "FINAL_SETTLE";
    case StopGoMappingControllerState::FinalMappingCommit: return "FINAL_MAPPING_COMMIT";
    case StopGoMappingControllerState::Completed: return "COMPLETED";
    case StopGoMappingControllerState::SingleCornerComplete: return "SINGLE_CORNER_COMPLETE";
    case StopGoMappingControllerState::WallLost: return "WALL_LOST";
    case StopGoMappingControllerState::FrontBlocked: return "FRONT_BLOCKED";
    case StopGoMappingControllerState::TurnClearanceBlocked: return "TURN_CLEARANCE_BLOCKED";
    case StopGoMappingControllerState::CornerTurnVerificationFailed: return "CORNER_TURN_VERIFICATION_FAILED";
    case StopGoMappingControllerState::PostTurnFrontBlocked: return "POST_TURN_FRONT_BLOCKED";
    case StopGoMappingControllerState::NewLeftWallReacquisitionFailed: return "NEW_LEFT_WALL_REACQUISITION_FAILED";
    case StopGoMappingControllerState::Fault: return "FAULT";
    case StopGoMappingControllerState::Estop: return "ESTOP";
    }
    return "UNKNOWN";
}

struct StopGoMappingControllerConfig {
    std::string mode = "straight";
    double forward_step_mm = 20.0;
    int max_steps = 20;
    double max_total_distance_mm = 400.0;
    double motion_max_rpm = 12.0;
    std::uint64_t motion_timeout_ms = 3000;
    MappingSettleGateConfig settle;
    StableThreeTofSamplerConfig tof;
    std::string log_path;
    std::string desired_distance_mode = "configured";
    double desired_base_to_wall_distance_m = 0.15;
    LeftWallLineEstimatorConfig wall_estimator;
    std::size_t wall_acquisition_max_forward_steps = 6;
    std::size_t max_stationary_resample_attempts = 3;
    std::size_t max_consecutive_wall_misses = 2;
    double heading_deadband_rad = 1.0 * 3.14159265358979323846 / 180.0;
    double distance_deadband_m = 0.010;
    double distance_to_heading_gain_rad_per_m = 20.0 * 3.14159265358979323846 / 180.0;
    double max_distance_bias_rad = 3.0 * 3.14159265358979323846 / 180.0;
    double min_correction_deg = 1.0;
    double max_correction_deg = 3.0;
    double min_allowed_base_to_wall_distance_m = 0.05;
    double max_allowed_base_to_wall_distance_m = 0.50;
    double front_stop_threshold_m = 0.20;
    std::size_t front_max_invalid_samples = 2;
    RobotPose2D base_T_left_tof;
    std::size_t maximum_corner_transitions = 1;
    std::size_t corner_confirmation_resamples = 2;
    std::size_t corner_confirmation_required_hits = 2;
    double robot_turning_sweep_radius_m = 0.080;
    double turn_clearance_margin_m = 0.020;
    double front_tof_forward_offset_m = 0.100;
    double expected_forward_overrun_m = 0.010;
    double corner_trigger_distance_m = 0.350;
    double minimum_right_turn_clearance_m = 0.200;
    std::size_t right_clearance_resample_attempts = 2;
    double main_turn_deg = 90.0;
    double corner_turn_acceptance_tolerance_rad = 3.0 * 3.14159265358979323846 / 180.0;
    double corner_turn_max_residual_correction_rad = 8.0 * 3.14159265358979323846 / 180.0;
    std::size_t corner_turn_max_residual_corrections = 2;
    double min_corner_residual_correction_deg = 1.0;
    double post_turn_front_stop_threshold_m = 0.20;
    std::size_t post_turn_sensor_resample_attempts = 3;
    double new_wall_min_distance_m = 0.080;
    double new_wall_max_distance_m = 0.500;
    std::size_t new_wall_stationary_resample_attempts = 3;
    std::size_t new_wall_acquisition_max_forward_steps = 8;
    std::size_t post_corner_required_follow_steps = 5;
    std::size_t rectangle_target_corner_transitions = 4;
    std::size_t rectangle_maximum_corner_transitions = 4;
    std::size_t rectangle_minimum_follow_steps_before_next_corner = 5;
    double rectangle_minimum_odom_distance_before_next_corner_m = 0.10;
    double rectangle_corner_rearm_front_clearance_m = 0.45;
    double rectangle_minimum_total_distance_before_closure_m = 1.0;
    std::size_t rectangle_minimum_steps_after_fourth_corner = 5;
    double rectangle_minimum_distance_after_fourth_corner_m = 0.10;
    double rectangle_closure_position_tolerance_m = 0.10;
    double rectangle_closure_yaw_tolerance_rad = 5.0 * 3.14159265358979323846 / 180.0;
    double rectangle_closure_wall_heading_tolerance_rad = 8.0 * 3.14159265358979323846 / 180.0;
    double rectangle_closure_wall_distance_tolerance_m = 0.035;
    double rectangle_closure_wall_line_offset_tolerance_m = 0.080;
    std::size_t rectangle_closure_confirmation_samples = 3;
    std::size_t rectangle_closure_confirmation_required_passes = 3;
    std::size_t rectangle_closure_confirmation_max_attempts = 5;
    std::size_t rectangle_maximum_post_fourth_corner_forward_steps = 30;
    double rectangle_maximum_post_fourth_corner_odom_distance_m = 0.70;
    std::size_t rectangle_maximum_total_forward_steps = 240;
    double rectangle_maximum_total_odom_distance_m = 6.0;
    double rectangle_maximum_runtime_s = 180.0;
    double rectangle_max_p95_wall_thickness_cells = 2.0;
    double rectangle_max_ghost_occupied_ratio = 0.10;
    std::size_t rectangle_max_duplicate_wall_bands = 0;
    double rectangle_min_observable_wall_coverage_ratio = 0.60;
    std::string rectangle_map_output_path;
    std::string rectangle_run_summary_output_path;
    bool rectangle_save_failure_diagnostic_map = true;
    std::string rectangle_failure_map_output_path;
};

inline StopGoMappingControllerConfig stop_go_mapping_config_from(
    const Config &config) {
    StopGoMappingControllerConfig result;
    result.forward_step_mm = config.stop_go_forward_step_mm;
    result.max_steps = config.stop_go_max_steps;
    result.max_total_distance_mm = config.stop_go_max_total_distance_mm;
    result.motion_max_rpm = config.stop_go_motion_max_rpm;
    result.motion_timeout_ms = static_cast<std::uint64_t>(config.stop_go_motion_timeout_ms);
    result.settle.wheel_rpm_threshold = config.stop_go_settle_wheel_rpm_threshold;
    result.settle.imu_gyro_threshold_deg_s = config.stop_go_settle_imu_gyro_threshold_deg_s;
    result.settle.consecutive_frames = static_cast<std::size_t>(config.stop_go_settle_consecutive_frames);
    result.settle.hold_ms = static_cast<std::uint64_t>(config.stop_go_settle_hold_ms);
    result.tof.samples_per_sensor = static_cast<std::size_t>(config.stop_go_tof_samples_per_sensor);
    result.tof.min_valid_samples = static_cast<std::size_t>(config.stop_go_tof_min_valid_samples);
    result.tof.protocol_min_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_protocol_min_distance_mm);
    result.tof.protocol_max_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_protocol_max_distance_mm);
    result.tof.mapping_min_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_mapping_min_distance_mm);
    result.tof.mapping_max_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_mapping_max_distance_mm);
    result.tof.mapping_min_confidence = static_cast<std::uint8_t>(config.stop_go_tof_mapping_min_confidence);
    result.log_path = config.stop_go_log_path;
    result.mode = config.stop_go_mapping_mode;
    result.desired_distance_mode = config.stop_go_desired_distance_mode;
    result.desired_base_to_wall_distance_m =
        config.stop_go_desired_base_to_wall_distance_mm / 1000.0;
    result.wall_estimator.window_max_points =
        static_cast<std::size_t>(config.stop_go_wall_window_max_points);
    result.wall_estimator.min_fit_points =
        static_cast<std::size_t>(config.stop_go_wall_min_fit_points);
    result.wall_estimator.min_baseline_m =
        config.stop_go_wall_min_baseline_mm / 1000.0;
    result.wall_estimator.max_fit_rms_m =
        config.stop_go_wall_max_fit_rms_mm / 1000.0;
    result.wall_estimator.outlier_min_threshold_m =
        config.stop_go_wall_outlier_min_threshold_mm / 1000.0;
    result.wall_estimator.outlier_mad_scale = config.stop_go_wall_outlier_mad_scale;
    result.wall_acquisition_max_forward_steps = static_cast<std::size_t>(
        config.stop_go_wall_acquisition_max_forward_steps);
    result.max_stationary_resample_attempts = static_cast<std::size_t>(
        config.stop_go_wall_max_stationary_resample_attempts);
    result.max_consecutive_wall_misses = static_cast<std::size_t>(
        config.stop_go_wall_max_consecutive_misses);
    result.heading_deadband_rad = config.stop_go_wall_heading_deadband_deg *
        3.14159265358979323846 / 180.0;
    result.distance_deadband_m = config.stop_go_wall_distance_deadband_mm / 1000.0;
    result.distance_to_heading_gain_rad_per_m =
        config.stop_go_wall_distance_to_heading_gain_deg_per_m *
        3.14159265358979323846 / 180.0;
    result.max_distance_bias_rad = config.stop_go_wall_max_distance_bias_deg *
        3.14159265358979323846 / 180.0;
    result.min_correction_deg = config.stop_go_wall_min_correction_deg;
    result.max_correction_deg = config.stop_go_wall_max_correction_deg;
    result.min_allowed_base_to_wall_distance_m =
        config.stop_go_wall_min_allowed_distance_mm / 1000.0;
    result.max_allowed_base_to_wall_distance_m =
        config.stop_go_wall_max_allowed_distance_mm / 1000.0;
    result.front_stop_threshold_m = config.stop_go_front_stop_threshold_mm / 1000.0;
    result.front_max_invalid_samples = static_cast<std::size_t>(
        config.stop_go_front_max_invalid_samples);
    result.base_T_left_tof = RobotPose2D{
        config.sparse_slam_left_tof_x_m,
        config.sparse_slam_left_tof_y_m,
        config.sparse_slam_left_tof_yaw_rad};
    result.maximum_corner_transitions = static_cast<std::size_t>(config.stop_go_corner_maximum_transitions);
    result.corner_confirmation_resamples = static_cast<std::size_t>(config.stop_go_corner_confirmation_resamples);
    result.corner_confirmation_required_hits = static_cast<std::size_t>(config.stop_go_corner_confirmation_required_hits);
    result.robot_turning_sweep_radius_m = config.stop_go_corner_robot_turning_sweep_radius_mm / 1000.0;
    result.turn_clearance_margin_m = config.stop_go_corner_turn_clearance_margin_mm / 1000.0;
    result.front_tof_forward_offset_m = config.stop_go_corner_front_tof_forward_offset_mm / 1000.0;
    result.expected_forward_overrun_m = config.stop_go_corner_expected_forward_overrun_mm / 1000.0;
    result.corner_trigger_distance_m = config.stop_go_corner_trigger_distance_mm / 1000.0;
    result.minimum_right_turn_clearance_m = config.stop_go_corner_minimum_right_turn_clearance_mm / 1000.0;
    result.right_clearance_resample_attempts = static_cast<std::size_t>(config.stop_go_corner_right_clearance_resample_attempts);
    result.main_turn_deg = config.stop_go_corner_main_turn_deg;
    result.corner_turn_acceptance_tolerance_rad = config.stop_go_corner_turn_acceptance_tolerance_deg * 3.14159265358979323846 / 180.0;
    result.corner_turn_max_residual_correction_rad = config.stop_go_corner_max_residual_correction_deg * 3.14159265358979323846 / 180.0;
    result.corner_turn_max_residual_corrections = static_cast<std::size_t>(config.stop_go_corner_max_residual_corrections);
    result.min_corner_residual_correction_deg = config.stop_go_corner_min_residual_correction_deg;
    result.post_turn_front_stop_threshold_m = config.stop_go_corner_post_turn_front_stop_threshold_mm / 1000.0;
    result.post_turn_sensor_resample_attempts = static_cast<std::size_t>(config.stop_go_corner_post_turn_sensor_resample_attempts);
    result.new_wall_min_distance_m = config.stop_go_corner_new_wall_min_distance_mm / 1000.0;
    result.new_wall_max_distance_m = config.stop_go_corner_new_wall_max_distance_mm / 1000.0;
    result.new_wall_stationary_resample_attempts = static_cast<std::size_t>(config.stop_go_corner_new_wall_stationary_resample_attempts);
    result.new_wall_acquisition_max_forward_steps = static_cast<std::size_t>(config.stop_go_corner_new_wall_acquisition_max_forward_steps);
    result.post_corner_required_follow_steps = static_cast<std::size_t>(config.stop_go_corner_post_corner_required_follow_steps);
    result.rectangle_target_corner_transitions = static_cast<std::size_t>(config.stop_go_rectangle_target_corner_transitions);
    result.rectangle_maximum_corner_transitions = static_cast<std::size_t>(config.stop_go_rectangle_maximum_corner_transitions);
    result.rectangle_minimum_follow_steps_before_next_corner = static_cast<std::size_t>(config.stop_go_rectangle_minimum_follow_steps_before_next_corner);
    result.rectangle_minimum_odom_distance_before_next_corner_m = config.stop_go_rectangle_minimum_odom_distance_before_next_corner_mm / 1000.0;
    result.rectangle_corner_rearm_front_clearance_m = config.stop_go_rectangle_corner_rearm_front_clearance_mm / 1000.0;
    result.rectangle_minimum_total_distance_before_closure_m = config.stop_go_rectangle_minimum_total_distance_before_closure_mm / 1000.0;
    result.rectangle_minimum_steps_after_fourth_corner = static_cast<std::size_t>(config.stop_go_rectangle_minimum_steps_after_fourth_corner);
    result.rectangle_minimum_distance_after_fourth_corner_m = config.stop_go_rectangle_minimum_distance_after_fourth_corner_mm / 1000.0;
    result.rectangle_closure_position_tolerance_m = config.stop_go_rectangle_closure_position_tolerance_mm / 1000.0;
    result.rectangle_closure_yaw_tolerance_rad = config.stop_go_rectangle_closure_yaw_tolerance_deg * 3.14159265358979323846 / 180.0;
    result.rectangle_closure_wall_heading_tolerance_rad = config.stop_go_rectangle_closure_wall_heading_tolerance_deg * 3.14159265358979323846 / 180.0;
    result.rectangle_closure_wall_distance_tolerance_m = config.stop_go_rectangle_closure_wall_distance_tolerance_mm / 1000.0;
    result.rectangle_closure_wall_line_offset_tolerance_m = config.stop_go_rectangle_closure_wall_line_offset_tolerance_mm / 1000.0;
    result.rectangle_closure_confirmation_samples = static_cast<std::size_t>(config.stop_go_rectangle_closure_confirmation_samples);
    result.rectangle_closure_confirmation_required_passes = static_cast<std::size_t>(config.stop_go_rectangle_closure_confirmation_required_passes);
    result.rectangle_closure_confirmation_max_attempts = static_cast<std::size_t>(config.stop_go_rectangle_closure_confirmation_max_attempts);
    result.rectangle_maximum_post_fourth_corner_forward_steps = static_cast<std::size_t>(config.stop_go_rectangle_maximum_post_fourth_corner_forward_steps);
    result.rectangle_maximum_post_fourth_corner_odom_distance_m = config.stop_go_rectangle_maximum_post_fourth_corner_odom_distance_mm / 1000.0;
    result.rectangle_maximum_total_forward_steps = static_cast<std::size_t>(config.stop_go_rectangle_maximum_total_forward_steps);
    result.rectangle_maximum_total_odom_distance_m = config.stop_go_rectangle_maximum_total_odom_distance_mm / 1000.0;
    result.rectangle_maximum_runtime_s = config.stop_go_rectangle_maximum_runtime_s;
    result.rectangle_max_p95_wall_thickness_cells = config.stop_go_rectangle_map_quality_max_p95_wall_thickness_cells;
    result.rectangle_max_ghost_occupied_ratio = config.stop_go_rectangle_map_quality_max_ghost_occupied_ratio;
    result.rectangle_max_duplicate_wall_bands = static_cast<std::size_t>(config.stop_go_rectangle_map_quality_max_duplicate_wall_bands);
    result.rectangle_min_observable_wall_coverage_ratio = config.stop_go_rectangle_map_quality_min_observable_wall_coverage_ratio;
    result.rectangle_map_output_path = config.stop_go_rectangle_map_output_path;
    result.rectangle_run_summary_output_path = config.stop_go_rectangle_run_summary_output_path;
    result.rectangle_save_failure_diagnostic_map = config.stop_go_rectangle_save_failure_diagnostic_map;
    result.rectangle_failure_map_output_path = config.stop_go_rectangle_failure_map_output_path;
    return result;
}

enum class LeftWallControlAction { NoTurn, Left, Right };

struct LeftWallControlDecision {
    LeftWallControlAction action = LeftWallControlAction::NoTurn;
    double heading_error_rad = 0.0;
    double distance_error_m = 0.0;
    double distance_bias_rad = 0.0;
    double turn_error_rad = 0.0;
    double correction_amount_deg = 0.0;
    std::string reason = "not_decided";
};

struct StopGoReplayOdomSample {
    WheelOdomFrame wheel;
    ImuFrame imu;
};

struct StopGoReplayRecord {
    std::size_t cycle_index = 0;
    std::uint64_t command_id = 0;
    RelativeMotionStepResult motion;
    StableThreeTofSample stable_sample;
    RobotSlamSensorSnapshot snapshot;
    std::uint64_t map_revision_before = 0;
    std::uint64_t map_revision_after = 0;
    std::uint64_t motor_settled_timestamp_us = 0;
    std::uint64_t mapping_ready_timestamp_us = 0;
    std::size_t settle_frame_count = 0;
    double settle_duration_ms = 0.0;
    // Read-only audit values copied from the canonical Core report.  The
    // controller never writes either state back into SLAM.
    RobotPose2D odom_pose;
    RobotPose2D map_from_odom;
    StopGoMappingControllerState controller_state = StopGoMappingControllerState::Idle;
    bool left_wall_point_accepted = false;
    LeftWallPoint left_wall_point;
    LeftWallModel wall_model;
    LeftWallControlDecision control_decision;
    bool post_turn_verified = false;
    std::uint64_t frame_transform_epoch = 0;
    bool corner_candidate = false;
    bool corner_confirmation = false;
    bool corner_confirmed = false;
    bool corner_right_clearance_passed = false;
    bool corner_main_turn = false;
    bool corner_residual_correction = false;
    bool corner_residual_right = false;
    bool post_corner_sensor_verified = false;
    bool new_wall_reacquisition = false;
    std::size_t post_corner_follow_steps = 0;
    std::size_t corner_confirmation_samples = 0;
    std::size_t corner_confirmation_rejects = 0;
    std::size_t corner_right_clearance_checks = 0;
    std::uint64_t wall_segment_id = 1;
    std::uint64_t corner_transition_id = 0;
    std::uint64_t corner_main_command_id = 0;
    std::uint64_t corner_residual_command_id = 0;
    double pre_turn_odom_yaw_rad = 0.0;
    double post_turn_odom_yaw_rad = 0.0;
    double actual_turn_delta_rad = 0.0;
    double turn_residual_rad = 0.0;
    bool closure_candidate = false;
    bool closure_confirmation_pass = false;
    bool final_mapping_commit = false;
    std::size_t corner_transition_count = 0;
    bool run_start_anchor_valid = false;
    RobotPose2D run_start_map_pose;
    RobotPose2D run_start_odom_pose;
    double run_start_timestamp_s = 0.0;
    std::uint64_t run_start_map_revision = 0;
    std::uint64_t run_start_frame_transform_epoch = 0;
    double run_start_wall_heading_rad = 0.0;
    double run_start_wall_distance_m = 0.0;
    double run_start_wall_line_offset_m = 0.0;
    double run_start_wall_rms_m = 0.0;
    double run_start_wall_baseline_m = 0.0;
    std::size_t run_start_wall_input_point_count = 0;
    std::size_t run_start_wall_inlier_point_count = 0;
    std::uint64_t run_start_wall_signature_hash = 0;
    double run_start_odom_distance_baseline_m = 0.0;
    std::size_t run_start_forward_step_baseline = 0;
    double total_odom_travel_distance_m = 0.0;
    double segment_odom_distance_m = 0.0;
    std::size_t forward_steps_since_last_corner = 0;
    std::size_t corner_rearm_count = 0;
    std::size_t closure_candidate_count = 0;
    std::size_t closure_confirmation_attempt_count = 0;
    std::size_t closure_confirmation_pass_count = 0;
    std::size_t closure_confirmation_reject_count = 0;
    double estimated_closure_position_error_m = 0.0;
    double estimated_closure_yaw_error_rad = 0.0;
    std::uint64_t final_map_revision = 0;
    std::uint64_t final_map_checksum = 0;
    std::vector<std::size_t> forward_steps_per_segment;
    std::vector<double> odom_distance_per_segment_m;
    std::vector<std::uint64_t> wall_segment_sequence;
    std::vector<std::uint64_t> corner_transition_ids;
    std::vector<StopGoReplayOdomSample> odom_samples;
};

inline LeftWallControlDecision decide_left_wall_control(
    const LeftWallModel &wall,
    const RobotPose2D &robot_pose,
    double desired_distance_m,
    const StopGoMappingControllerConfig &config) {
    LeftWallControlDecision result;
    if (!wall.valid || !sparse_slam_pose_finite(robot_pose) ||
        !std::isfinite(desired_distance_m) || desired_distance_m <= 0.0) {
        result.reason = "invalid_wall_or_pose";
        return result;
    }
    result.heading_error_rad = sparse_slam_shortest_yaw_delta(
        robot_pose.yaw_rad, wall.wall_heading_rad);
    result.distance_error_m =
        wall.signed_base_to_wall_distance_m - desired_distance_m;
    if (std::fabs(result.distance_error_m) <= config.distance_deadband_m) {
        result.distance_bias_rad = 0.0;
    } else {
        result.distance_bias_rad = std::clamp(
            config.distance_to_heading_gain_rad_per_m * result.distance_error_m,
            -config.max_distance_bias_rad, config.max_distance_bias_rad);
    }
    const double desired_heading = sparse_slam_normalize_yaw(
        wall.wall_heading_rad + result.distance_bias_rad);
    result.turn_error_rad = sparse_slam_shortest_yaw_delta(
        robot_pose.yaw_rad, desired_heading);
    const double turn_deg = std::fabs(result.turn_error_rad) *
        180.0 / 3.14159265358979323846;
    if (result.turn_error_rad > config.heading_deadband_rad) {
        result.action = LeftWallControlAction::Left;
        result.reason = "left_correction_required";
    } else if (result.turn_error_rad < -config.heading_deadband_rad) {
        result.action = LeftWallControlAction::Right;
        result.reason = "right_correction_required";
    } else {
        result.reason = "heading_and_distance_within_deadband";
    }
    if (result.action != LeftWallControlAction::NoTurn) {
        result.correction_amount_deg = std::clamp(
            turn_deg, config.min_correction_deg, config.max_correction_deg);
    }
    return result;
}

inline const char *to_string(LeftWallControlAction action) {
    switch (action) {
    case LeftWallControlAction::NoTurn: return "NO_TURN";
    case LeftWallControlAction::Left: return "LEFT";
    case LeftWallControlAction::Right: return "RIGHT";
    }
    return "UNKNOWN";
}

struct RectangleRunStartAnchor {
    bool valid = false;
    RobotPose2D start_map_pose;
    RobotPose2D start_odom_pose;
    double start_timestamp_s = 0.0;
    std::uint64_t start_map_revision = 0;
    std::uint64_t start_frame_transform_epoch = 0;
    std::uint64_t start_wall_segment_id = 1;
    double initial_wall_heading_rad = 0.0;
    double initial_base_to_wall_distance_m = 0.0;
    double initial_wall_line_offset_m = 0.0;
    double initial_wall_model_rms_m = 0.0;
    double initial_wall_model_baseline_m = 0.0;
    std::size_t initial_wall_model_input_point_count = 0;
    std::size_t initial_wall_model_inlier_point_count = 0;
    std::uint64_t initial_wall_model_signature_hash = 0;
    double odom_distance_at_start_m = 0.0;
    std::size_t completed_forward_steps_at_start = 0;
};

struct StopGoMappingRunReport {
    bool ok = false;
    std::size_t completed_steps = 0;
    std::size_t simulation_tick_count = 0;
    std::size_t commands_submitted = 0;
    std::size_t commands_completed = 0;
    std::size_t forward_command_count = 0;
    std::size_t correction_command_count = 0;
    std::size_t left_correction_count = 0;
    std::size_t right_correction_count = 0;
    std::size_t wall_acquisition_steps = 0;
    std::size_t wall_model_valid_count = 0;
    std::size_t wall_model_reset_count = 0;
    std::size_t wall_model_reset_due_to_frame_change_count = 0;
    std::size_t post_turn_verification_count = 0;
    std::size_t post_turn_verification_failure_count = 0;
    std::size_t stable_samples = 0;
    std::size_t map_commits = 0;
    std::uint64_t map_revision = 0;
    std::size_t map_cells = 0;
    std::size_t left_wall_observed_cells = 0;
    std::size_t collision_count = 0;
    std::size_t map_writes_while_moving = 0;
    std::size_t map_writes_during_turn_or_verify = 0;
    std::size_t core_wait_ticks = 0;
    bool core_ready_observed = false;
    bool command_speed_used_as_odometry = false;
    bool ground_truth_used_by_algorithm = false;
    bool command_target_used_as_odometry = false;
    std::size_t wall_fit_input_points = 0;
    std::size_t wall_fit_inliers = 0;
    double wall_fit_baseline_m = 0.0;
    double wall_fit_rms_m = 0.0;
    double initial_heading_error_deg = 0.0;
    double final_heading_error_deg = 0.0;
    double initial_distance_error_mm = 0.0;
    double final_distance_error_mm = 0.0;
    double maximum_abs_heading_error_deg = 0.0;
    double maximum_abs_distance_error_mm = 0.0;
    RobotPose2D final_estimated_pose;
    std::uint64_t frame_transform_epoch = 0;
    std::uint64_t wall_point_sequence_hash = 1469598103934665603ULL;
    std::uint64_t wall_model_sequence_hash = 1469598103934665603ULL;
    std::uint64_t final_map_checksum = 0;
    std::string termination_reason = "not_started";
    std::string log_error;
    std::vector<std::string> state_trace;
    std::vector<std::uint64_t> command_ids;
    std::vector<StopGoReplayRecord> replay_records;
    // P5 single-corner audit.  Angles are radians internally; formal output
    // converts them to degrees at the runner boundary.
    std::size_t completed_forward_steps_before_corner = 0;
    std::size_t completed_forward_steps_after_corner = 0;
    std::size_t corner_candidate_count = 0;
    std::size_t corner_confirmation_count = 0;
    std::size_t corner_confirmation_reject_count = 0;
    std::size_t corner_right_clearance_check_count = 0;
    std::size_t corner_right_clearance_reject_count = 0;
    std::size_t corner_main_turn_command_count = 0;
    std::size_t corner_right_turn_command_count = 0;
    std::size_t corner_left_turn_command_count = 0;
    std::uint64_t corner_main_command_id = 0;
    double pre_turn_odom_yaw_rad = 0.0;
    double post_main_turn_odom_yaw_rad = 0.0;
    double verified_corner_turn_delta_rad = 0.0;
    double final_corner_turn_error_rad = 0.0;
    std::size_t corner_residual_correction_count = 0;
    std::size_t corner_residual_right_count = 0;
    std::size_t corner_residual_left_count = 0;
    std::size_t post_turn_sensor_verify_count = 0;
    std::uint64_t old_wall_segment_id = 1;
    std::uint64_t new_wall_segment_id = 1;
    std::size_t wall_model_reset_due_to_corner_count = 0;
    std::size_t new_wall_reacquisition_samples = 0;
    std::size_t new_wall_acquisition_steps = 0;
    bool new_wall_model_valid = false;
    std::size_t post_corner_follow_steps = 0;
    std::uint64_t map_revision_before_corner = 0;
    std::uint64_t map_revision_after_corner = 0;
    std::size_t map_writes_during_corner_confirm = 0;
    std::size_t map_writes_during_corner_turn = 0;
    std::size_t map_writes_during_turn_verification = 0;
    std::uint64_t wall_segment_id = 1;
    std::uint64_t corner_transition_id = 0;
    std::uint64_t old_wall_point_hash = 1469598103934665603ULL;
    std::uint64_t new_wall_point_hash = 1469598103934665603ULL;
    std::uint64_t old_wall_model_hash = 1469598103934665603ULL;
    std::uint64_t new_wall_model_hash = 1469598103934665603ULL;
    std::uint64_t corner_event_sequence_hash = 1469598103934665603ULL;
    std::uint64_t corner_command_sequence_hash = 1469598103934665603ULL;
    std::string corner_failure_reason;
    bool rectangle_mode = false;
    RectangleRunStartAnchor run_start_anchor;
    bool corner_detector_armed = false;
    std::size_t corner_rearm_count = 0;
    std::size_t forward_steps_since_last_corner = 0;
    double odom_distance_since_last_corner_m = 0.0;
    std::vector<std::size_t> forward_steps_per_segment;
    std::vector<double> odom_distance_per_segment_m;
    std::vector<std::uint64_t> wall_segment_sequence{1};
    std::vector<std::uint64_t> corner_transition_ids;
    std::size_t corner_transition_count = 0;
    double total_odom_travel_distance_m = 0.0;
    double segment_odom_distance_m = 0.0;
    std::size_t odom_distance_jump_reject_count = 0;
    std::string odom_distance_rejection_reason = "none";
    std::size_t closure_candidate_count = 0;
    std::size_t first_closure_candidate_transition_count = 0;
    std::size_t closure_confirmation_attempt_count = 0;
    std::size_t closure_confirmation_pass_count = 0;
    std::size_t closure_confirmation_reject_count = 0;
    std::size_t fourth_corner_follow_steps = 0;
    double estimated_closure_position_error_m = 0.0;
    double estimated_closure_yaw_error_rad = 0.0;
    double initial_wall_heading_error_rad = 0.0;
    double wall_distance_revisit_error_m = 0.0;
    double wall_line_offset_revisit_error_m = 0.0;
    bool forced_pose_snap_used = false;
    bool rectangle_geometry_snap_used = false;
    std::size_t controller_map_odom_write_attempt_count = 0;
    std::size_t map_writes_during_closure_confirm = 0;
    bool final_settle_complete = false;
    std::size_t final_mapping_commit_count = 0;
    std::size_t commands_submitted_after_completion = 0;
    std::size_t frame_epoch_recovery_count = 0;
    bool frame_epoch_change_injected = false;
    std::uint64_t injected_frame_epoch_before = 0;
    std::uint64_t injected_frame_epoch_after = 0;
    bool final_map_saved = false;
    bool final_map_reload_verified = false;
    std::string final_map_path;
    std::string run_summary_path;
    std::string map_save_failure_reason;
    std::size_t map_occupied_cell_count = 0;
    std::size_t map_free_cell_count = 0;
    std::size_t map_unknown_cell_count = 0;
    std::size_t map_uncertain_cell_count = 0;
    bool map_has_bounds = false;
    std::int32_t map_min_x_cell = 0;
    std::int32_t map_max_x_cell = 0;
    std::int32_t map_min_y_cell = 0;
    std::int32_t map_max_y_cell = 0;
    double observable_wall_coverage_ratio = 0.0;
    double p95_wall_thickness_cells = 0.0;
    double maximum_wall_thickness_cells = 0.0;
    double ghost_occupied_cell_ratio = 0.0;
    std::size_t duplicate_wall_band_count = 0;
    std::size_t collision_count_from_evaluator = 0;
    double ground_truth_final_position_error_m = 0.0;
    double ground_truth_final_yaw_error_rad = 0.0;
    bool ground_truth_used_by_controller = false;
    bool ground_truth_used_by_core = false;
    bool ground_truth_used_only_by_acceptance_evaluator = true;
    std::uint64_t final_map_reload_checksum = 0;
    std::uint64_t run_anchor_hash = 1469598103934665603ULL;
    std::uint64_t closure_sequence_hash = 1469598103934665603ULL;
};

class StopGoMappingController final {
public:
    using SnapshotReader = std::function<RobotSlamSensorSnapshot(double)>;

    StopGoMappingController(RobotSlamApplication &application,
                            RelativeMotionStepPort &motion,
                            SnapshotReader snapshot_reader,
                            StableThreeTofSampler sampler,
                            StopGoMappingControllerConfig config,
                            std::string log_path = {})
        : application_(application), motion_(motion),
          snapshot_reader_(std::move(snapshot_reader)), sampler_(std::move(sampler)),
          config_(std::move(config)), gate_(config_.settle),
          wall_estimator_(config_.wall_estimator) {
        if (!log_path.empty()) config_.log_path = std::move(log_path);
        desired_distance_m_ = config_.desired_base_to_wall_distance_m;
        report_.rectangle_mode = config_.mode == "rectangle_loop";
        if (!config_.log_path.empty()) log_.open(config_.log_path, std::ios::out | std::ios::trunc);
        // Core startup owns gyro calibration, wheel baseline, odom_T_base,
        // map_T_odom and the pose buffer.  Stop-go cannot enter its formal
        // state machine until Core explicitly reports localization_ready().
        transition(StopGoMappingControllerState::WaitingForCoreReady);
    }

    bool tick(double now_s) {
        if (single_corner_mode() || rectangle_mode()) return tick_single_corner(now_s);
        if (left_wall_mode()) return tick_left_wall(now_s);
        if (terminal()) return report_.ok;
        if (!snapshot_reader_ || !motion_.ready()) return fail("controller_port_or_sensor_not_ready", now_s);
        const auto snapshot = snapshot_reader_(now_s);

        if (state_ == StopGoMappingControllerState::WaitingForCoreReady) {
            report_.core_wait_ticks++;
            // A missing/invalid startup sample is a reason to keep waiting for
            // Core's stationary calibration, not a stop-go motion fault.
            if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
                return true;
            }

            const auto odom_only = make_odom_only(snapshot);
            const auto app_step = application_.step(odom_only, now_s);
            if (application_.core().localization_ready()) {
                report_.core_ready_observed = true;
                transition(StopGoMappingControllerState::InitialSettle);
                return true;
            }

            const auto &core_report = application_.core().report();
            if (core_report.initialization_status == "fault" ||
                core_report.initialization_status == "gyro_calibration_failed" ||
                core_report.initialization_status == "wheel_baseline_failed" ||
                core_report.last_fault != "none") {
                return fail("core_initialization_failed:" +
                                (app_step.reason.empty()
                                     ? core_report.last_message
                                     : app_step.reason),
                            now_s);
            }
            // During normal startup Core returns ok=false while collecting
            // stationary samples.  That is an expected waiting state.
            return true;
        }

        if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
            return fail("canonical_wheel_or_imu_feedback_invalid", now_s);
        }
        const auto odom_only = make_odom_only(snapshot);
        // A stable ToF commit is itself the canonical sample for this stop.
        // Do not first consume the same wheel timestamp as an odometry-only
        // sample; the core deliberately rejects non-monotonic feedback.
        const auto revision_before_odom_step =
            application_.core().report().current_map_revision;
        if (state_ != StopGoMappingControllerState::InitialSample &&
            state_ != StopGoMappingControllerState::AcquireThreeTof) {
            const auto app_step = application_.step(odom_only, now_s);
            if (!app_step.ok || !app_step.core_step_executed) {
                return fail("application_core_step_failed:" + app_step.reason, now_s);
            }
            const auto revision_after_odom_step =
                application_.core().report().current_map_revision;
            if ((state_ == StopGoMappingControllerState::WaitMotionDone ||
                 state_ == StopGoMappingControllerState::WaitMappingSettle) &&
                revision_after_odom_step != revision_before_odom_step) {
                report_.map_writes_while_moving++;
            }
        }

        if (state_ == StopGoMappingControllerState::InitialSettle) {
            RelativeMotionStepResult initial;
            initial.state = RelativeMotionStepState::Done;
            const auto settled = gate_.update(initial, odom_only,
                                              monotonic_us(now_s), true);
            if (settled.stable) transition(StopGoMappingControllerState::InitialSample);
            return true;
        }
        if (state_ == StopGoMappingControllerState::InitialSample) {
            transition(StopGoMappingControllerState::AcquireThreeTof);
            return acquire_and_commit(snapshot, now_s, 0);
        }
        if (state_ == StopGoMappingControllerState::IssueForwardStep) {
            if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
                total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
                return complete("max_steps_or_distance_reached", now_s);
            }
            command_ = {};
            command_.command_id = next_command_id_++;
            command_.action = RelativeMotionStepAction::Forward;
            command_.target_amount = config_.forward_step_mm;
            command_.max_rpm = config_.motion_max_rpm;
            command_.timeout_ms = config_.motion_timeout_ms;
            const auto accepted = motion_.submit(command_, now_s);
            if (accepted.state != RelativeMotionStepState::Accepted) {
                return fail("motion_submit_failed:" + accepted.reason, now_s);
            }
            report_.commands_submitted++;
            report_.command_ids.push_back(command_.command_id);
            transition(StopGoMappingControllerState::WaitMotionDone);
            return true;
        }
        if (state_ == StopGoMappingControllerState::WaitMotionDone) {
            const auto result = motion_.poll(now_s);
            last_motion_ = result;
            if (result.state == RelativeMotionStepState::Fault ||
                result.state == RelativeMotionStepState::Timeout ||
                result.state == RelativeMotionStepState::Cancelled ||
                result.state == RelativeMotionStepState::EstopLatched) {
                return fail("motion_failed:" + result.reason, now_s);
            }
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitMappingSettle);
            }
            return true;
        }
        if (state_ == StopGoMappingControllerState::WaitMappingSettle) {
            last_motion_ = motion_.poll(now_s);
            const auto settled = gate_.update(last_motion_, odom_only,
                                              monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::AcquireThreeTof);
            return true;
        }
        if (state_ == StopGoMappingControllerState::AcquireThreeTof) {
            return acquire_and_commit(snapshot, now_s, report_.completed_steps + 1);
        }
        return true;
    }

    bool request_stop(double now_s) {
        const auto result = motion_.stop(now_s);
        if (state_ == StopGoMappingControllerState::WaitMotionDone ||
            state_ == StopGoMappingControllerState::WaitMappingSettle ||
            state_ == StopGoMappingControllerState::WaitTurnDone ||
            state_ == StopGoMappingControllerState::WaitTurnSettle ||
            state_ == StopGoMappingControllerState::VerifyAfterTurn ||
            state_ == StopGoMappingControllerState::WaitCornerTurnDone ||
            state_ == StopGoMappingControllerState::WaitCornerTurnSettle ||
            state_ == StopGoMappingControllerState::VerifyCornerTurn ||
            state_ == StopGoMappingControllerState::WaitCornerResidualDone ||
            state_ == StopGoMappingControllerState::WaitCornerResidualSettle ||
            state_ == StopGoMappingControllerState::PostCornerTurnSensorVerify) {
            last_motion_ = result;
            transition(StopGoMappingControllerState::Fault);
            report_.termination_reason = "cancelled_by_stop";
        }
        return result.state == RelativeMotionStepState::Done ||
               result.state == RelativeMotionStepState::Cancelled;
    }

    bool request_estop(double now_s) {
        last_motion_ = motion_.emergency_stop(now_s);
        transition(StopGoMappingControllerState::Estop);
        report_.termination_reason = "estop_latched";
        return true;
    }

    bool terminal() const {
        return state_ == StopGoMappingControllerState::Completed ||
               state_ == StopGoMappingControllerState::SingleCornerComplete ||
               state_ == StopGoMappingControllerState::WallLost ||
               state_ == StopGoMappingControllerState::FrontBlocked ||
               state_ == StopGoMappingControllerState::TurnClearanceBlocked ||
               state_ == StopGoMappingControllerState::CornerTurnVerificationFailed ||
               state_ == StopGoMappingControllerState::PostTurnFrontBlocked ||
               state_ == StopGoMappingControllerState::NewLeftWallReacquisitionFailed ||
               state_ == StopGoMappingControllerState::Fault ||
               state_ == StopGoMappingControllerState::Estop;
    }
    StopGoMappingControllerState state() const { return state_; }
    const StopGoMappingRunReport &report() const { return report_; }

private:
    bool left_wall_mode() const { return config_.mode == "left_wall_follow"; }
    bool single_corner_mode() const { return config_.mode == "single_corner"; }
    bool rectangle_mode() const { return config_.mode == "rectangle_loop"; }

    static double minimum_corner_trigger_distance_m(
        const StopGoMappingControllerConfig &config) {
        return std::max(0.0, config.robot_turning_sweep_radius_m +
            config.turn_clearance_margin_m - config.front_tof_forward_offset_m) +
            config.forward_step_mm / 1000.0 + config.expected_forward_overrun_m;
    }

    bool corner_safety_config_valid() const {
        const bool transition_config_valid = single_corner_mode()
            ? config_.maximum_corner_transitions == 1
            : rectangle_mode() && config_.rectangle_target_corner_transitions == 4 &&
              config_.rectangle_maximum_corner_transitions == 4;
        return transition_config_valid &&
               config_.corner_confirmation_resamples > 0 &&
               config_.corner_confirmation_required_hits > 0 &&
               config_.corner_confirmation_required_hits <= config_.corner_confirmation_resamples &&
               std::isfinite(config_.main_turn_deg) &&
               std::fabs(config_.main_turn_deg - 90.0) < 1e-9 &&
               config_.right_clearance_resample_attempts > 0 &&
               config_.corner_turn_max_residual_corrections > 0 &&
               config_.post_turn_sensor_resample_attempts > 0 &&
               config_.new_wall_stationary_resample_attempts > 0 &&
               config_.new_wall_acquisition_max_forward_steps > 0 &&
               config_.post_corner_required_follow_steps > 0 &&
               config_.new_wall_min_distance_m <= config_.new_wall_max_distance_m &&
               config_.corner_trigger_distance_m + 1e-9 >=
                   minimum_corner_trigger_distance_m(config_) &&
               (!rectangle_mode() ||
                (config_.rectangle_closure_confirmation_samples > 0 &&
                 config_.rectangle_closure_confirmation_required_passes > 0 &&
                 config_.rectangle_closure_confirmation_required_passes <=
                     config_.rectangle_closure_confirmation_samples &&
                 config_.rectangle_closure_confirmation_samples <=
                     config_.rectangle_closure_confirmation_max_attempts &&
                 config_.rectangle_closure_confirmation_required_passes <=
                     config_.rectangle_closure_confirmation_max_attempts &&
                 config_.rectangle_maximum_post_fourth_corner_forward_steps > 0 &&
                 config_.rectangle_maximum_total_forward_steps > 0 &&
                 std::isfinite(config_.rectangle_maximum_runtime_s) &&
                 config_.rectangle_maximum_runtime_s > 0.0));
    }

    bool tick_single_corner(double now_s) {
        if (terminal()) return report_.ok;
        if (!corner_safety_config_valid()) return fail("invalid_single_corner_config", now_s);
        if (!snapshot_reader_ || !motion_.ready() || !sampler_.ready()) {
            return fail("single_corner_port_or_sensor_not_ready", now_s);
        }
        const auto snapshot = snapshot_reader_(now_s);
        const auto &core = application_.core();
        if (rectangle_mode() && !update_rectangle_odom_distance()) {
            return fail("rectangle_odom_jump_rejected", now_s);
        }

        if (state_ == StopGoMappingControllerState::WaitingForCoreReady) {
            report_.core_wait_ticks++;
            if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) return true;
            const auto stepped = application_.step(make_odom_only(snapshot), now_s);
            if (core.localization_ready()) {
                report_.core_ready_observed = true;
                gate_.reset();
                transition(StopGoMappingControllerState::InitialSettle);
                return true;
            }
            const auto &cr = core.report();
            if (!core.relocalization_required() && !cr.relocalization_active &&
                !cr.localization_stop_required &&
                (cr.initialization_status == "fault" ||
                 cr.initialization_status == "gyro_calibration_failed" ||
                 cr.initialization_status == "wheel_baseline_failed" ||
                 cr.last_fault != "none")) {
                return fail("core_initialization_failed:" +
                    (stepped.reason.empty() ? cr.last_message : stepped.reason), now_s);
            }
            return true;
        }

        if (!core.localization_ready() ||
            core.localization_health_state() == LocalizationHealthState::Lost ||
            core.report().localization_stop_required || core.relocalization_required()) {
            (void)motion_.stop(now_s);
            clear_wall_window("core_not_ready_or_recovering", true);
            reset_corner_flow_for_recovery();
            transition(StopGoMappingControllerState::WaitingForCoreReady);
            if (snapshot.has_wheel && snapshot.has_imu && snapshot.wheel.valid) {
                (void)application_.step(make_odom_only(snapshot), now_s);
            }
            return true;
        }
        if (wall_epoch_ != 0 && core.frame_transform_epoch() != wall_epoch_) {
            (void)motion_.stop(now_s);
            clear_wall_window("frame_transform_epoch_changed_during_corner", true);
            ++report_.frame_epoch_recovery_count;
            reset_corner_flow_for_recovery();
            transition(StopGoMappingControllerState::WaitingForCoreReady);
            return true;
        }
        if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
            return fail("canonical_wheel_or_imu_feedback_invalid", now_s);
        }

        const bool mapping_sample_state =
            state_ == StopGoMappingControllerState::InitialSample ||
            state_ == StopGoMappingControllerState::WallAcquisition ||
            state_ == StopGoMappingControllerState::AcquireThreeTof ||
            state_ == StopGoMappingControllerState::CornerCandidate ||
            state_ == StopGoMappingControllerState::CornerConfirming ||
            state_ == StopGoMappingControllerState::PostCornerTurnSensorVerify ||
            state_ == StopGoMappingControllerState::NewLeftWallReacquisition ||
            state_ == StopGoMappingControllerState::VerifyAfterTurn ||
            (state_ == StopGoMappingControllerState::RectangleClosureConfirming &&
             closure_gate_complete_) ||
            state_ == StopGoMappingControllerState::FinalMappingCommit;
        if (!mapping_sample_state) {
            const auto before = core.report().current_map_revision;
            const auto stepped = application_.step(make_odom_only(snapshot), now_s);
            if (!stepped.ok || !stepped.core_step_executed) {
                return fail("application_core_step_failed:" + stepped.reason, now_s);
            }
            const auto after = core.report().current_map_revision;
            if (state_ == StopGoMappingControllerState::CornerConfirming ||
                state_ == StopGoMappingControllerState::CornerCandidate) {
                if (after != before) report_.map_writes_during_corner_confirm++;
            } else if (state_ == StopGoMappingControllerState::WaitCornerTurnDone ||
                       state_ == StopGoMappingControllerState::WaitCornerTurnSettle ||
                       state_ == StopGoMappingControllerState::WaitCornerResidualDone ||
                       state_ == StopGoMappingControllerState::WaitCornerResidualSettle) {
                if (after != before) report_.map_writes_during_corner_turn++;
            } else if (state_ == StopGoMappingControllerState::VerifyCornerTurn) {
                if (after != before) report_.map_writes_during_turn_verification++;
            }
            pending_replay_odom_samples_.push_back({snapshot.wheel, snapshot.imu});
        }

        switch (state_) {
        case StopGoMappingControllerState::InitialSettle: {
            RelativeMotionStepResult initial;
            initial.state = RelativeMotionStepState::Done;
            const auto settled = gate_.update(initial, make_odom_only(snapshot),
                                              monotonic_us(now_s), true);
            if (settled.stable) transition(StopGoMappingControllerState::InitialSample);
            return true;
        }
        case StopGoMappingControllerState::InitialSample:
            transition(StopGoMappingControllerState::WallAcquisition);
            return acquire_single_corner_sample(snapshot, now_s, 0);
        case StopGoMappingControllerState::WallAcquisition:
        case StopGoMappingControllerState::AcquireThreeTof:
            return acquire_single_corner_sample(snapshot, now_s, report_.completed_steps + 1);
        case StopGoMappingControllerState::IssueForwardStep:
            return issue_single_corner_forward(now_s);
        case StopGoMappingControllerState::WaitMotionDone: {
            const auto result = motion_.poll(now_s);
            if (!motion_result_matches_command(result)) return fail("forward_command_id_mismatch", now_s);
            last_motion_ = result;
            if (motion_failed(result)) return fail("motion_failed:" + result.reason, now_s);
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitMappingSettle);
            }
            return true;
        }
        case StopGoMappingControllerState::WaitMappingSettle: {
            last_motion_ = motion_.poll(now_s);
            if (!motion_result_matches_command(last_motion_)) return fail("forward_settle_command_id_mismatch", now_s);
            const auto settled = gate_.update(last_motion_, make_odom_only(snapshot), monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::AcquireThreeTof);
            return true;
        }
        case StopGoMappingControllerState::CornerCandidate:
        case StopGoMappingControllerState::CornerConfirming:
            return confirm_corner(snapshot, now_s);
        case StopGoMappingControllerState::IssueCornerRightTurn:
            return issue_corner_right_turn(now_s);
        case StopGoMappingControllerState::WaitCornerTurnDone:
            return poll_corner_turn(now_s, false);
        case StopGoMappingControllerState::WaitCornerTurnSettle:
            return settle_corner_turn(snapshot, now_s, false);
        case StopGoMappingControllerState::VerifyCornerTurn:
            return verify_corner_turn(now_s);
        case StopGoMappingControllerState::IssueCornerResidualCorrection:
            return issue_corner_residual_correction(now_s);
        case StopGoMappingControllerState::WaitCornerResidualDone:
            return poll_corner_turn(now_s, true);
        case StopGoMappingControllerState::WaitCornerResidualSettle:
            return settle_corner_turn(snapshot, now_s, true);
        case StopGoMappingControllerState::PostCornerTurnSensorVerify:
            return post_corner_sensor_verify(snapshot, now_s);
        case StopGoMappingControllerState::NewLeftWallReacquisition:
            return acquire_new_wall_sample(snapshot, now_s);
        case StopGoMappingControllerState::IssueTurnCorrection:
            return issue_turn_correction(now_s);
        case StopGoMappingControllerState::WaitTurnDone: {
            const auto result = motion_.poll(now_s);
            if (!motion_result_matches_command(result)) return fail("turn_command_id_mismatch", now_s);
            last_motion_ = result;
            if (motion_failed(result)) return fail("turn_motion_failed:" + result.reason, now_s);
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitTurnSettle);
            }
            return true;
        }
        case StopGoMappingControllerState::WaitTurnSettle: {
            last_motion_ = motion_.poll(now_s);
            if (!motion_result_matches_command(last_motion_)) return fail("turn_settle_command_id_mismatch", now_s);
            const auto settled = gate_.update(last_motion_, make_odom_only(snapshot), monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::VerifyAfterTurn);
            return true;
        }
        case StopGoMappingControllerState::VerifyAfterTurn:
            return verify_after_turn(snapshot, now_s);
        case StopGoMappingControllerState::CheckLimit:
            return check_single_corner_limit(now_s);
        case StopGoMappingControllerState::RectangleClosureCandidate:
            return begin_rectangle_closure_confirmation(now_s);
        case StopGoMappingControllerState::RectangleClosureConfirming:
            return confirm_rectangle_closure(snapshot, now_s);
        case StopGoMappingControllerState::FinalSettle:
            return settle_rectangle_final(snapshot, now_s);
        case StopGoMappingControllerState::FinalMappingCommit:
            return commit_rectangle_final_sample(snapshot, now_s);
        case StopGoMappingControllerState::Completed:
        case StopGoMappingControllerState::SingleCornerComplete:
        case StopGoMappingControllerState::WallLost:
        case StopGoMappingControllerState::FrontBlocked:
        case StopGoMappingControllerState::TurnClearanceBlocked:
        case StopGoMappingControllerState::CornerTurnVerificationFailed:
        case StopGoMappingControllerState::PostTurnFrontBlocked:
        case StopGoMappingControllerState::NewLeftWallReacquisitionFailed:
        case StopGoMappingControllerState::Fault:
        case StopGoMappingControllerState::Estop:
        case StopGoMappingControllerState::Idle:
        case StopGoMappingControllerState::WaitingForCoreReady:
        case StopGoMappingControllerState::CommitMappingSample:
            return true;
        }
        return true;
    }

    void reset_corner_flow_for_recovery() {
        corner_candidate_pending_ = false;
        corner_confirmed_ = false;
        main_turn_finished_ = false;
        post_turn_sensor_verified_ = false;
        new_wall_mode_ = false;
        corner_confirmation_attempts_ = 0;
        corner_confirmation_hits_ = 0;
        right_clearance_attempts_ = 0;
        post_turn_sensor_attempts_ = 0;
        new_wall_resample_attempts_ = 0;
        new_wall_acquisition_steps_ = 0;
        corner_residual_corrections_ = 0;
        current_corner_main_turn_command_count_ = 0;
        current_corner_follow_steps_ = 0;
        corner_detector_armed_ = false;
        post_fourth_search_active_ = false;
        corner_main_command_id_ = 0;
        corner_residual_command_id_ = 0;
        pre_turn_odom_yaw_rad_ = 0.0;
        post_main_odom_yaw_rad_ = 0.0;
        verified_turn_delta_rad_ = 0.0;
        final_turn_error_rad_ = 0.0;
    }

    bool issue_single_corner_forward(double now_s) {
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return fail("corner_not_reached_before_motion_limit", now_s);
        }
        command_ = {};
        command_.command_id = next_command_id_++;
        command_.action = RelativeMotionStepAction::Forward;
        command_.target_amount = config_.forward_step_mm;
        command_.max_rpm = config_.motion_max_rpm;
        command_.timeout_ms = config_.motion_timeout_ms;
        const auto accepted = motion_.submit(command_, now_s);
        if (accepted.state != RelativeMotionStepState::Accepted) {
            return fail("motion_submit_failed:" + accepted.reason, now_s);
        }
        report_.commands_submitted++;
        report_.forward_command_count++;
        report_.command_ids.push_back(command_.command_id);
        if (new_wall_mode_ && !wall_model_.valid) {
            ++new_wall_acquisition_steps_;
            report_.new_wall_acquisition_steps = new_wall_acquisition_steps_;
        } else if (!wall_model_.valid) {
            report_.wall_acquisition_steps++;
        }
        transition(StopGoMappingControllerState::WaitMotionDone);
        return true;
    }

    bool stable_front_is_corner_hit(const StableTofReading &reading) const {
        return reading.valid && reading.usable_for_mapping &&
               reading.confidence >= config_.tof.mapping_min_confidence &&
               static_cast<double>(reading.distance_mm) <=
                   config_.corner_trigger_distance_m * 1000.0 + 1e-9;
    }

    bool core_accepts_no_map_snapshot(const RobotSlamSensorSnapshot &base,
                                      const StableThreeTofSample &stable,
                                      double now_s, const char *failure_reason) {
        const auto full = make_stable_snapshot(base, stable);
        const auto before = application_.core().report().current_map_revision;
        const auto poses_before = application_.core().report().measurement_pose_set_success_count;
        const auto stepped = application_.step(full, now_s, {},
            ActiveObservationControl::None, false);
        const auto after = application_.core().report().current_map_revision;
        if (!stepped.ok || after != before ||
            application_.core().report().measurement_pose_set_success_count <= poses_before ||
            !application_.core().localization_ready() ||
            application_.core().frame_transform_epoch() != wall_epoch_) {
            if (after != before) {
                if (state_ == StopGoMappingControllerState::CornerCandidate ||
                    state_ == StopGoMappingControllerState::CornerConfirming) {
                    report_.map_writes_during_corner_confirm++;
                } else if (state_ ==
                           StopGoMappingControllerState::RectangleClosureConfirming) {
                    report_.map_writes_during_closure_confirm++;
                } else {
                    report_.map_writes_during_turn_verification++;
                }
            }
            report_.corner_failure_reason = std::string(failure_reason) + ":" +
                (stepped.reason.empty() ? application_.core().report().last_message : stepped.reason);
            return fail(failure_reason, now_s);
        }
        pending_replay_odom_samples_.push_back({full.wheel, full.imu});
        return true;
    }

    bool confirm_corner(const RobotSlamSensorSnapshot &base, double now_s) {
        if (!wall_model_.valid || wall_model_.frame_transform_epoch !=
                application_.core().frame_transform_epoch()) {
            return fail("corner_confirmation_without_valid_wall_model", now_s);
        }
        transition(StopGoMappingControllerState::CornerConfirming);
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        last_stable_front_ = stable.front;
        ++corner_confirmation_attempts_;
        report_.corner_confirmation_count++;
        const bool front_hit = stable_front_is_corner_hit(stable.front);
        const bool left_hit = stable.left.valid && stable.left.usable_for_mapping;
        if (front_hit && left_hit) {
            if (!core_accepts_no_map_snapshot(base, stable, now_s,
                                              "corner_confirmation_core_rejected")) return false;
            ++corner_confirmation_hits_;
            pending_replay_corner_confirmation_ = true;
            if (corner_confirmation_hits_ >= config_.corner_confirmation_required_hits) {
                corner_confirmed_ = true;
                report_.map_revision_before_corner = application_.core().report().current_map_revision;
                report_.old_wall_segment_id = wall_segment_id_;
                pre_turn_odom_yaw_rad_ = application_.core().report().last_odom_pose.yaw_rad;
                pre_turn_map_yaw_rad_ = application_.core().current_map_pose().map_T_base.yaw_rad;
                report_.pre_turn_odom_yaw_rad = pre_turn_odom_yaw_rad_;
                transition(StopGoMappingControllerState::IssueCornerRightTurn);
                return true;
            }
        }
        if (corner_confirmation_attempts_ >= config_.corner_confirmation_resamples) {
            ++report_.corner_confirmation_reject_count;
            corner_candidate_pending_ = false;
            corner_confirmation_attempts_ = 0;
            corner_confirmation_hits_ = 0;
            if (!stable.front.usable_for_mapping || !stable.left.usable_for_mapping) {
                if (new_wall_mode_) return fail_new_wall("corner_confirmation_sensor_invalid", now_s);
                return front_blocked("corner_confirmation_sensor_invalid", now_s);
            }
            transition(StopGoMappingControllerState::IssueForwardStep);
        }
        return true;
    }

    bool issue_corner_right_turn(double now_s) {
        if (!corner_confirmed_ || current_corner_main_turn_command_count_ != 0 ||
            (!rectangle_mode() && report_.corner_right_turn_command_count != 0)) {
            return fail("invalid_corner_turn_lifecycle", now_s);
        }
        right_clearance_attempts_++;
        report_.corner_right_clearance_check_count++;
        // The actual stable right sample is checked in confirm_corner's next
        // pass.  This state is intentionally a separate gate: no direction
        // decision is derived from it, and RIGHT remains fixed.
        if (!right_clearance_passed_) {
            const auto stable = sampler_.acquire();
            report_.stable_samples++;
            if (!stable.right.usable_for_mapping) {
                if (right_clearance_attempts_ < config_.right_clearance_resample_attempts) return true;
                ++report_.corner_right_clearance_reject_count;
                return turn_clearance_blocked("right_clearance_invalid", now_s);
            }
            if (static_cast<double>(stable.right.distance_mm) <
                    config_.minimum_right_turn_clearance_m * 1000.0 - 1e-9) {
                ++report_.corner_right_clearance_reject_count;
                return turn_clearance_blocked("right_turn_clearance_too_small", now_s);
            }
            right_clearance_passed_ = true;
            pending_replay_corner_clearance_ = true;
        }
        command_ = {};
        command_.command_id = next_command_id_++;
        command_.action = RelativeMotionStepAction::Right;
        command_.target_amount = config_.main_turn_deg;
        command_.max_rpm = config_.motion_max_rpm;
        command_.timeout_ms = config_.motion_timeout_ms;
        const auto accepted = motion_.submit(command_, now_s);
        if (accepted.state != RelativeMotionStepState::Accepted) {
            return fail("corner_right_turn_submit_failed:" + accepted.reason, now_s);
        }
        report_.commands_submitted++;
        report_.corner_main_turn_command_count++;
        report_.corner_right_turn_command_count++;
        current_corner_main_turn_command_count_++;
        report_.corner_main_command_id = command_.command_id;
        corner_main_command_id_ = command_.command_id;
        report_.command_ids.push_back(command_.command_id);
        hash_mix(report_.corner_command_sequence_hash, command_.command_id);
        hash_mix(report_.corner_command_sequence_hash, static_cast<std::uint64_t>(command_.action));
        hash_mix(report_.corner_command_sequence_hash, quantized(command_.target_amount));
        pending_replay_corner_main_turn_ = true;
        transition(StopGoMappingControllerState::WaitCornerTurnDone);
        return true;
    }

    bool poll_corner_turn(double now_s, bool residual) {
        const auto result = motion_.poll(now_s);
        if (!motion_result_matches_command(result)) {
            return fail(residual ? "corner_residual_command_id_mismatch" : "corner_turn_command_id_mismatch", now_s);
        }
        last_motion_ = result;
        if (motion_failed(result)) {
            return fail(residual ? "corner_residual_motion_failed:" + result.reason
                                 : "corner_turn_motion_failed:" + result.reason, now_s);
        }
        if (result.state == RelativeMotionStepState::Done) {
            gate_.reset();
            transition(residual ? StopGoMappingControllerState::WaitCornerResidualSettle
                                : StopGoMappingControllerState::WaitCornerTurnSettle);
        }
        return true;
    }

    bool settle_corner_turn(const RobotSlamSensorSnapshot &snapshot,
                            double now_s, bool residual) {
        last_motion_ = motion_.poll(now_s);
        if (!motion_result_matches_command(last_motion_)) {
            return fail(residual ? "corner_residual_settle_command_id_mismatch"
                                 : "corner_turn_settle_command_id_mismatch", now_s);
        }
        const auto settled = gate_.update(last_motion_, make_odom_only(snapshot), monotonic_us(now_s));
        if (settled.stable) {
            transition(residual ? StopGoMappingControllerState::VerifyCornerTurn
                                : StopGoMappingControllerState::VerifyCornerTurn);
        }
        return true;
    }

    bool verify_corner_turn(double now_s) {
        if (application_.core().frame_transform_epoch() != wall_epoch_) {
            return fail("frame_transform_epoch_changed_during_corner_verification", now_s);
        }
        const double post = application_.core().report().last_odom_pose.yaw_rad;
        if (!std::isfinite(post) || !std::isfinite(pre_turn_odom_yaw_rad_)) {
            return corner_turn_verification_failed("corner_odom_yaw_invalid", now_s);
        }
        post_main_odom_yaw_rad_ = post;
        const double actual = sparse_slam_shortest_yaw_delta(pre_turn_odom_yaw_rad_, post);
        const double target = -3.14159265358979323846 / 2.0;
        const double residual = sparse_slam_normalize_yaw(target - actual);
        verified_turn_delta_rad_ = actual;
        final_turn_error_rad_ = residual;
        if (corner_residual_corrections_ == 0) {
            report_.post_main_turn_odom_yaw_rad = post;
        }
        report_.verified_corner_turn_delta_rad = actual;
        report_.final_corner_turn_error_rad = residual;
        if (std::fabs(residual) <= config_.corner_turn_acceptance_tolerance_rad + 1e-9) {
            main_turn_finished_ = true;
            transition(StopGoMappingControllerState::PostCornerTurnSensorVerify);
            return true;
        }
        if (std::fabs(residual) > config_.corner_turn_max_residual_correction_rad + 1e-9 ||
            corner_residual_corrections_ >= config_.corner_turn_max_residual_corrections) {
            return corner_turn_verification_failed("corner_turn_residual_out_of_bounds", now_s);
        }
        pending_corner_residual_rad_ = residual;
        transition(StopGoMappingControllerState::IssueCornerResidualCorrection);
        return true;
    }

    bool issue_corner_residual_correction(double now_s) {
        const double residual_deg = std::fabs(pending_corner_residual_rad_) *
            180.0 / 3.14159265358979323846;
        const double amount = std::clamp(residual_deg,
            config_.min_corner_residual_correction_deg,
            config_.corner_turn_max_residual_correction_rad *
                180.0 / 3.14159265358979323846);
        command_ = {};
        command_.command_id = next_command_id_++;
        command_.action = pending_corner_residual_rad_ < 0.0
            ? RelativeMotionStepAction::Right : RelativeMotionStepAction::Left;
        command_.target_amount = amount;
        command_.max_rpm = config_.motion_max_rpm;
        command_.timeout_ms = config_.motion_timeout_ms;
        const auto accepted = motion_.submit(command_, now_s);
        if (accepted.state != RelativeMotionStepState::Accepted) {
            return fail("corner_residual_submit_failed:" + accepted.reason, now_s);
        }
        ++corner_residual_corrections_;
        ++report_.corner_residual_correction_count;
        if (command_.action == RelativeMotionStepAction::Right) {
            ++report_.corner_residual_right_count;
            ++report_.corner_right_turn_command_count;
        } else {
            ++report_.corner_residual_left_count;
            ++report_.corner_left_turn_command_count;
        }
        report_.commands_submitted++;
        report_.command_ids.push_back(command_.command_id);
        hash_mix(report_.corner_command_sequence_hash, command_.command_id);
        hash_mix(report_.corner_command_sequence_hash, static_cast<std::uint64_t>(command_.action));
        hash_mix(report_.corner_command_sequence_hash, quantized(command_.target_amount));
        pending_replay_corner_residual_ = true;
        pending_replay_corner_residual_right_ = command_.action == RelativeMotionStepAction::Right;
        corner_residual_command_id_ = command_.command_id;
        transition(StopGoMappingControllerState::WaitCornerResidualDone);
        return true;
    }

    bool post_corner_sensor_verify(const RobotSlamSensorSnapshot &base, double now_s) {
        ++post_turn_sensor_attempts_;
        ++report_.post_turn_sensor_verify_count;
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.front.usable_for_mapping ||
            stable.front.distance_mm <= config_.post_turn_front_stop_threshold_m * 1000.0 + 1e-9) {
            if (post_turn_sensor_attempts_ < config_.post_turn_sensor_resample_attempts) return true;
            return post_turn_front_blocked("post_turn_front_blocked", now_s);
        }
        if (!stable.left.usable_for_mapping) {
            if (post_turn_sensor_attempts_ < config_.post_turn_sensor_resample_attempts) return true;
            return begin_new_wall_reacquisition(now_s);
        }
        if (!stable.right.usable_for_mapping) {
            if (post_turn_sensor_attempts_ < config_.post_turn_sensor_resample_attempts) return true;
            return turn_clearance_blocked("post_turn_right_sample_invalid", now_s);
        }
        if (!core_accepts_no_map_snapshot(base, stable, now_s,
                                          "post_corner_sensor_core_rejected")) return false;
        post_turn_sensor_verified_ = true;
        pending_replay_post_corner_verify_ = true;
        return begin_new_wall_reacquisition(now_s);
    }

    bool begin_new_wall_reacquisition(double now_s) {
        (void)now_s;
        if (!main_turn_finished_ || current_corner_main_turn_command_count_ == 0) {
            return corner_turn_verification_failed("corner_turn_not_verified", now_s);
        }
        old_wall_points_hash_ = segment_wall_point_hash_;
        old_wall_models_hash_ = segment_wall_model_hash_;
        report_.old_wall_point_hash = old_wall_points_hash_;
        report_.old_wall_model_hash = old_wall_models_hash_;
        wall_points_.clear();
        wall_model_ = {};
        pending_decision_ = {};
        ++wall_segment_id_;
        ++corner_transition_id_;
        report_.old_wall_segment_id = wall_segment_id_ - 1;
        report_.new_wall_segment_id = wall_segment_id_;
        report_.wall_segment_id = wall_segment_id_;
        report_.corner_transition_id = corner_transition_id_;
        ++report_.wall_model_reset_due_to_corner_count;
        segment_wall_point_hash_ = 1469598103934665603ULL;
        segment_wall_model_hash_ = 1469598103934665603ULL;
        if (rectangle_mode()) {
            report_.forward_steps_per_segment.push_back(segment_forward_steps_);
            report_.odom_distance_per_segment_m.push_back(report_.segment_odom_distance_m);
            segment_forward_steps_ = 0;
            report_.segment_odom_distance_m = 0.0;
            forward_steps_since_last_corner_ = 0;
            report_.forward_steps_since_last_corner = 0;
            report_.odom_distance_since_last_corner_m = 0.0;
        }
        report_.new_wall_point_hash = segment_wall_point_hash_;
        report_.new_wall_model_hash = segment_wall_model_hash_;
        corner_transition_event("wall_segment_reset");
        new_wall_mode_ = true;
        new_wall_resample_attempts_ = 0;
        new_wall_acquisition_steps_ = 0;
        report_.new_wall_model_valid = false;
        current_corner_follow_steps_ = 0;
        report_.post_corner_follow_steps = 0;
        corner_detector_armed_ = false;
        pending_rectangle_transition_completion_ = rectangle_mode();
        if (rectangle_mode()) {
            // The completed turn has been validated above.  These are
            // per-transition guards; cumulative report counters stay intact.
            corner_candidate_pending_ = false;
            corner_confirmed_ = false;
            main_turn_finished_ = false;
            post_turn_sensor_verified_ = false;
            right_clearance_passed_ = false;
            current_corner_main_turn_command_count_ = 0;
            corner_residual_corrections_ = 0;
            corner_main_command_id_ = 0;
            corner_residual_command_id_ = 0;
        }
        transition(StopGoMappingControllerState::NewLeftWallReacquisition);
        return true;
    }

    bool acquire_new_wall_sample(const RobotSlamSensorSnapshot &base, double now_s) {
        ++report_.new_wall_reacquisition_samples;
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.front.usable_for_mapping ||
            stable.front.distance_mm <= config_.post_turn_front_stop_threshold_m * 1000.0 + 1e-9) {
            if (++new_wall_resample_attempts_ < config_.new_wall_stationary_resample_attempts) return true;
            return post_turn_front_blocked("new_wall_front_not_safe", now_s);
        }
        if (!stable.left.usable_for_mapping ||
            stable.left.distance_mm < config_.new_wall_min_distance_m * 1000.0 - 1e-9 ||
            stable.left.distance_mm > config_.new_wall_max_distance_m * 1000.0 + 1e-9) {
            ++new_wall_resample_attempts_;
            if (new_wall_resample_attempts_ < config_.new_wall_stationary_resample_attempts) return true;
            if (new_wall_acquisition_steps_ < config_.new_wall_acquisition_max_forward_steps) {
                new_wall_resample_attempts_ = 0;
                transition(StopGoMappingControllerState::IssueForwardStep);
                return true;
            }
            return fail_new_wall("new_left_wall_reacquisition_failed", now_s);
        }
        if (!stable.right.usable_for_mapping) {
            if (++new_wall_resample_attempts_ < config_.new_wall_stationary_resample_attempts) return true;
            return fail_new_wall("new_wall_right_sample_invalid", now_s);
        }
        new_wall_resample_attempts_ = 0;
        const auto full = make_stable_snapshot(base, stable);
        const auto before = application_.core().report().current_map_revision;
        const auto committed = application_.step(full, now_s);
        const auto after = application_.core().report().current_map_revision;
        const auto &accepted = application_.core().last_accepted_map_observation();
        if (!committed.ok || after <= before || !accepted.valid || !accepted.backend_accepted ||
            !accepted.measurement_poses.left.valid || accepted.frame_transform_epoch != wall_epoch_) {
            return fail_new_wall("new_wall_mapping_commit_failed", now_s);
        }
        ++report_.map_commits;
        const auto point = project_left_wall_point(stable.left,
            accepted.measurement_poses.left.base_pose_in_map, after,
            accepted.frame_transform_epoch, report_.completed_steps);
        if (!point) return fail_new_wall("new_wall_point_projection_failed", now_s);
        LeftWallPoint segmented = *point;
        segmented.wall_segment_id = wall_segment_id_;
        wall_points_.push_back(segmented);
        while (wall_points_.size() > config_.wall_estimator.window_max_points) wall_points_.erase(wall_points_.begin());
        hash_wall_point(segmented);
        wall_model_ = wall_estimator_.fit(wall_points_,
            application_.core().current_map_pose().map_T_base, wall_epoch_);
        wall_model_.wall_segment_id = wall_segment_id_;
        hash_wall_model(wall_model_);
        update_wall_fit_report();
        StopGoReplayRecord replay = make_replay_record(full, stable, before, after,
            report_.completed_steps, 0);
        replay.controller_state = StopGoMappingControllerState::CommitMappingSample;
        replay.left_wall_point_accepted = true;
        replay.left_wall_point = segmented;
        replay.wall_model = wall_model_;
        replay.wall_segment_id = wall_segment_id_;
        replay.corner_transition_id = corner_transition_id_;
        replay.corner_transition_count = report_.corner_transition_count;
        replay.post_corner_sensor_verified = pending_replay_post_corner_verify_;
        replay.new_wall_reacquisition = wall_segment_id_ >= 2;
        replay.post_corner_follow_steps = report_.post_corner_follow_steps;
        replay.corner_confirmation_samples = report_.corner_confirmation_count;
        replay.corner_confirmation_rejects = report_.corner_confirmation_reject_count;
        replay.corner_right_clearance_checks = report_.corner_right_clearance_check_count;
        replay.odom_samples = std::move(pending_replay_odom_samples_);
        pending_replay_odom_samples_.clear();
        pending_replay_post_corner_verify_ = false;
        pending_replay_corner_candidate_ = false;
        pending_replay_corner_confirmation_ = false;
        pending_replay_corner_clearance_ = false;
        pending_replay_corner_main_turn_ = false;
        pending_replay_corner_residual_ = false;
        pending_replay_corner_residual_right_ = false;
        report_.replay_records.push_back(replay);
        write_log(replay, now_s);
        if (wall_model_.valid) {
            report_.new_wall_model_valid = true;
            new_wall_mode_ = false;
            transition(StopGoMappingControllerState::CheckLimit);
            if (rectangle_mode()) {
                return prepare_rectangle_segment_follow(now_s);
            }
            return check_single_corner_limit(now_s);
        }
        if (new_wall_acquisition_steps_ >= config_.new_wall_acquisition_max_forward_steps) {
            return fail_new_wall("new_wall_fit_limit_reached", now_s);
        }
        transition(StopGoMappingControllerState::IssueForwardStep);
        return true;
    }

    static double rectangle_line_offset(const LeftWallModel &model,
                                        const RobotPose2D &pose) {
        const double nx = -std::sin(model.wall_heading_rad);
        const double ny = std::cos(model.wall_heading_rad);
        return nx * pose.x_m + ny * pose.y_m +
               model.signed_base_to_wall_distance_m;
    }

    static std::uint64_t rectangle_wall_signature(const LeftWallModel &model) {
        std::uint64_t hash = 1469598103934665603ULL;
        hash_mix(hash, quantized(model.wall_heading_rad));
        hash_mix(hash, quantized(model.signed_base_to_wall_distance_m));
        hash_mix(hash, quantized(model.baseline_m));
        hash_mix(hash, quantized(model.rms_residual_m));
        hash_mix(hash, model.inlier_point_count);
        return hash;
    }

    bool update_rectangle_odom_distance() {
        const RobotPose2D pose = application_.core().report().last_odom_pose;
        if (!sparse_slam_pose_finite(pose)) {
            ++report_.odom_distance_jump_reject_count;
            report_.odom_distance_rejection_reason = "odom_pose_non_finite";
            return false;
        }
        if (!previous_odom_pose_valid_) {
            previous_odom_pose_ = pose;
            previous_odom_pose_valid_ = true;
            return true;
        }
        const double dx = pose.x_m - previous_odom_pose_.x_m;
        const double dy = pose.y_m - previous_odom_pose_.y_m;
        const double distance = std::hypot(dx, dy);
        previous_odom_pose_ = pose;
        if (!std::isfinite(distance) || distance < 0.0) {
            ++report_.odom_distance_jump_reject_count;
            report_.odom_distance_rejection_reason = "odom_increment_non_finite";
            return false;
        }
        // A 50 Hz odometry update cannot legitimately jump by a quarter metre
        // in this low-speed formal runner.  Never replace a rejected increment
        // with the requested command distance.
        if (distance > 0.25) {
            ++report_.odom_distance_jump_reject_count;
            report_.odom_distance_rejection_reason = "odom_increment_exceeds_0_25_m";
            return false;
        }
        report_.total_odom_travel_distance_m += distance;
        report_.segment_odom_distance_m += distance;
        report_.odom_distance_since_last_corner_m += distance;
        total_odom_travel_distance_m_ = report_.total_odom_travel_distance_m;
        return true;
    }

    void capture_rectangle_start_anchor_if_ready(
        const RobotSlamSensorSnapshot &, double now_s) {
        if (!rectangle_mode() || report_.run_start_anchor.valid || !wall_model_.valid ||
            wall_segment_id_ != 1 || report_.corner_main_turn_command_count != 0 ||
            !application_.core().localization_ready()) return;
        const auto pose = application_.core().current_map_pose().map_T_base;
        const auto odom = application_.core().report().last_odom_pose;
        if (!sparse_slam_pose_finite(pose) || !sparse_slam_pose_finite(odom)) return;
        RectangleRunStartAnchor anchor;
        anchor.valid = true;
        anchor.start_map_pose = pose;
        anchor.start_odom_pose = odom;
        anchor.start_timestamp_s = now_s;
        anchor.start_map_revision = application_.core().report().current_map_revision;
        anchor.start_frame_transform_epoch = wall_epoch_;
        anchor.start_wall_segment_id = wall_segment_id_;
        anchor.initial_wall_heading_rad = wall_model_.wall_heading_rad;
        anchor.initial_base_to_wall_distance_m = wall_model_.signed_base_to_wall_distance_m;
        anchor.initial_wall_line_offset_m = rectangle_line_offset(wall_model_, pose);
        anchor.initial_wall_model_rms_m = wall_model_.rms_residual_m;
        anchor.initial_wall_model_baseline_m = wall_model_.baseline_m;
        anchor.initial_wall_model_input_point_count = wall_model_.input_point_count;
        anchor.initial_wall_model_inlier_point_count = wall_model_.inlier_point_count;
        anchor.initial_wall_model_signature_hash = rectangle_wall_signature(wall_model_);
        anchor.odom_distance_at_start_m = report_.total_odom_travel_distance_m;
        anchor.completed_forward_steps_at_start = report_.completed_steps;
        report_.run_start_anchor = anchor;
        report_.run_anchor_hash = anchor.initial_wall_model_signature_hash;
        hash_mix(report_.run_anchor_hash, quantized(anchor.start_map_pose.x_m));
        hash_mix(report_.run_anchor_hash, quantized(anchor.start_map_pose.y_m));
        hash_mix(report_.run_anchor_hash, quantized(anchor.start_map_pose.yaw_rad));
    }

    void maybe_rearm_corner_detector(double front_distance_m) {
        if (!rectangle_mode() || report_.run_start_anchor.valid == false ||
            wall_segment_id_ > config_.rectangle_target_corner_transitions + 1 ||
            corner_detector_armed_ || new_wall_mode_ || !wall_model_.valid ||
            !application_.core().localization_ready() || wall_epoch_ == 0 ||
            application_.core().frame_transform_epoch() != wall_epoch_ ||
            !std::isfinite(front_distance_m) ||
            front_distance_m < config_.rectangle_corner_rearm_front_clearance_m ||
            forward_steps_since_last_corner_ <
                config_.rectangle_minimum_follow_steps_before_next_corner ||
            report_.odom_distance_since_last_corner_m + 1e-9 <
                config_.rectangle_minimum_odom_distance_before_next_corner_m) {
            return;
        }
        corner_detector_armed_ = true;
        // Count every successful disarmed->armed edge.  Nominal closure may
        // finish before segment 5 needs its extra-corner guard, so the usual
        // four events are the initial arm plus segments 2..4.  A failed
        // closure search that continues far enough records the segment-5 arm
        // as an additional bounded guard event.
        ++report_.corner_rearm_count;
        report_.corner_detector_armed = true;
        corner_transition_event("corner_rearmed");
    }

    bool prepare_rectangle_segment_follow(double now_s) {
        if (!rectangle_mode()) return check_single_corner_limit(now_s);
        if (!wall_model_.valid || wall_segment_id_ < 2) {
            return fail_new_wall("rectangle_new_wall_model_invalid", now_s);
        }
        current_corner_follow_steps_ = 0;
        report_.post_corner_follow_steps = 0;
        pending_rectangle_transition_completion_ = true;
        corner_detector_armed_ = false;
        transition(StopGoMappingControllerState::CheckLimit);
        return check_rectangle_progress(now_s);
    }

    bool complete_rectangle_transition_if_ready() {
        if (!pending_rectangle_transition_completion_ ||
            current_corner_follow_steps_ < config_.post_corner_required_follow_steps ||
            !wall_model_.valid) return false;
        pending_rectangle_transition_completion_ = false;
        ++report_.corner_transition_count;
        report_.corner_transition_ids.push_back(corner_transition_id_);
        report_.wall_segment_id = wall_segment_id_;
        if (report_.wall_segment_sequence.empty() ||
            report_.wall_segment_sequence.back() != wall_segment_id_) {
            report_.wall_segment_sequence.push_back(wall_segment_id_);
        }
        if (wall_segment_id_ == config_.rectangle_target_corner_transitions + 1) {
            report_.fourth_corner_follow_steps = current_corner_follow_steps_;
            post_fourth_search_active_ = true;
        }
        corner_detector_armed_ = false;
        report_.corner_detector_armed = false;
        return true;
    }

    bool rectangle_wall_revisit_compatible() {
        if (!report_.run_start_anchor.valid || !wall_model_.valid) return false;
        const auto &anchor = report_.run_start_anchor;
        const auto current_pose = application_.core().current_map_pose().map_T_base;
        const double raw_heading_error = sparse_slam_shortest_yaw_delta(
            anchor.initial_wall_heading_rad, wall_model_.wall_heading_rad);
        // A fitted line is undirected.  Fold the PCA/TLS 180-degree ambiguity
        // into [-pi/2, pi/2] and align the current normal/offset sign with the
        // initial line before comparing offsets.
        report_.initial_wall_heading_error_rad = raw_heading_error;
        if (report_.initial_wall_heading_error_rad > 0.5 * kPi) {
            report_.initial_wall_heading_error_rad -= kPi;
        } else if (report_.initial_wall_heading_error_rad < -0.5 * kPi) {
            report_.initial_wall_heading_error_rad += kPi;
        }
        report_.wall_distance_revisit_error_m =
            std::fabs(wall_model_.signed_base_to_wall_distance_m) -
            std::fabs(anchor.initial_base_to_wall_distance_m);
        const double initial_nx = -std::sin(anchor.initial_wall_heading_rad);
        const double initial_ny = std::cos(anchor.initial_wall_heading_rad);
        const double current_nx = -std::sin(wall_model_.wall_heading_rad);
        const double current_ny = std::cos(wall_model_.wall_heading_rad);
        double current_offset = rectangle_line_offset(wall_model_, current_pose);
        if (initial_nx * current_nx + initial_ny * current_ny < 0.0) {
            current_offset = -current_offset;
        }
        report_.wall_line_offset_revisit_error_m =
            current_offset - anchor.initial_wall_line_offset_m;
        return std::fabs(report_.initial_wall_heading_error_rad) <=
                   config_.rectangle_closure_wall_heading_tolerance_rad + 1e-9 &&
               std::fabs(report_.wall_distance_revisit_error_m) <=
                   config_.rectangle_closure_wall_distance_tolerance_m + 1e-9 &&
               std::fabs(report_.wall_line_offset_revisit_error_m) <=
                   config_.rectangle_closure_wall_line_offset_tolerance_m + 1e-9;
    }

    bool rectangle_closure_geometry_matches() {
        if (!report_.run_start_anchor.valid || !wall_model_.valid ||
            !application_.core().localization_ready() ||
            application_.core().frame_transform_epoch() != wall_epoch_) return false;
        const auto current = application_.core().current_map_pose().map_T_base;
        const auto &start = report_.run_start_anchor.start_map_pose;
        report_.estimated_closure_position_error_m =
            std::hypot(current.x_m - start.x_m, current.y_m - start.y_m);
        report_.estimated_closure_yaw_error_rad = sparse_slam_shortest_yaw_delta(
            start.yaw_rad, current.yaw_rad);
        return report_.estimated_closure_position_error_m <=
                   config_.rectangle_closure_position_tolerance_m + 1e-9 &&
               std::fabs(report_.estimated_closure_yaw_error_rad) <=
                   config_.rectangle_closure_yaw_tolerance_rad + 1e-9 &&
               rectangle_wall_revisit_compatible();
    }

    bool check_rectangle_progress(double now_s) {
        report_.total_odom_travel_distance_m = total_odom_travel_distance_m_;
        report_.corner_detector_armed = corner_detector_armed_;
        if (complete_rectangle_transition_if_ready()) {
            report_.corner_detector_armed = false;
        }
        if (wall_model_.valid && pending_decision_.action != LeftWallControlAction::NoTurn &&
            !corner_candidate_pending_ && !corner_confirmed_) {
            transition(StopGoMappingControllerState::IssueTurnCorrection);
            return true;
        }
        const bool closure_retry_progress_complete =
            !closure_retry_requires_progress_ ||
            (report_.completed_steps > closure_retry_forward_step_baseline_ &&
             report_.total_odom_travel_distance_m >
                 closure_retry_odom_distance_baseline_m_ + 1e-9);
        if (report_.corner_transition_count == config_.rectangle_target_corner_transitions &&
            wall_segment_id_ == config_.rectangle_target_corner_transitions + 1 &&
            current_corner_follow_steps_ >= config_.rectangle_minimum_steps_after_fourth_corner &&
            report_.segment_odom_distance_m >= config_.rectangle_minimum_distance_after_fourth_corner_m &&
            report_.total_odom_travel_distance_m >= config_.rectangle_minimum_total_distance_before_closure_m &&
            closure_retry_progress_complete && rectangle_wall_revisit_compatible()) {
            if (rectangle_closure_geometry_matches()) {
                closure_retry_requires_progress_ = false;
                ++report_.closure_candidate_count;
                if (report_.first_closure_candidate_transition_count == 0) {
                    report_.first_closure_candidate_transition_count =
                        report_.corner_transition_count;
                }
                hash_mix(report_.closure_sequence_hash, report_.corner_transition_count);
                hash_mix(report_.closure_sequence_hash, quantized(report_.estimated_closure_position_error_m));
                transition(StopGoMappingControllerState::RectangleClosureCandidate);
                return true;
            }
        }
        if (report_.corner_transition_count >= config_.rectangle_target_corner_transitions &&
            stable_front_is_corner_hit(last_stable_front_) && corner_detector_armed_) {
            return fail("unexpected_extra_corner_before_closure", now_s);
        }
        if (report_.completed_steps >= config_.rectangle_maximum_total_forward_steps ||
            report_.total_odom_travel_distance_m >= config_.rectangle_maximum_total_odom_distance_m ||
            (report_.run_start_anchor.valid &&
             now_s - report_.run_start_anchor.start_timestamp_s >=
                 config_.rectangle_maximum_runtime_s) ||
            (post_fourth_search_active_ &&
             (report_.fourth_corner_follow_steps >= config_.rectangle_maximum_post_fourth_corner_forward_steps ||
              report_.segment_odom_distance_m >= config_.rectangle_maximum_post_fourth_corner_odom_distance_m))) {
            return fail("rectangle_closure_not_reached", now_s);
        }
        transition(StopGoMappingControllerState::IssueForwardStep);
        return true;
    }

    bool begin_rectangle_closure_confirmation(double now_s) {
        (void)motion_.stop(now_s);
        gate_.reset();
        closure_gate_complete_ = false;
        closure_confirmation_attempts_ = 0;
        closure_confirmation_passes_ = 0;
        closure_confirmation_consecutive_passes_ = 0;
        transition(StopGoMappingControllerState::RectangleClosureConfirming);
        return true;
    }

    bool confirm_rectangle_closure(const RobotSlamSensorSnapshot &base,
                                   double now_s) {
        if (!closure_gate_complete_) {
            last_motion_ = motion_.poll(now_s);
            const auto settled = gate_.update(last_motion_, make_odom_only(base),
                                              monotonic_us(now_s));
            if (!settled.stable) return true;
            closure_gate_complete_ = true;
            // This tick already appended the odometry-only pose above.  Wait
            // one fresh sensor timestamp before asking Core to resolve the
            // confirmation ToF snapshot.
            return true;
        }
        if (!application_.core().localization_ready() ||
            application_.core().frame_transform_epoch() != wall_epoch_) {
            return fail("rectangle_closure_core_not_ready", now_s);
        }
        ++closure_confirmation_attempts_;
        report_.closure_confirmation_attempt_count = closure_confirmation_attempts_;
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        bool pass = stable.front.usable_for_mapping && stable.left.usable_for_mapping &&
                    stable.right.usable_for_mapping &&
                    stable.front.distance_mm > config_.front_stop_threshold_m * 1000.0;
        if (pass && !core_accepts_no_map_snapshot(base, stable, now_s,
                                                   "rectangle_closure_sensor_core_rejected")) {
            return false;
        }
        if (pass) pass = rectangle_closure_geometry_matches();
        if (pass) {
            ++closure_confirmation_passes_;
            ++closure_confirmation_consecutive_passes_;
            report_.closure_confirmation_pass_count = closure_confirmation_passes_;
        } else {
            closure_confirmation_consecutive_passes_ = 0;
            ++report_.closure_confirmation_reject_count;
        }
        report_.closure_confirmation_attempt_count = closure_confirmation_attempts_;
        if (closure_confirmation_consecutive_passes_ >=
            config_.rectangle_closure_confirmation_required_passes) {
            if (closure_confirmation_attempts_ <
                config_.rectangle_closure_confirmation_samples) {
                return true;
            }
            gate_.reset();
            transition(StopGoMappingControllerState::FinalSettle);
            return true;
        }
        if (closure_confirmation_attempts_ >= config_.rectangle_closure_confirmation_max_attempts) {
            closure_gate_complete_ = false;
            closure_retry_requires_progress_ = true;
            closure_retry_forward_step_baseline_ = report_.completed_steps;
            closure_retry_odom_distance_baseline_m_ =
                report_.total_odom_travel_distance_m;
            transition(StopGoMappingControllerState::CheckLimit);
            return check_rectangle_progress(now_s);
        }
        return true;
    }

    bool settle_rectangle_final(const RobotSlamSensorSnapshot &snapshot,
                                double now_s) {
        last_motion_ = motion_.poll(now_s);
        const auto settled = gate_.update(last_motion_, make_odom_only(snapshot),
                                          monotonic_us(now_s), true);
        if (!settled.stable) return true;
        report_.final_settle_complete = true;
        transition(StopGoMappingControllerState::FinalMappingCommit);
        return true;
    }

    bool commit_rectangle_final_sample(const RobotSlamSensorSnapshot &base,
                                       double now_s) {
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.usable_for_mapping || !stable.front.usable_for_mapping ||
            !stable.left.usable_for_mapping || !stable.right.usable_for_mapping) {
            return fail("rectangle_final_stable_sample_failed", now_s);
        }
        const auto full = make_stable_snapshot(base, stable);
        const auto before = application_.core().report().current_map_revision;
        const auto committed = application_.step(full, now_s);
        const auto after = application_.core().report().current_map_revision;
        if (!committed.ok || after <= before ||
            !application_.core().localization_ready() ||
            application_.core().active_observation_state() !=
                ActiveObservationBundleState::Idle ||
            application_.core().prepared_local_match_input().is_ready() ||
            application_.core().local_match_result() != nullptr) {
            report_.map_save_failure_reason = committed.reason;
            return fail("rectangle_final_mapping_commit_failed", now_s);
        }
        ++report_.map_commits;
        ++report_.final_mapping_commit_count;
        report_.map_revision = after;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        if (report_.forward_steps_per_segment.size() < wall_segment_id_) {
            report_.forward_steps_per_segment.push_back(segment_forward_steps_);
        }
        if (report_.odom_distance_per_segment_m.size() < wall_segment_id_) {
            report_.odom_distance_per_segment_m.push_back(report_.segment_odom_distance_m);
        }
        StopGoReplayRecord replay = make_replay_record(
            full, stable, before, after, report_.stable_samples, 0);
        replay.closure_candidate = true;
        replay.closure_confirmation_pass = true;
        replay.final_mapping_commit = true;
        replay.corner_transition_count = report_.corner_transition_count;
        replay.final_map_revision = after;
        report_.replay_records.push_back(replay);
        write_log(replay, now_s);
        report_.ok = true;
        report_.termination_reason = "rectangle_closure_confirmed";
        motion_.stop(now_s);
        transition(StopGoMappingControllerState::Completed);
        return true;
    }

    bool check_single_corner_limit(double now_s) {
        report_.map_revision_after_corner = application_.core().report().current_map_revision;
        if (rectangle_mode()) return check_rectangle_progress(now_s);
        if (!new_wall_mode_ && wall_model_.valid && report_.corner_main_turn_command_count != 0) {
            if (report_.post_corner_follow_steps >= config_.post_corner_required_follow_steps) {
                report_.wall_segment_id = wall_segment_id_;
                report_.new_wall_point_hash = segment_wall_point_hash_;
                report_.new_wall_model_hash = segment_wall_model_hash_;
                return complete_single_corner(now_s);
            }
        }
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return fail("single_corner_follow_limit_reached", now_s);
        }
        transition(StopGoMappingControllerState::IssueForwardStep);
        return true;
    }

    bool complete_single_corner(double) {
        motion_.stop(0.0);
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        report_.map_revision_after_corner = report_.map_revision;
        report_.wall_segment_id = wall_segment_id_;
        report_.new_wall_point_hash = segment_wall_point_hash_;
        report_.new_wall_model_hash = segment_wall_model_hash_;
        report_.ok = true;
        report_.termination_reason = "single_corner_complete";
        transition(StopGoMappingControllerState::SingleCornerComplete);
        return true;
    }

    bool turn_clearance_blocked(const char *reason, double now_s) {
        motion_.stop(now_s);
        report_.corner_failure_reason = reason;
        report_.ok = false;
        report_.termination_reason = "turn_clearance_blocked";
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        transition(StopGoMappingControllerState::TurnClearanceBlocked);
        return false;
    }

    bool corner_turn_verification_failed(const char *reason, double now_s) {
        motion_.stop(now_s);
        report_.corner_failure_reason = reason;
        report_.ok = false;
        report_.termination_reason = "corner_turn_verification_failed";
        transition(StopGoMappingControllerState::CornerTurnVerificationFailed);
        return false;
    }

    bool post_turn_front_blocked(const char *reason, double now_s) {
        motion_.stop(now_s);
        report_.corner_failure_reason = reason;
        report_.ok = false;
        report_.termination_reason = "post_turn_front_blocked";
        transition(StopGoMappingControllerState::PostTurnFrontBlocked);
        return false;
    }

    bool fail_new_wall(const char *reason, double now_s) {
        motion_.stop(now_s);
        report_.corner_failure_reason = reason;
        report_.ok = false;
        report_.termination_reason = "new_left_wall_reacquisition_failed";
        transition(StopGoMappingControllerState::NewLeftWallReacquisitionFailed);
        return false;
    }

    void corner_transition_event(const char *name) {
        for (const unsigned char ch : std::string(name)) hash_mix(report_.corner_event_sequence_hash, ch);
    }

    void update_wall_fit_report() {
        report_.frame_transform_epoch = wall_epoch_;
        report_.wall_fit_input_points = wall_model_.input_point_count;
        report_.wall_fit_inliers = wall_model_.inlier_point_count;
        report_.wall_fit_baseline_m = wall_model_.baseline_m;
        report_.wall_fit_rms_m = wall_model_.rms_residual_m;
        if (wall_model_.valid) ++report_.wall_model_valid_count;
    }

    StopGoReplayRecord make_replay_record(const RobotSlamSensorSnapshot &full,
        const StableThreeTofSample &stable, std::uint64_t before,
        std::uint64_t after, std::size_t cycle, std::uint64_t command_id) const {
        StopGoReplayRecord replay;
        replay.cycle_index = cycle;
        replay.command_id = command_id;
        replay.motion = last_motion_;
        replay.snapshot = full;
        replay.stable_sample = stable;
        replay.map_revision_before = before;
        replay.map_revision_after = after;
        replay.motor_settled_timestamp_us = monotonic_us(last_motion_.final_feedback.wheel.timestamp_s);
        replay.mapping_ready_timestamp_us = gate_.last_result().stable_timestamp_us;
        replay.settle_frame_count = gate_.last_result().consecutive_frames;
        replay.settle_duration_ms = gate_.last_result().stable_duration_ms;
        replay.odom_pose = application_.core().report().last_odom_pose;
        replay.map_from_odom = application_.core().frame_state().current_map_from_odom().map_T_odom;
        replay.frame_transform_epoch = wall_epoch_;
        replay.wall_segment_id = wall_segment_id_;
        replay.corner_transition_id = corner_transition_id_;
        replay.corner_candidate = pending_replay_corner_candidate_;
        replay.corner_confirmation = pending_replay_corner_confirmation_;
        replay.corner_confirmed = corner_confirmed_;
        replay.corner_right_clearance_passed = pending_replay_corner_clearance_;
        replay.corner_main_turn = pending_replay_corner_main_turn_;
        replay.corner_residual_correction = pending_replay_corner_residual_;
        replay.corner_residual_right = pending_replay_corner_residual_right_;
        replay.post_corner_sensor_verified = pending_replay_post_corner_verify_;
        replay.corner_main_command_id = corner_main_command_id_;
        replay.corner_residual_command_id = corner_residual_command_id_;
        replay.pre_turn_odom_yaw_rad = pre_turn_odom_yaw_rad_;
        replay.post_turn_odom_yaw_rad = post_main_odom_yaw_rad_;
        replay.actual_turn_delta_rad = verified_turn_delta_rad_;
        replay.turn_residual_rad = final_turn_error_rad_;
        replay.corner_transition_count = report_.corner_transition_count;
        replay.run_start_anchor_valid = report_.run_start_anchor.valid;
        replay.run_start_map_pose = report_.run_start_anchor.start_map_pose;
        replay.run_start_odom_pose = report_.run_start_anchor.start_odom_pose;
        replay.run_start_timestamp_s = report_.run_start_anchor.start_timestamp_s;
        replay.run_start_map_revision = report_.run_start_anchor.start_map_revision;
        replay.run_start_frame_transform_epoch =
            report_.run_start_anchor.start_frame_transform_epoch;
        replay.run_start_wall_heading_rad =
            report_.run_start_anchor.initial_wall_heading_rad;
        replay.run_start_wall_distance_m =
            report_.run_start_anchor.initial_base_to_wall_distance_m;
        replay.run_start_wall_line_offset_m =
            report_.run_start_anchor.initial_wall_line_offset_m;
        replay.run_start_wall_rms_m =
            report_.run_start_anchor.initial_wall_model_rms_m;
        replay.run_start_wall_baseline_m =
            report_.run_start_anchor.initial_wall_model_baseline_m;
        replay.run_start_wall_input_point_count =
            report_.run_start_anchor.initial_wall_model_input_point_count;
        replay.run_start_wall_inlier_point_count =
            report_.run_start_anchor.initial_wall_model_inlier_point_count;
        replay.run_start_wall_signature_hash =
            report_.run_start_anchor.initial_wall_model_signature_hash;
        replay.run_start_odom_distance_baseline_m =
            report_.run_start_anchor.odom_distance_at_start_m;
        replay.run_start_forward_step_baseline =
            report_.run_start_anchor.completed_forward_steps_at_start;
        replay.total_odom_travel_distance_m =
            report_.total_odom_travel_distance_m;
        replay.segment_odom_distance_m = report_.segment_odom_distance_m;
        replay.forward_steps_since_last_corner =
            report_.forward_steps_since_last_corner;
        replay.corner_rearm_count = report_.corner_rearm_count;
        replay.closure_candidate_count = report_.closure_candidate_count;
        replay.closure_confirmation_attempt_count =
            report_.closure_confirmation_attempt_count;
        replay.closure_confirmation_pass_count =
            report_.closure_confirmation_pass_count;
        replay.closure_confirmation_reject_count =
            report_.closure_confirmation_reject_count;
        replay.estimated_closure_position_error_m =
            report_.estimated_closure_position_error_m;
        replay.estimated_closure_yaw_error_rad =
            report_.estimated_closure_yaw_error_rad;
        replay.final_map_revision = report_.map_revision;
        replay.forward_steps_per_segment = report_.forward_steps_per_segment;
        replay.odom_distance_per_segment_m = report_.odom_distance_per_segment_m;
        replay.wall_segment_sequence = report_.wall_segment_sequence;
        replay.corner_transition_ids = report_.corner_transition_ids;
        return replay;
    }

    bool tick_left_wall(double now_s) {
        if (terminal()) return report_.ok;
        if (!snapshot_reader_ || !motion_.ready()) {
            return fail("controller_port_or_sensor_not_ready", now_s);
        }
        const auto snapshot = snapshot_reader_(now_s);
        const auto &core = application_.core();

        if (state_ == StopGoMappingControllerState::WaitingForCoreReady) {
            report_.core_wait_ticks++;
            if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
                return true;
            }
            const auto app_step = application_.step(
                make_odom_only(snapshot), now_s);
            if (core.localization_ready()) {
                report_.core_ready_observed = true;
                gate_.reset();
                transition(StopGoMappingControllerState::InitialSettle);
                return true;
            }
            const auto &core_report = core.report();
            const bool recovery_wait = core.relocalization_required() ||
                core_report.relocalization_active ||
                core_report.localization_stop_required;
            if (!recovery_wait &&
                (core_report.initialization_status == "fault" ||
                 core_report.initialization_status == "gyro_calibration_failed" ||
                 core_report.initialization_status == "wheel_baseline_failed" ||
                 core_report.last_fault != "none")) {
                return fail("core_initialization_failed:" +
                                (app_step.reason.empty()
                                     ? core_report.last_message
                                     : app_step.reason),
                            now_s);
            }
            return true;
        }

        if (!core.localization_ready() ||
            core.localization_health_state() == LocalizationHealthState::Lost ||
            core.report().localization_stop_required ||
            core.relocalization_required()) {
            if (state_ != StopGoMappingControllerState::WaitingForCoreReady) {
                (void)motion_.stop(now_s);
                clear_wall_window("core_not_ready_or_recovering", true);
                transition(StopGoMappingControllerState::WaitingForCoreReady);
            }
            if (snapshot.has_wheel && snapshot.has_imu && snapshot.wheel.valid) {
                (void)application_.step(make_odom_only(snapshot), now_s);
            }
            return true;
        }

        if (wall_epoch_ != 0 && core.frame_transform_epoch() != wall_epoch_) {
            (void)motion_.stop(now_s);
            clear_wall_window("frame_transform_epoch_changed", true);
            transition(StopGoMappingControllerState::WallAcquisition);
            return true;
        }
        if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
            return fail("canonical_wheel_or_imu_feedback_invalid", now_s);
        }
        const auto odom_only = make_odom_only(snapshot);
        const bool commit_state =
            state_ == StopGoMappingControllerState::InitialSample ||
            state_ == StopGoMappingControllerState::AcquireThreeTof ||
            state_ == StopGoMappingControllerState::VerifyAfterTurn;
        if (!commit_state) {
            const auto before = core.report().current_map_revision;
            const auto app_step = application_.step(odom_only, now_s);
            if (!app_step.ok || !app_step.core_step_executed) {
                return fail("application_core_step_failed:" + app_step.reason, now_s);
            }
            const auto after = core.report().current_map_revision;
            if (state_ == StopGoMappingControllerState::WaitMotionDone ||
                state_ == StopGoMappingControllerState::WaitMappingSettle) {
                if (after != before) report_.map_writes_while_moving++;
            }
            if (state_ == StopGoMappingControllerState::WaitTurnDone ||
                state_ == StopGoMappingControllerState::WaitTurnSettle) {
                if (after != before) report_.map_writes_during_turn_or_verify++;
            }
            pending_replay_odom_samples_.push_back({odom_only.wheel, odom_only.imu});
        }

        switch (state_) {
        case StopGoMappingControllerState::InitialSettle: {
            RelativeMotionStepResult initial;
            initial.state = RelativeMotionStepState::Done;
            const auto settled = gate_.update(initial, odom_only,
                                              monotonic_us(now_s), true);
            if (settled.stable) {
                transition(StopGoMappingControllerState::InitialSample);
            }
            return true;
        }
        case StopGoMappingControllerState::InitialSample:
            transition(StopGoMappingControllerState::WallAcquisition);
            return acquire_left_wall_sample(snapshot, now_s, 0);
        case StopGoMappingControllerState::WallAcquisition:
            return acquire_left_wall_sample(snapshot, now_s,
                                            report_.completed_steps + 1);
        case StopGoMappingControllerState::IssueForwardStep:
            return issue_forward_step(now_s);
        case StopGoMappingControllerState::WaitMotionDone: {
            const auto result = motion_.poll(now_s);
            if (!motion_result_matches_command(result)) {
                return fail("forward_command_id_mismatch", now_s);
            }
            last_motion_ = result;
            if (motion_failed(result)) return fail("motion_failed:" + result.reason, now_s);
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitMappingSettle);
            }
            return true;
        }
        case StopGoMappingControllerState::WaitMappingSettle: {
            last_motion_ = motion_.poll(now_s);
            if (!motion_result_matches_command(last_motion_)) {
                return fail("forward_settle_command_id_mismatch", now_s);
            }
            const auto settled = gate_.update(last_motion_, odom_only,
                                              monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::AcquireThreeTof);
            return true;
        }
        case StopGoMappingControllerState::AcquireThreeTof:
            return acquire_left_wall_sample(snapshot, now_s,
                                             report_.completed_steps + 1);
        case StopGoMappingControllerState::IssueTurnCorrection:
            return issue_turn_correction(now_s);
        case StopGoMappingControllerState::WaitTurnDone: {
            const auto result = motion_.poll(now_s);
            if (!motion_result_matches_command(result)) {
                return fail("turn_command_id_mismatch", now_s);
            }
            last_motion_ = result;
            if (motion_failed(result)) return fail("turn_motion_failed:" + result.reason, now_s);
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitTurnSettle);
            }
            return true;
        }
        case StopGoMappingControllerState::WaitTurnSettle: {
            last_motion_ = motion_.poll(now_s);
            if (!motion_result_matches_command(last_motion_)) {
                return fail("turn_settle_command_id_mismatch", now_s);
            }
            const auto settled = gate_.update(last_motion_, odom_only,
                                              monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::VerifyAfterTurn);
            return true;
        }
        case StopGoMappingControllerState::VerifyAfterTurn:
            return verify_after_turn(snapshot, now_s);
        case StopGoMappingControllerState::CheckLimit:
            return check_left_wall_limit(now_s);
        case StopGoMappingControllerState::Completed:
        case StopGoMappingControllerState::WallLost:
        case StopGoMappingControllerState::FrontBlocked:
        case StopGoMappingControllerState::Fault:
        case StopGoMappingControllerState::Estop:
        case StopGoMappingControllerState::Idle:
        case StopGoMappingControllerState::WaitingForCoreReady:
        case StopGoMappingControllerState::CommitMappingSample:
            return true;
        }
        return true;
    }

    static bool motion_failed(const RelativeMotionStepResult &result) {
        return result.state == RelativeMotionStepState::Fault ||
               result.state == RelativeMotionStepState::Timeout ||
               result.state == RelativeMotionStepState::Cancelled ||
               result.state == RelativeMotionStepState::EstopLatched;
    }

    bool motion_result_matches_command(const RelativeMotionStepResult &result) const {
        return result.command_id == command_.command_id &&
               result.action == command_.action;
    }

    bool issue_forward_step(double now_s) {
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return complete("max_steps_or_distance_reached", now_s);
        }
        command_ = {};
        command_.command_id = next_command_id_++;
        command_.action = RelativeMotionStepAction::Forward;
        command_.target_amount = config_.forward_step_mm;
        command_.max_rpm = config_.motion_max_rpm;
        command_.timeout_ms = config_.motion_timeout_ms;
        const auto accepted = motion_.submit(command_, now_s);
        if (accepted.state != RelativeMotionStepState::Accepted) {
            return fail("motion_submit_failed:" + accepted.reason, now_s);
        }
        report_.commands_submitted++;
        report_.forward_command_count++;
        report_.command_ids.push_back(command_.command_id);
        if (!wall_model_.valid) report_.wall_acquisition_steps++;
        transition(StopGoMappingControllerState::WaitMotionDone);
        return true;
    }

    bool issue_turn_correction(double now_s) {
        if (pending_decision_.action == LeftWallControlAction::NoTurn) {
            transition(StopGoMappingControllerState::IssueForwardStep);
            return true;
        }
        command_ = {};
        command_.command_id = next_command_id_++;
        command_.action = pending_decision_.action == LeftWallControlAction::Left
            ? RelativeMotionStepAction::Left : RelativeMotionStepAction::Right;
        command_.target_amount = pending_decision_.correction_amount_deg;
        command_.max_rpm = config_.motion_max_rpm;
        command_.timeout_ms = config_.motion_timeout_ms;
        const auto accepted = motion_.submit(command_, now_s);
        if (accepted.state != RelativeMotionStepState::Accepted) {
            return fail("turn_submit_failed:" + accepted.reason, now_s);
        }
        report_.commands_submitted++;
        report_.correction_command_count++;
        if (command_.action == RelativeMotionStepAction::Left) {
            report_.left_correction_count++;
        } else {
            report_.right_correction_count++;
        }
        report_.command_ids.push_back(command_.command_id);
        transition(StopGoMappingControllerState::WaitTurnDone);
        return true;
    }

    bool acquire_left_wall_sample(const RobotSlamSensorSnapshot &base,
                                  double now_s, std::size_t cycle) {
        transition(StopGoMappingControllerState::AcquireThreeTof);
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.front.usable_for_mapping) {
            ++front_invalid_samples_;
            if (front_invalid_samples_ <= config_.front_max_invalid_samples) {
                return true;
            }
            return front_blocked("front_sample_invalid", now_s);
        }
        front_invalid_samples_ = 0;
        if (stable.front.distance_mm <=
            config_.front_stop_threshold_m * 1000.0 + 1e-9) {
            return front_blocked("front_obstacle_below_stop_threshold", now_s);
        }
        if (!stable.left.usable_for_mapping) {
            if (wall_points_.empty()) {
                ++stationary_resample_attempts_;
                if (stationary_resample_attempts_ <=
                    config_.max_stationary_resample_attempts) {
                    return true;
                }
                return wall_lost("initial_left_wall_resample_exhausted", now_s);
            }
            ++consecutive_wall_misses_;
            if (consecutive_wall_misses_ <= config_.max_consecutive_wall_misses) {
                return true;
            }
            return wall_lost("left_wall_measurement_lost", now_s);
        }
        stationary_resample_attempts_ = 0;
        consecutive_wall_misses_ = 0;
        if (!stable.right.usable_for_mapping) {
            return fail("right_measurement_unusable_for_core_map_commit", now_s);
        }

        transition(StopGoMappingControllerState::CommitMappingSample);
        const auto full = make_stable_snapshot(base, stable);
        const std::uint64_t before = application_.core().report().current_map_revision;
        const auto committed = application_.step(full, now_s);
        const std::uint64_t after = application_.core().report().current_map_revision;
        const auto &accepted = application_.core().last_accepted_map_observation();
        if (!committed.ok || after <= before || !accepted.valid ||
            !accepted.backend_accepted ||
            accepted.map_revision_before != before ||
            accepted.map_revision_after != after ||
            !accepted.measurement_poses.left.valid ||
            accepted.frame_transform_epoch != application_.core().frame_transform_epoch()) {
            return fail("stable_snapshot_map_commit_failed:" + committed.reason, now_s);
        }
        report_.map_commits++;
        report_.completed_steps += cycle == 0 ? 0 : 1;
        if (cycle != 0) {
            total_distance_mm_ += last_motion_.actual_distance_mm;
            report_.commands_completed++;
        }
        const auto point = project_left_wall_point(
            stable.left, accepted.measurement_poses.left.base_pose_in_map,
            accepted.map_revision_after, accepted.frame_transform_epoch, cycle);
        if (!point) return fail("left_wall_point_projection_failed", now_s);
        if (wall_epoch_ == 0) wall_epoch_ = accepted.frame_transform_epoch;
        if (point->frame_transform_epoch != wall_epoch_) {
            clear_wall_window("frame_transform_epoch_changed_at_point", true);
            transition(StopGoMappingControllerState::WallAcquisition);
            return true;
        }
        wall_points_.push_back(*point);
        while (wall_points_.size() > config_.wall_estimator.window_max_points) {
            wall_points_.erase(wall_points_.begin());
        }
        hash_wall_point(*point);
        wall_model_ = wall_estimator_.fit(
            wall_points_, application_.core().current_map_pose().map_T_base,
            wall_epoch_);
        report_.frame_transform_epoch = wall_epoch_;
        report_.wall_fit_input_points = wall_model_.input_point_count;
        report_.wall_fit_inliers = wall_model_.inlier_point_count;
        report_.wall_fit_baseline_m = wall_model_.baseline_m;
        report_.wall_fit_rms_m = wall_model_.rms_residual_m;
        hash_wall_model(wall_model_);
        if (wall_model_.valid) {
            report_.wall_model_valid_count++;
            if (!desired_distance_captured_ &&
                config_.desired_distance_mode == "capture_initial") {
                desired_distance_m_ = wall_model_.signed_base_to_wall_distance_m;
                desired_distance_captured_ = true;
            }
            pending_decision_ = decide_left_wall_control(
                wall_model_, application_.core().current_map_pose().map_T_base,
                desired_distance_m_, config_);
            update_control_metrics(pending_decision_);
            if (wall_model_.signed_base_to_wall_distance_m <
                    config_.min_allowed_base_to_wall_distance_m ||
                wall_model_.signed_base_to_wall_distance_m >
                    config_.max_allowed_base_to_wall_distance_m) {
                return wall_lost("base_to_wall_distance_out_of_safe_range", now_s);
            }
        } else {
            pending_decision_ = {};
            if (report_.wall_acquisition_steps >=
                config_.wall_acquisition_max_forward_steps) {
                return wall_lost("wall_acquisition_limit_reached", now_s);
            }
        }

        StopGoReplayRecord replay;
        replay.cycle_index = cycle;
        replay.command_id = cycle == 0 ? 0 : command_.command_id;
        replay.motion = cycle == 0 ? RelativeMotionStepResult{} : last_motion_;
        replay.snapshot = full;
        replay.stable_sample = stable;
        replay.map_revision_before = before;
        replay.map_revision_after = after;
        replay.motor_settled_timestamp_us = cycle == 0
            ? 0 : monotonic_us(last_motion_.final_feedback.wheel.timestamp_s);
        replay.mapping_ready_timestamp_us = gate_.last_result().stable_timestamp_us;
        replay.settle_frame_count = gate_.last_result().consecutive_frames;
        replay.settle_duration_ms = gate_.last_result().stable_duration_ms;
        replay.odom_pose = application_.core().report().last_odom_pose;
        replay.map_from_odom = application_.core().frame_state().current_map_from_odom().map_T_odom;
        replay.controller_state = StopGoMappingControllerState::CommitMappingSample;
        replay.left_wall_point_accepted = true;
        replay.left_wall_point = *point;
        replay.wall_model = wall_model_;
        replay.control_decision = pending_decision_;
        replay.post_turn_verified = post_turn_verified_since_last_commit_;
        replay.frame_transform_epoch = wall_epoch_;
        replay.odom_samples = std::move(pending_replay_odom_samples_);
        pending_replay_odom_samples_.clear();
        post_turn_verified_since_last_commit_ = false;
        report_.replay_records.push_back(replay);
        write_log(replay, now_s);
        transition(StopGoMappingControllerState::CheckLimit);
        return check_left_wall_limit(now_s);
    }

    bool acquire_single_corner_sample(const RobotSlamSensorSnapshot &base,
                                      double now_s, std::size_t cycle) {
        transition(StopGoMappingControllerState::AcquireThreeTof);
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (rectangle_mode()) last_stable_front_ = stable.front;
        if (!stable.front.usable_for_mapping) {
            if (++front_invalid_samples_ <= config_.front_max_invalid_samples) return true;
            return front_blocked("front_sample_invalid", now_s);
        }
        front_invalid_samples_ = 0;
        if (stable.front.distance_mm <= config_.front_stop_threshold_m * 1000.0 + 1e-9 &&
            (!wall_model_.valid || new_wall_mode_)) {
            return front_blocked("front_obstacle_below_stop_threshold", now_s);
        }
        if (!stable.left.usable_for_mapping) {
            ++stationary_resample_attempts_;
            if (new_wall_mode_) {
                if (stationary_resample_attempts_ < config_.new_wall_stationary_resample_attempts) return true;
                stationary_resample_attempts_ = 0;
                if (new_wall_acquisition_steps_ < config_.new_wall_acquisition_max_forward_steps) {
                    transition(StopGoMappingControllerState::IssueForwardStep);
                    return true;
                }
                return fail_new_wall("new_left_wall_reacquisition_failed", now_s);
            }
            if (stationary_resample_attempts_ <= config_.max_stationary_resample_attempts) return true;
            return wall_lost("initial_left_wall_resample_exhausted", now_s);
        }
        stationary_resample_attempts_ = 0;
        if (!stable.right.usable_for_mapping) return fail("right_measurement_unusable_for_core_map_commit", now_s);

        const auto full = make_stable_snapshot(base, stable);
        const std::uint64_t before = application_.core().report().current_map_revision;
        const auto committed = application_.step(full, now_s);
        const std::uint64_t after = application_.core().report().current_map_revision;
        const auto &accepted = application_.core().last_accepted_map_observation();
        if (!committed.ok || after <= before || !accepted.valid || !accepted.backend_accepted ||
            accepted.map_revision_before != before || accepted.map_revision_after != after ||
            !accepted.measurement_poses.left.valid ||
            accepted.frame_transform_epoch != application_.core().frame_transform_epoch()) {
            return fail(new_wall_mode_ ? "new_wall_mapping_commit_failed" : "stable_snapshot_map_commit_failed", now_s);
        }
        report_.map_commits++;
        if (cycle != 0) {
            ++report_.completed_steps;
            ++report_.commands_completed;
            total_distance_mm_ += last_motion_.actual_distance_mm;
            if (rectangle_mode()) {
                ++forward_steps_since_last_corner_;
                ++segment_forward_steps_;
                report_.forward_steps_since_last_corner = forward_steps_since_last_corner_;
                if (wall_segment_id_ >= 2) {
                    ++current_corner_follow_steps_;
                    report_.post_corner_follow_steps = current_corner_follow_steps_;
                    report_.fourth_corner_follow_steps = wall_segment_id_ == 5
                        ? current_corner_follow_steps_ : report_.fourth_corner_follow_steps;
                }
            } else if (report_.corner_main_turn_command_count == 0) {
                report_.completed_forward_steps_before_corner = report_.completed_steps;
            } else {
                report_.completed_forward_steps_after_corner = report_.completed_steps -
                    report_.completed_forward_steps_before_corner;
                if (new_wall_mode_ || wall_segment_id_ > 1) {
                    ++report_.post_corner_follow_steps;
                }
            }
        }
        const auto projected = project_left_wall_point(stable.left,
            accepted.measurement_poses.left.base_pose_in_map, after,
            accepted.frame_transform_epoch, cycle);
        if (!projected) return fail("left_wall_point_projection_failed", now_s);
        LeftWallPoint point = *projected;
        point.wall_segment_id = wall_segment_id_;
        if (wall_epoch_ == 0) wall_epoch_ = accepted.frame_transform_epoch;
        if (point.frame_transform_epoch != wall_epoch_) {
            clear_wall_window("frame_transform_epoch_changed_at_point", true);
            reset_corner_flow_for_recovery();
            transition(StopGoMappingControllerState::WaitingForCoreReady);
            return true;
        }
        wall_points_.push_back(point);
        while (wall_points_.size() > config_.wall_estimator.window_max_points) wall_points_.erase(wall_points_.begin());
        hash_wall_point(point);
        wall_model_ = wall_estimator_.fit(wall_points_,
            application_.core().current_map_pose().map_T_base, wall_epoch_);
        wall_model_.wall_segment_id = wall_segment_id_;
        hash_wall_model(wall_model_);
        update_wall_fit_report();
        if (wall_model_.valid) {
            if (!desired_distance_captured_ && config_.desired_distance_mode == "capture_initial") {
                desired_distance_m_ = wall_model_.signed_base_to_wall_distance_m;
                desired_distance_captured_ = true;
            }
            pending_decision_ = decide_left_wall_control(
                wall_model_, application_.core().current_map_pose().map_T_base,
                desired_distance_m_, config_);
            update_control_metrics(pending_decision_);
            if (wall_model_.signed_base_to_wall_distance_m < config_.min_allowed_base_to_wall_distance_m ||
                wall_model_.signed_base_to_wall_distance_m > config_.max_allowed_base_to_wall_distance_m) {
                return wall_lost("base_to_wall_distance_out_of_safe_range", now_s);
            }
        }

        if (rectangle_mode() && wall_model_.valid) {
            capture_rectangle_start_anchor_if_ready(base, now_s);
            maybe_rearm_corner_detector(stable.front.distance_mm / 1000.0);
        }

        const bool rectangle_corner_window = rectangle_mode() && corner_detector_armed_ &&
            wall_segment_id_ <= config_.rectangle_target_corner_transitions;
        const bool corner_candidate = wall_model_.valid && !new_wall_mode_ &&
            ((single_corner_mode() && wall_segment_id_ == 1) || rectangle_corner_window) &&
            stable_front_is_corner_hit(stable.front) &&
            stable.left.usable_for_mapping;
        if (corner_candidate) {
            ++report_.corner_candidate_count;
            corner_candidate_pending_ = true;
            if (rectangle_mode()) corner_detector_armed_ = false;
            corner_confirmation_attempts_ = 0;
            corner_confirmation_hits_ = 0;
            pending_replay_corner_candidate_ = true;
            corner_transition_event("corner_candidate");
        }
        if (rectangle_mode() && wall_model_.valid && !new_wall_mode_ &&
            wall_segment_id_ == config_.rectangle_target_corner_transitions + 1 &&
            stable_front_is_corner_hit(stable.front) && stable.left.usable_for_mapping &&
            corner_detector_armed_) {
            return fail("unexpected_extra_corner_before_closure", now_s);
        }
        StopGoReplayRecord replay = make_replay_record(full, stable, before, after,
            cycle, cycle == 0 ? 0 : command_.command_id);
        replay.controller_state = StopGoMappingControllerState::CommitMappingSample;
        replay.left_wall_point_accepted = true;
        replay.left_wall_point = point;
        replay.wall_model = wall_model_;
        replay.control_decision = pending_decision_;
        replay.odom_samples = std::move(pending_replay_odom_samples_);
        pending_replay_odom_samples_.clear();
        report_.replay_records.push_back(replay);
        write_log(replay, now_s);
        pending_replay_corner_candidate_ = false;
        pending_replay_corner_confirmation_ = false;
        pending_replay_corner_clearance_ = false;
        pending_replay_corner_main_turn_ = false;
        pending_replay_corner_residual_ = false;
        pending_replay_post_corner_verify_ = false;

        if (corner_candidate) {
            transition(StopGoMappingControllerState::CornerCandidate);
            return true;
        }
        if (new_wall_mode_ && wall_model_.valid) {
            report_.new_wall_model_valid = true;
            report_.new_wall_point_hash = segment_wall_point_hash_;
            report_.new_wall_model_hash = segment_wall_model_hash_;
            new_wall_mode_ = false;
        }
        transition(StopGoMappingControllerState::CheckLimit);
        return check_single_corner_limit(now_s);
    }

    std::optional<LeftWallPoint> project_left_wall_point(
        const StableTofReading &reading,
        const MapPose2D &measurement_base_pose,
        std::uint64_t map_revision,
        std::uint64_t epoch,
        std::size_t cycle) const {
        if (!reading.usable_for_mapping || !sparse_slam_frame_pose_valid(measurement_base_pose) ||
            !std::isfinite(reading.sample_timestamp_us) ||
            !std::isfinite(reading.distance_mm)) {
            return std::nullopt;
        }
        const auto map_T_left_tof = compose_robot_poses(
            measurement_base_pose.map_T_base, config_.base_T_left_tof);
        const double distance_m = reading.distance_mm / 1000.0;
        LeftWallPoint point;
        point.x_m = map_T_left_tof.x_m + std::cos(map_T_left_tof.yaw_rad) * distance_m;
        point.y_m = map_T_left_tof.y_m + std::sin(map_T_left_tof.yaw_rad) * distance_m;
        point.timestamp_s = reading.sample_timestamp_us / 1e6;
        point.confidence = reading.confidence / 100.0;
        point.cycle_index = cycle;
        point.map_revision = map_revision;
        point.frame_transform_epoch = epoch;
        if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
            !std::isfinite(point.timestamp_s) || !std::isfinite(point.confidence)) {
            return std::nullopt;
        }
        return point;
    }

    static void hash_mix(std::uint64_t &hash, std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ULL;
    }

    static std::uint64_t quantized(double value) {
        if (!std::isfinite(value)) return 0;
        return static_cast<std::uint64_t>(std::llround(value * 1000000.0));
    }

    void hash_wall_point(const LeftWallPoint &point) {
        hash_mix(report_.wall_point_sequence_hash, quantized(point.x_m));
        hash_mix(report_.wall_point_sequence_hash, quantized(point.y_m));
        hash_mix(report_.wall_point_sequence_hash, point.cycle_index);
        hash_mix(report_.wall_point_sequence_hash, point.frame_transform_epoch);
        hash_mix(report_.wall_point_sequence_hash, point.wall_segment_id);
        hash_mix(segment_wall_point_hash_, quantized(point.x_m));
        hash_mix(segment_wall_point_hash_, quantized(point.y_m));
        hash_mix(segment_wall_point_hash_, point.cycle_index);
        hash_mix(segment_wall_point_hash_, point.wall_segment_id);
    }

    void hash_wall_model(const LeftWallModel &model) {
        hash_mix(report_.wall_model_sequence_hash, model.valid ? 1U : 0U);
        hash_mix(report_.wall_model_sequence_hash, quantized(model.wall_heading_rad));
        hash_mix(report_.wall_model_sequence_hash,
                 quantized(model.signed_base_to_wall_distance_m));
        hash_mix(report_.wall_model_sequence_hash, quantized(model.baseline_m));
        hash_mix(report_.wall_model_sequence_hash, quantized(model.rms_residual_m));
        hash_mix(report_.wall_model_sequence_hash, model.inlier_point_count);
        hash_mix(report_.wall_model_sequence_hash, model.frame_transform_epoch);
        hash_mix(report_.wall_model_sequence_hash, model.wall_segment_id);
        hash_mix(segment_wall_model_hash_, model.valid ? 1U : 0U);
        hash_mix(segment_wall_model_hash_, quantized(model.wall_heading_rad));
        hash_mix(segment_wall_model_hash_, quantized(model.signed_base_to_wall_distance_m));
        hash_mix(segment_wall_model_hash_, quantized(model.baseline_m));
        hash_mix(segment_wall_model_hash_, quantized(model.rms_residual_m));
        hash_mix(segment_wall_model_hash_, model.inlier_point_count);
        hash_mix(segment_wall_model_hash_, model.wall_segment_id);
    }

    void update_control_metrics(const LeftWallControlDecision &decision) {
        const double heading_deg = decision.heading_error_rad *
            180.0 / 3.14159265358979323846;
        const double distance_mm = decision.distance_error_m * 1000.0;
        if (!have_control_metrics_) {
            report_.initial_heading_error_deg = heading_deg;
            report_.initial_distance_error_mm = distance_mm;
            have_control_metrics_ = true;
        }
        report_.final_heading_error_deg = heading_deg;
        report_.final_distance_error_mm = distance_mm;
        report_.maximum_abs_heading_error_deg = std::max(
            report_.maximum_abs_heading_error_deg, std::fabs(heading_deg));
        report_.maximum_abs_distance_error_mm = std::max(
            report_.maximum_abs_distance_error_mm, std::fabs(distance_mm));
    }

    void clear_wall_window(const char *reason, bool frame_change) {
        if (!wall_points_.empty() || wall_model_.valid) {
            report_.wall_model_reset_count++;
            if (frame_change) report_.wall_model_reset_due_to_frame_change_count++;
        }
        wall_points_.clear();
        wall_model_ = {};
        pending_decision_ = {};
        consecutive_wall_misses_ = 0;
        stationary_resample_attempts_ = 0;
        front_invalid_samples_ = 0;
        desired_distance_captured_ = false;
        desired_distance_m_ = config_.desired_base_to_wall_distance_m;
        wall_epoch_ = application_.core().frame_transform_epoch();
        report_.frame_transform_epoch = wall_epoch_;
        (void)reason;
    }

    bool wall_lost(const char *reason, double now_s) {
        (void)motion_.stop(now_s);
        clear_wall_window(reason, false);
        transition(StopGoMappingControllerState::WallLost);
        report_.ok = false;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        return false;
    }

    bool front_blocked(const char *reason, double now_s) {
        (void)motion_.stop(now_s);
        transition(StopGoMappingControllerState::FrontBlocked);
        report_.ok = false;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        return false;
    }

    bool verify_after_turn(const RobotSlamSensorSnapshot &base, double now_s) {
        report_.post_turn_verification_count++;
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.front.usable_for_mapping) {
            report_.post_turn_verification_failure_count++;
            return front_blocked("post_turn_front_sample_invalid", now_s);
        }
        if (stable.front.distance_mm <=
            config_.front_stop_threshold_m * 1000.0 + 1e-9) {
            report_.post_turn_verification_failure_count++;
            return front_blocked("post_turn_front_blocked", now_s);
        }
        if (!stable.left.usable_for_mapping) {
            report_.post_turn_verification_failure_count++;
            return wall_lost("post_turn_left_wall_lost", now_s);
        }
        if (!stable.right.usable_for_mapping) {
            report_.post_turn_verification_failure_count++;
            return fail("post_turn_right_sample_invalid", now_s);
        }
        const auto full = make_stable_snapshot(base, stable);
        const auto before = application_.core().report().current_map_revision;
        const auto pose_success_before =
            application_.core().report().measurement_pose_set_success_count;
        const auto verified = application_.step(
            full, now_s, {}, ActiveObservationControl::None, false);
        const auto after = application_.core().report().current_map_revision;
        if (!verified.ok || after != before ||
            application_.core().report().measurement_pose_set_success_count <=
                pose_success_before ||
            !application_.core().localization_ready()) {
            report_.post_turn_verification_failure_count++;
            if (after != before) report_.map_writes_during_turn_or_verify++;
            return fail("post_turn_verification_core_rejected", now_s);
        }
        pending_replay_odom_samples_.push_back({full.wheel, full.imu});
        post_turn_verified_since_last_commit_ = true;
        transition(StopGoMappingControllerState::IssueForwardStep);
        return true;
    }

    bool check_left_wall_limit(double now_s) {
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return complete("max_steps_or_distance_reached", now_s);
        }
        if (wall_model_.valid && pending_decision_.action != LeftWallControlAction::NoTurn) {
            transition(StopGoMappingControllerState::IssueTurnCorrection);
        } else {
            transition(StopGoMappingControllerState::IssueForwardStep);
        }
        return true;
    }

    static std::uint64_t monotonic_us(double now_s) {
        return static_cast<std::uint64_t>(std::llround(now_s * 1e6));
    }

    static RobotSlamSensorSnapshot make_odom_only(RobotSlamSensorSnapshot snapshot) {
        snapshot.has_tof = false;
        snapshot.has_multi_tof = false;
        snapshot.tof = {};
        snapshot.multi_tof = {};
        return snapshot;
    }

    RobotSlamSensorSnapshot make_stable_snapshot(
        const RobotSlamSensorSnapshot &base,
        const StableThreeTofSample &stable) const {
        RobotSlamSensorSnapshot snapshot = base;
        snapshot.has_tof = false;
        snapshot.has_multi_tof = true;
        snapshot.multi_tof = {};
        snapshot.multi_tof.has_front = stable.front.usable_for_mapping;
        snapshot.multi_tof.has_left = stable.left.usable_for_mapping;
        snapshot.multi_tof.has_right = stable.right.usable_for_mapping;
        snapshot.multi_tof.valid_tof_count = 0;
        auto fill = [](const StableTofReading &reading,
                       ScalarTofSnapshotFrame &frame,
                       const char *id, double yaw) {
            frame.distance_mm = reading.distance_mm;
            frame.distance_m = reading.distance_mm / 1000.0;
            frame.confidence = reading.confidence;
            frame.effective_timestamp_s = reading.sample_timestamp_us / 1e6;
            frame.protocol_valid = reading.valid;
            frame.usable_for_mapping = reading.usable_for_mapping;
            frame.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
            frame.mount_yaw_rad = yaw;
            frame.frame_id = id;
            frame.source = "stop_go_stable_three_tof";
            frame.reason = reading.rejection_reason;
        };
        fill(stable.front, snapshot.multi_tof.front, "tof_front_frame", 0.0);
        fill(stable.left, snapshot.multi_tof.left, "tof_left_frame", 1.5707963267948966);
        fill(stable.right, snapshot.multi_tof.right, "tof_right_frame", -1.5707963267948966);
        snapshot.multi_tof.valid_tof_count =
            static_cast<int>(snapshot.multi_tof.has_front) +
            static_cast<int>(snapshot.multi_tof.has_left) +
            static_cast<int>(snapshot.multi_tof.has_right);
        snapshot.multi_tof.synchronized_time_s =
            std::max({stable.front.sample_timestamp_us,
                      stable.left.sample_timestamp_us,
                      stable.right.sample_timestamp_us}) / 1e6;
        snapshot.multi_tof.source = "stop_go_stable_three_tof";
        snapshot.multi_tof.multi_tof_imu_dt_s =
            snapshot.multi_tof.synchronized_time_s - snapshot.imu.timestamp_s;
        snapshot.multi_tof.multi_tof_wheel_dt_s =
            snapshot.multi_tof.synchronized_time_s - snapshot.wheel.timestamp_s;
        return snapshot;
    }

    bool acquire_and_commit(const RobotSlamSensorSnapshot &base,
                            double now_s, std::size_t cycle) {
        transition(StopGoMappingControllerState::AcquireThreeTof);
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.usable_for_mapping) {
            return fail("stable_three_tof_unusable", now_s);
        }
        transition(StopGoMappingControllerState::CommitMappingSample);
        const auto full = make_stable_snapshot(base, stable);
        const std::uint64_t before = application_.core().report().current_map_revision;
        const auto committed = application_.step(full, now_s);
        const std::uint64_t after = application_.core().report().current_map_revision;
        if (!committed.ok || after <= before) {
            return fail("stable_snapshot_map_commit_failed:" + committed.reason, now_s);
        }
        report_.map_commits++;
        report_.completed_steps += cycle == 0 ? 0 : 1;
        if (cycle != 0) {
            total_distance_mm_ += last_motion_.actual_distance_mm;
            report_.commands_completed++;
        }
        StopGoReplayRecord replay;
        replay.cycle_index = cycle;
        replay.command_id = cycle == 0 ? 0 : command_.command_id;
        replay.motion = cycle == 0 ? RelativeMotionStepResult{} : last_motion_;
        replay.snapshot = full;
        replay.stable_sample = stable;
        replay.map_revision_before = before;
        replay.map_revision_after = after;
        replay.motor_settled_timestamp_us = cycle == 0
            ? 0 : monotonic_us(last_motion_.final_feedback.wheel.timestamp_s);
        replay.mapping_ready_timestamp_us = gate_.last_result().stable_timestamp_us;
        replay.settle_frame_count = gate_.last_result().consecutive_frames;
        replay.settle_duration_ms = gate_.last_result().stable_duration_ms;
        replay.odom_pose = application_.core().report().last_odom_pose;
        replay.map_from_odom =
            application_.core().frame_state().current_map_from_odom().map_T_odom;
        replay.controller_state = StopGoMappingControllerState::CommitMappingSample;
        report_.replay_records.push_back(replay);
        write_log(replay, now_s);
        transition(StopGoMappingControllerState::CheckLimit);
        return check_limit(now_s);
    }

    bool check_limit(double now_s) {
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return complete("max_steps_or_distance_reached", now_s);
        }
        transition(StopGoMappingControllerState::IssueForwardStep);
        return true;
    }

    bool complete(const char *reason, double) {
        motion_.stop(0.0);
        transition(StopGoMappingControllerState::Completed);
        report_.ok = true;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        report_.frame_transform_epoch = application_.core().frame_transform_epoch();
        return true;
    }

    bool fail(const std::string &reason, double now_s) {
        // Stop precedes any error/logging result.  The port implementation is
        // idempotent, so this is safe for a failed submit and a prior stop.
        motion_.stop(now_s);
        transition(StopGoMappingControllerState::Fault);
        report_.ok = false;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        report_.frame_transform_epoch = application_.core().frame_transform_epoch();
        return false;
    }

    void transition(StopGoMappingControllerState next) {
        state_ = next;
        report_.state_trace.push_back(to_string(next));
    }

    void write_log(const StopGoReplayRecord &record, double now_s) {
        if (!log_.is_open()) return;
        const auto &s = record.snapshot;
        const auto write_raw_samples = [this](const char *name,
                                               const StableTofReading &reading) {
            log_ << ",\"" << name << "\":[";
            for (std::size_t index = 0; index < reading.raw_frame_count; ++index) {
                if (index != 0) log_ << ",";
                const auto &raw = reading.raw_frames[index];
                log_ << "{\"valid\":" << (raw.frame.valid ? 1 : 0)
                     << ",\"distance_mm\":" << raw.frame.distance_mm
                     << ",\"confidence\":" << static_cast<int>(raw.frame.confidence)
                     << ",\"request_start_us\":" << raw.request_start_us
                     << ",\"response_received_us\":" << raw.response_received_us
                     << ",\"sample_timestamp_us\":" << raw.sample_timestamp_us << "}";
            }
            log_ << "]";
        };
        log_ << "{\"cycle_index\":" << record.cycle_index
             << ",\"command_id\":" << record.command_id
             << ",\"action\":\"" << to_string(record.motion.action)
             << "\",\"requested_amount\":" << record.motion.requested_amount
             << ",\"motion_state\":\"" << to_string(record.motion.state)
             << "\",\"actual_distance_mm\":" << record.motion.actual_distance_mm
             << ",\"actual_angle_deg\":" << record.motion.actual_angle_deg
             << ",\"error_code\":" << record.motion.error_code
             << ",\"reason\":\"" << record.motion.reason << "\""
             << ",\"now_s\":" << now_s
             << ",\"left_ticks\":" << s.wheel.left_ticks
             << ",\"right_ticks\":" << s.wheel.right_ticks
             << ",\"left_rpm\":" << s.wheel.left_rpm
             << ",\"right_rpm\":" << s.wheel.right_rpm
             << ",\"pair_skew_us\":" << s.wheel.pair_skew_s * 1e6
             << ",\"gyro_z\":" << s.imu.yaw_rate_rad_s
             << ",\"motor_settled_timestamp_us\":" << record.motor_settled_timestamp_us
             << ",\"mapping_ready_timestamp_us\":" << record.mapping_ready_timestamp_us
             << ",\"settle_frame_count\":" << record.settle_frame_count
             << ",\"settle_duration_ms\":" << record.settle_duration_ms
             << ",\"map_revision_before\":" << record.map_revision_before
             << ",\"map_revision_after\":" << record.map_revision_after
             << ",\"front_mm\":" << record.stable_sample.front.distance_mm
             << ",\"left_mm\":" << record.stable_sample.left.distance_mm
             << ",\"right_mm\":" << record.stable_sample.right.distance_mm
             << ",\"front_selected_us\":" << record.stable_sample.front.sample_timestamp_us
             << ",\"left_selected_us\":" << record.stable_sample.left.sample_timestamp_us
             << ",\"right_selected_us\":" << record.stable_sample.right.sample_timestamp_us;
        write_raw_samples("front_raw_samples", record.stable_sample.front);
        write_raw_samples("left_raw_samples", record.stable_sample.left);
        write_raw_samples("right_raw_samples", record.stable_sample.right);
        log_ << ",\"frame_transform_epoch\":" << record.frame_transform_epoch
             << ",\"left_wall_point_accepted\":"
             << (record.left_wall_point_accepted ? 1 : 0)
             << ",\"wall_point_x_m\":" << record.left_wall_point.x_m
             << ",\"wall_point_y_m\":" << record.left_wall_point.y_m
             << ",\"wall_model_valid\":" << (record.wall_model.valid ? 1 : 0)
             << ",\"wall_heading_rad\":" << record.wall_model.wall_heading_rad
             << ",\"signed_wall_distance_m\":"
             << record.wall_model.signed_base_to_wall_distance_m
             << ",\"wall_fit_baseline_m\":" << record.wall_model.baseline_m
             << ",\"wall_fit_rms_m\":" << record.wall_model.rms_residual_m
             << ",\"wall_fit_input_points\":" << record.wall_model.input_point_count
             << ",\"wall_fit_inliers\":" << record.wall_model.inlier_point_count
             << ",\"heading_error_rad\":" << record.control_decision.heading_error_rad
             << ",\"distance_error_m\":" << record.control_decision.distance_error_m
             << ",\"distance_bias_rad\":" << record.control_decision.distance_bias_rad
             << ",\"turn_error_rad\":" << record.control_decision.turn_error_rad
             << ",\"control_action\":\""
             << to_string(record.control_decision.action) << "\""
             << ",\"correction_amount_deg\":"
             << record.control_decision.correction_amount_deg
             << ",\"post_turn_verified\":"
             << (record.post_turn_verified ? 1 : 0)
             << ",\"corner_candidate\":" << (record.corner_candidate ? 1 : 0)
             << ",\"corner_confirmation\":" << (record.corner_confirmation ? 1 : 0)
             << ",\"corner_confirmed\":" << (record.corner_confirmed ? 1 : 0)
             << ",\"corner_clearance_passed\":" << (record.corner_right_clearance_passed ? 1 : 0)
             << ",\"corner_main_turn\":" << (record.corner_main_turn ? 1 : 0)
             << ",\"corner_residual_correction\":" << (record.corner_residual_correction ? 1 : 0)
             << ",\"post_corner_sensor_verified\":" << (record.post_corner_sensor_verified ? 1 : 0)
             << ",\"wall_segment_id\":" << record.wall_segment_id
             << ",\"corner_transition_id\":" << record.corner_transition_id
             << ",\"corner_main_command_id\":" << record.corner_main_command_id
             << ",\"pre_turn_odom_yaw_rad\":" << record.pre_turn_odom_yaw_rad
             << ",\"post_turn_odom_yaw_rad\":" << record.post_turn_odom_yaw_rad
             << ",\"actual_turn_delta_rad\":" << record.actual_turn_delta_rad
             << ",\"turn_residual_rad\":" << record.turn_residual_rad
             << ",\"rectangle_mode\":" << (report_.rectangle_mode ? 1 : 0)
             << ",\"run_start_anchor_valid\":"
             << (report_.run_start_anchor.valid ? 1 : 0)
             << ",\"run_anchor_hash\":" << report_.run_anchor_hash
             << ",\"corner_transition_count\":" << record.corner_transition_count
             << ",\"closure_candidate\":" << (record.closure_candidate ? 1 : 0)
             << ",\"closure_confirmation_pass\":"
             << (record.closure_confirmation_pass ? 1 : 0)
             << ",\"final_mapping_commit\":"
             << (record.final_mapping_commit ? 1 : 0)
             << ",\"final_map_revision\":" << report_.map_revision;
        log_ << ",\"controller_state\":\"" << to_string(record.controller_state)
             << "\"}\n";
        if (!log_) report_.log_error = "stop_go_log_write_failed";
    }

    RobotSlamApplication &application_;
    RelativeMotionStepPort &motion_;
    SnapshotReader snapshot_reader_;
    StableThreeTofSampler sampler_;
    StopGoMappingControllerConfig config_;
    MappingSettleGate gate_;
    LeftWallLineEstimator wall_estimator_;
    StopGoMappingControllerState state_ = StopGoMappingControllerState::Idle;
    StopGoMappingRunReport report_;
    RelativeMotionStepCommand command_;
    RelativeMotionStepResult last_motion_;
    std::uint64_t next_command_id_ = 1;
    double total_distance_mm_ = 0.0;
    std::vector<LeftWallPoint> wall_points_;
    LeftWallModel wall_model_;
    LeftWallControlDecision pending_decision_;
    std::uint64_t wall_epoch_ = 0;
    std::size_t consecutive_wall_misses_ = 0;
    std::size_t stationary_resample_attempts_ = 0;
    std::size_t front_invalid_samples_ = 0;
    double desired_distance_m_ = 0.15;
    bool desired_distance_captured_ = false;
    bool have_control_metrics_ = false;
    std::vector<StopGoReplayOdomSample> pending_replay_odom_samples_;
    bool post_turn_verified_since_last_commit_ = false;
    bool corner_candidate_pending_ = false;
    bool corner_confirmed_ = false;
    bool main_turn_finished_ = false;
    bool post_turn_sensor_verified_ = false;
    bool new_wall_mode_ = false;
    bool right_clearance_passed_ = false;
    std::size_t corner_confirmation_attempts_ = 0;
    std::size_t corner_confirmation_hits_ = 0;
    std::size_t right_clearance_attempts_ = 0;
    std::size_t post_turn_sensor_attempts_ = 0;
    std::size_t new_wall_resample_attempts_ = 0;
    std::size_t new_wall_acquisition_steps_ = 0;
    std::size_t corner_residual_corrections_ = 0;
    std::uint64_t wall_segment_id_ = 1;
    std::uint64_t corner_transition_id_ = 0;
    std::uint64_t corner_main_command_id_ = 0;
    std::uint64_t corner_residual_command_id_ = 0;
    double pre_turn_odom_yaw_rad_ = 0.0;
    double pre_turn_map_yaw_rad_ = 0.0;
    double post_main_odom_yaw_rad_ = 0.0;
    double verified_turn_delta_rad_ = 0.0;
    double final_turn_error_rad_ = 0.0;
    double pending_corner_residual_rad_ = 0.0;
    std::uint64_t segment_wall_point_hash_ = 1469598103934665603ULL;
    std::uint64_t segment_wall_model_hash_ = 1469598103934665603ULL;
    std::uint64_t old_wall_points_hash_ = 1469598103934665603ULL;
    std::uint64_t old_wall_models_hash_ = 1469598103934665603ULL;
    RobotPose2D previous_odom_pose_;
    bool previous_odom_pose_valid_ = false;
    double total_odom_travel_distance_m_ = 0.0;
    std::size_t segment_forward_steps_ = 0;
    std::size_t forward_steps_since_last_corner_ = 0;
    std::size_t current_corner_follow_steps_ = 0;
    std::size_t current_corner_main_turn_command_count_ = 0;
    bool corner_detector_armed_ = false;
    std::size_t corner_rearm_count_ = 0;
    bool pending_rectangle_transition_completion_ = false;
    bool post_fourth_search_active_ = false;
    bool closure_gate_complete_ = false;
    std::size_t closure_confirmation_attempts_ = 0;
    std::size_t closure_confirmation_passes_ = 0;
    std::size_t closure_confirmation_consecutive_passes_ = 0;
    bool closure_retry_requires_progress_ = false;
    std::size_t closure_retry_forward_step_baseline_ = 0;
    double closure_retry_odom_distance_baseline_m_ = 0.0;
    StableTofReading last_stable_front_;
    bool pending_replay_corner_candidate_ = false;
    bool pending_replay_corner_confirmation_ = false;
    bool pending_replay_corner_clearance_ = false;
    bool pending_replay_corner_main_turn_ = false;
    bool pending_replay_corner_residual_ = false;
    bool pending_replay_corner_residual_right_ = false;
    bool pending_replay_post_corner_verify_ = false;
    std::ofstream log_;
};

} // namespace robot_slamd
