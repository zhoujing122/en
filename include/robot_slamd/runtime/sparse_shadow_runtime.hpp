#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/runtime/slam_runtime_mode.hpp"
#include "robot_slamd/runtime/sparse_slam_runtime_core.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

namespace robot_slamd {

struct SparseShadowRuntimeReport {
    bool ok = false;
    std::string runtime_mode = "sparse_shadow";
    std::string sensor_source = "deterministic_simulation";

    bool startup_zero_used = false;
    bool initial_yaw_measured_by_imu = false;
    bool initial_yaw_defined_by_startup_frame = false;
    bool map_from_odom_assumed_identity = false;
    bool runtime_core_constructed = false;
    bool initialization_attempted = false;
    bool initialization_complete = false;
    std::string initialization_status = "uninitialized";
    std::string map_startup_mode = "create_new";
    std::string initial_pose_mode = "startup_zero";
    bool configured_pose_used = false;
    double gyro_bias_rad_s = 0.0;
    std::size_t gyro_sample_count = 0;
    bool stationary_check_passed = false;
    bool wheel_baseline_established = false;
    bool map_from_odom_initialized = false;
    double map_from_odom_x_m = 0.0;
    double map_from_odom_y_m = 0.0;
    double map_from_odom_yaw_rad = 0.0;

    bool sensor_port_constructed = false;
    bool wheel_imu_estimator_constructed = false;
    bool sparse_backend_constructed = false;

    std::size_t sensor_snapshot_count = 0;
    std::size_t wheel_imu_update_attempt_count = 0;
    std::size_t wheel_imu_update_success_count = 0;
    std::size_t wheel_imu_update_reject_count = 0;

    std::size_t backend_update_attempt_count = 0;
    std::size_t backend_accept_count = 0;
    std::size_t backend_reject_count = 0;
    std::size_t backend_fault_count = 0;

    std::size_t predicted_pose_handoff_count = 0;
    std::size_t missing_predicted_pose_count = 0;
    std::size_t native_multi_tof_snapshot_count = 0;
    std::size_t pose_buffer_append_count = 0;
    std::size_t pose_buffer_reject_count = 0;
    std::size_t pose_buffer_size = 0;
    std::size_t front_pose_lookup_success_count = 0;
    std::size_t left_pose_lookup_success_count = 0;
    std::size_t right_pose_lookup_success_count = 0;
    std::size_t measurement_pose_set_success_count = 0;
    std::size_t measurement_pose_set_reject_count = 0;
    std::size_t map_update_before_initialization_count = 0;
    std::size_t matcher_attempt_count = 0;
    std::size_t keyframe_attempt_count = 0;
    std::size_t pose_correction_attempt_count = 0;

    std::string active_bundle_state = "idle";
    std::uint64_t bundle_id = 0;
    std::size_t bundle_begin_count = 0;
    std::size_t bundle_end_motion_count = 0;
    std::size_t bundle_freeze_attempt_count = 0;
    std::size_t bundle_freeze_success_count = 0;
    std::size_t bundle_abort_count = 0;
    std::size_t bundle_discard_count = 0;
    std::size_t bundle_invalid_transition_count = 0;
    std::size_t bundle_snapshot_attempt_count = 0;
    std::size_t bundle_snapshot_accept_count = 0;
    std::size_t bundle_snapshot_reject_count = 0;
    std::size_t bundle_front_ray_count = 0;
    std::size_t bundle_left_ray_count = 0;
    std::size_t bundle_right_ray_count = 0;
    std::size_t bundle_hit_ray_count = 0;
    std::size_t bundle_no_return_ray_count = 0;
    std::size_t bundle_invalid_ray_count = 0;
    std::size_t bundle_unspecified_ray_count = 0;
    std::size_t bundle_matchable_ray_count = 0;
    double bundle_start_timestamp_s = 0.0;
    double bundle_end_timestamp_s = 0.0;
    double bundle_duration_s = 0.0;
    double bundle_accumulated_abs_yaw_rad = 0.0;
    double bundle_yaw_span_rad = 0.0;
    std::uint64_t reference_map_revision = 0;
    std::uint64_t current_map_revision = 0;
    std::size_t reference_map_cell_count = 0;
    std::size_t map_commit_blocked_count = 0;
    std::size_t map_update_during_bundle_count = 0;
    std::size_t odom_update_during_motion_count = 0;
    std::size_t odom_update_during_settle_count = 0;
    std::size_t odom_update_after_freeze_count = 0;
    std::size_t tof_snapshot_during_motion_count = 0;
    std::size_t tof_snapshot_during_settle_count = 0;
    std::size_t measurement_pose_set_during_motion_count = 0;
    std::size_t measurement_pose_set_during_settle_count = 0;
    std::size_t settle_update_count = 0;
    std::size_t settle_reset_count = 0;
    std::size_t settle_consecutive_sample_count = 0;
    double settle_stable_duration_s = 0.0;
    std::size_t settle_success_count = 0;
    bool frozen_bundle_available = false;
    bool frozen_bundle_immutable = true;
    std::string last_bundle_fault = "none";
    std::size_t reference_snapshot_capture_attempt_count = 0;
    std::size_t reference_snapshot_capture_success_count = 0;
    std::size_t reference_snapshot_capture_reject_count = 0;
    std::uint64_t reference_snapshot_revision = 0;
    std::size_t reference_snapshot_cell_count = 0;
    std::size_t reference_snapshot_occupied_cell_count = 0;
    std::size_t reference_snapshot_free_cell_count = 0;
    std::size_t reference_snapshot_uncertain_cell_count = 0;
    double reference_snapshot_resolution_m = 0.0;
    bool reference_snapshot_ready = false;
    std::size_t matcher_input_prepare_attempt_count = 0;
    std::size_t matcher_input_prepare_success_count = 0;
    std::size_t matcher_input_prepare_reject_count = 0;
    bool matcher_input_ready = false;
    std::uint64_t matcher_input_bundle_id = 0;
    std::uint64_t matcher_input_reference_revision = 0;
    std::size_t matcher_input_bundle_frame_count = 0;
    std::size_t matcher_input_matchable_ray_count = 0;
    std::string matcher_input_requested_mode = "yaw_only";
    std::string matcher_input_status = "not_prepared";
    bool matcher_executed = false;
    std::size_t matcher_execution_count = 0;
    std::size_t matcher_accept_count = 0;
    std::size_t matcher_reject_count = 0;
    std::size_t matcher_fault_count = 0;
    std::size_t matcher_evaluated_candidate_count = 0;
    std::size_t matcher_coarse_candidate_count = 0;
    std::size_t matcher_fine_candidate_count = 0;
    std::size_t matcher_used_ray_count = 0;
    std::size_t matcher_known_cell_count = 0;
    std::size_t matcher_unknown_cell_count = 0;
    double matcher_unknown_ratio = 0.0;
    std::size_t matcher_contradiction_count = 0;
    double matcher_best_yaw_rad = 0.0;
    double matcher_best_score = 0.0;
    bool matcher_runner_up_available = false;
    double matcher_runner_up_yaw_rad = 0.0;
    double matcher_runner_up_score = 0.0;
    double matcher_score_margin = 0.0;
    double matcher_score_min = 0.0;
    double matcher_score_max = 0.0;
    double matcher_score_mean = 0.0;
    double matcher_score_range = 0.0;
    bool matcher_best_at_search_edge = false;
    bool matcher_flat_curve = false;
    bool matcher_multimodal = false;
    bool matcher_accepted = false;
    std::string matcher_status = "not_run";
    std::string matcher_rejection_reason = "none";
    double matcher_delta_yaw_rad = 0.0;
    bool matcher_proposal_ready = false;
    RobotPose2D map_from_odom_before_match;
    RobotPose2D proposed_map_from_odom;
    RobotPose2D map_from_odom_after_match;
    std::string last_matcher_input_fault = "none";

    bool native_multi_tof_consumed = false;
    bool measurement_time_pose_consumed = false;
    bool single_pose_fallback_consumed = false;
    bool legacy_projection_consumed = false;

    bool old_chunked_grid_constructed = false;
    bool old_matcher_constructed = false;
    bool old_correction_constructed = false;

    bool real_hardware_accessed = false;
    bool real_motion_attempted = false;
    bool real_map_write_attempted = false;
    bool real_pose_writeback_attempted = false;

    RobotPose2D last_odom_pose;
    RobotPose2D last_predicted_map_pose;
    std::size_t sparse_map_cell_count = 0;

    std::string last_fault = "none";
    std::string last_message;
    std::string summary;
};

inline std::string write_sparse_shadow_runtime_report(
    const SparseShadowRuntimeReport &report) {
    std::ostringstream out;
    out << "Sparse Shadow Runtime Report\n";
    out << "ok=" << bool_yaml(report.ok) << "\n";
    out << "runtime_mode=" << report.runtime_mode << "\n";
    out << "sensor_source=" << report.sensor_source << "\n";
    out << "startup_zero_used=" << bool_yaml(report.startup_zero_used) << "\n";
    out << "initial_yaw_measured_by_imu="
        << bool_yaml(report.initial_yaw_measured_by_imu) << "\n";
    out << "initial_yaw_defined_by_startup_frame="
        << bool_yaml(report.initial_yaw_defined_by_startup_frame) << "\n";
    out << "map_from_odom_assumed_identity="
        << bool_yaml(report.map_from_odom_assumed_identity) << "\n";
    out << "runtime_core_constructed="
        << bool_yaml(report.runtime_core_constructed) << "\n";
    out << "initialization_attempted="
        << bool_yaml(report.initialization_attempted) << "\n";
    out << "initialization_complete="
        << bool_yaml(report.initialization_complete) << "\n";
    out << "initialization_status=" << report.initialization_status << "\n";
    out << "map_startup_mode=" << report.map_startup_mode << "\n";
    out << "initial_pose_mode=" << report.initial_pose_mode << "\n";
    out << "configured_pose_used="
        << bool_yaml(report.configured_pose_used) << "\n";
    out << "gyro_bias_rad_s=" << report.gyro_bias_rad_s << "\n";
    out << "gyro_sample_count=" << report.gyro_sample_count << "\n";
    out << "stationary_check_passed="
        << bool_yaml(report.stationary_check_passed) << "\n";
    out << "wheel_baseline_established="
        << bool_yaml(report.wheel_baseline_established) << "\n";
    out << "map_from_odom_initialized="
        << bool_yaml(report.map_from_odom_initialized) << "\n";
    out << "map_from_odom=" << report.map_from_odom_x_m << ","
        << report.map_from_odom_y_m << ","
        << report.map_from_odom_yaw_rad << "\n";
    out << "sensor_port_constructed="
        << bool_yaml(report.sensor_port_constructed) << "\n";
    out << "wheel_imu_estimator_constructed="
        << bool_yaml(report.wheel_imu_estimator_constructed) << "\n";
    out << "sparse_backend_constructed="
        << bool_yaml(report.sparse_backend_constructed) << "\n";
    out << "sensor_snapshot_count=" << report.sensor_snapshot_count << "\n";
    out << "wheel_imu_update_attempt_count="
        << report.wheel_imu_update_attempt_count << "\n";
    out << "wheel_imu_update_success_count="
        << report.wheel_imu_update_success_count << "\n";
    out << "wheel_imu_update_reject_count="
        << report.wheel_imu_update_reject_count << "\n";
    out << "backend_update_attempt_count="
        << report.backend_update_attempt_count << "\n";
    out << "backend_accept_count=" << report.backend_accept_count << "\n";
    out << "backend_reject_count=" << report.backend_reject_count << "\n";
    out << "backend_fault_count=" << report.backend_fault_count << "\n";
    out << "predicted_pose_handoff_count="
        << report.predicted_pose_handoff_count << "\n";
    out << "missing_predicted_pose_count="
        << report.missing_predicted_pose_count << "\n";
    out << "native_multi_tof_snapshot_count="
        << report.native_multi_tof_snapshot_count << "\n";
    out << "pose_buffer_append_count="
        << report.pose_buffer_append_count << "\n";
    out << "pose_buffer_reject_count="
        << report.pose_buffer_reject_count << "\n";
    out << "pose_buffer_size=" << report.pose_buffer_size << "\n";
    out << "front_pose_lookup_success_count="
        << report.front_pose_lookup_success_count << "\n";
    out << "left_pose_lookup_success_count="
        << report.left_pose_lookup_success_count << "\n";
    out << "right_pose_lookup_success_count="
        << report.right_pose_lookup_success_count << "\n";
    out << "measurement_pose_set_success_count="
        << report.measurement_pose_set_success_count << "\n";
    out << "measurement_pose_set_reject_count="
        << report.measurement_pose_set_reject_count << "\n";
    out << "map_update_before_initialization_count="
        << report.map_update_before_initialization_count << "\n";
    out << "matcher_attempt_count=" << report.matcher_attempt_count << "\n";
    out << "keyframe_attempt_count=" << report.keyframe_attempt_count << "\n";
    out << "pose_correction_attempt_count="
        << report.pose_correction_attempt_count << "\n";
    out << "active_bundle_state=" << report.active_bundle_state << "\n";
    out << "bundle_id=" << report.bundle_id << "\n";
    out << "bundle_begin_count=" << report.bundle_begin_count << "\n";
    out << "bundle_end_motion_count=" << report.bundle_end_motion_count << "\n";
    out << "bundle_freeze_attempt_count="
        << report.bundle_freeze_attempt_count << "\n";
    out << "bundle_freeze_success_count="
        << report.bundle_freeze_success_count << "\n";
    out << "bundle_abort_count=" << report.bundle_abort_count << "\n";
    out << "bundle_discard_count=" << report.bundle_discard_count << "\n";
    out << "bundle_invalid_transition_count="
        << report.bundle_invalid_transition_count << "\n";
    out << "bundle_snapshot_attempt_count="
        << report.bundle_snapshot_attempt_count << "\n";
    out << "bundle_snapshot_accept_count="
        << report.bundle_snapshot_accept_count << "\n";
    out << "bundle_snapshot_reject_count="
        << report.bundle_snapshot_reject_count << "\n";
    out << "bundle_front_ray_count=" << report.bundle_front_ray_count << "\n";
    out << "bundle_left_ray_count=" << report.bundle_left_ray_count << "\n";
    out << "bundle_right_ray_count=" << report.bundle_right_ray_count << "\n";
    out << "bundle_hit_ray_count=" << report.bundle_hit_ray_count << "\n";
    out << "bundle_no_return_ray_count="
        << report.bundle_no_return_ray_count << "\n";
    out << "bundle_invalid_ray_count="
        << report.bundle_invalid_ray_count << "\n";
    out << "bundle_unspecified_ray_count="
        << report.bundle_unspecified_ray_count << "\n";
    out << "bundle_matchable_ray_count="
        << report.bundle_matchable_ray_count << "\n";
    out << "bundle_start_timestamp_s="
        << report.bundle_start_timestamp_s << "\n";
    out << "bundle_end_timestamp_s=" << report.bundle_end_timestamp_s << "\n";
    out << "bundle_duration_s=" << report.bundle_duration_s << "\n";
    out << "bundle_accumulated_abs_yaw_rad="
        << report.bundle_accumulated_abs_yaw_rad << "\n";
    out << "bundle_yaw_span_rad=" << report.bundle_yaw_span_rad << "\n";
    out << "reference_map_revision=" << report.reference_map_revision << "\n";
    out << "current_map_revision=" << report.current_map_revision << "\n";
    out << "reference_map_cell_count="
        << report.reference_map_cell_count << "\n";
    out << "map_commit_blocked_count="
        << report.map_commit_blocked_count << "\n";
    out << "map_update_during_bundle_count="
        << report.map_update_during_bundle_count << "\n";
    out << "odom_update_during_motion_count="
        << report.odom_update_during_motion_count << "\n";
    out << "odom_update_during_settle_count="
        << report.odom_update_during_settle_count << "\n";
    out << "odom_update_after_freeze_count="
        << report.odom_update_after_freeze_count << "\n";
    out << "tof_snapshot_during_motion_count="
        << report.tof_snapshot_during_motion_count << "\n";
    out << "tof_snapshot_during_settle_count="
        << report.tof_snapshot_during_settle_count << "\n";
    out << "measurement_pose_set_during_motion_count="
        << report.measurement_pose_set_during_motion_count << "\n";
    out << "measurement_pose_set_during_settle_count="
        << report.measurement_pose_set_during_settle_count << "\n";
    out << "settle_update_count=" << report.settle_update_count << "\n";
    out << "settle_reset_count=" << report.settle_reset_count << "\n";
    out << "settle_consecutive_sample_count="
        << report.settle_consecutive_sample_count << "\n";
    out << "settle_stable_duration_s="
        << report.settle_stable_duration_s << "\n";
    out << "settle_success_count=" << report.settle_success_count << "\n";
    out << "frozen_bundle_available="
        << bool_yaml(report.frozen_bundle_available) << "\n";
    out << "frozen_bundle_immutable="
        << bool_yaml(report.frozen_bundle_immutable) << "\n";
    out << "last_bundle_fault=" << report.last_bundle_fault << "\n";
    out << "reference_snapshot_capture_attempt_count=" << report.reference_snapshot_capture_attempt_count << "\n";
    out << "reference_snapshot_capture_success_count=" << report.reference_snapshot_capture_success_count << "\n";
    out << "reference_snapshot_capture_reject_count=" << report.reference_snapshot_capture_reject_count << "\n";
    out << "reference_snapshot_revision=" << report.reference_snapshot_revision << "\n";
    out << "reference_snapshot_cell_count=" << report.reference_snapshot_cell_count << "\n";
    out << "reference_snapshot_occupied_cell_count=" << report.reference_snapshot_occupied_cell_count << "\n";
    out << "reference_snapshot_free_cell_count=" << report.reference_snapshot_free_cell_count << "\n";
    out << "reference_snapshot_uncertain_cell_count=" << report.reference_snapshot_uncertain_cell_count << "\n";
    out << "reference_snapshot_resolution_m=" << report.reference_snapshot_resolution_m << "\n";
    out << "reference_snapshot_ready=" << bool_yaml(report.reference_snapshot_ready) << "\n";
    out << "matcher_input_prepare_attempt_count=" << report.matcher_input_prepare_attempt_count << "\n";
    out << "matcher_input_prepare_success_count=" << report.matcher_input_prepare_success_count << "\n";
    out << "matcher_input_prepare_reject_count=" << report.matcher_input_prepare_reject_count << "\n";
    out << "matcher_input_ready=" << bool_yaml(report.matcher_input_ready) << "\n";
    out << "matcher_input_bundle_id=" << report.matcher_input_bundle_id << "\n";
    out << "matcher_input_reference_revision=" << report.matcher_input_reference_revision << "\n";
    out << "matcher_input_bundle_frame_count=" << report.matcher_input_bundle_frame_count << "\n";
    out << "matcher_input_matchable_ray_count=" << report.matcher_input_matchable_ray_count << "\n";
    out << "matcher_input_requested_mode=" << report.matcher_input_requested_mode << "\n";
    out << "matcher_input_status=" << report.matcher_input_status << "\n";
    out << "matcher_executed=" << bool_yaml(report.matcher_executed) << "\n";
    out << "matcher_execution_count=" << report.matcher_execution_count << "\n";
    out << "matcher_accept_count=" << report.matcher_accept_count << "\n";
    out << "matcher_reject_count=" << report.matcher_reject_count << "\n";
    out << "matcher_fault_count=" << report.matcher_fault_count << "\n";
    out << "matcher_evaluated_candidate_count=" << report.matcher_evaluated_candidate_count << "\n";
    out << "matcher_coarse_candidate_count=" << report.matcher_coarse_candidate_count << "\n";
    out << "matcher_fine_candidate_count=" << report.matcher_fine_candidate_count << "\n";
    out << "matcher_used_ray_count=" << report.matcher_used_ray_count << "\n";
    out << "matcher_known_cell_count=" << report.matcher_known_cell_count << "\n";
    out << "matcher_unknown_cell_count=" << report.matcher_unknown_cell_count << "\n";
    out << "matcher_unknown_ratio=" << report.matcher_unknown_ratio << "\n";
    out << "matcher_contradiction_count=" << report.matcher_contradiction_count << "\n";
    out << "matcher_best_yaw_rad=" << report.matcher_best_yaw_rad << "\n";
    out << "matcher_best_score=" << report.matcher_best_score << "\n";
    out << "matcher_runner_up_available=" << bool_yaml(report.matcher_runner_up_available) << "\n";
    out << "matcher_runner_up_yaw_rad=" << report.matcher_runner_up_yaw_rad << "\n";
    out << "matcher_runner_up_score=" << report.matcher_runner_up_score << "\n";
    out << "matcher_score_margin=" << report.matcher_score_margin << "\n";
    out << "matcher_score_min=" << report.matcher_score_min << "\n";
    out << "matcher_score_max=" << report.matcher_score_max << "\n";
    out << "matcher_score_mean=" << report.matcher_score_mean << "\n";
    out << "matcher_score_range=" << report.matcher_score_range << "\n";
    out << "matcher_best_at_search_edge=" << bool_yaml(report.matcher_best_at_search_edge) << "\n";
    out << "matcher_flat_curve=" << bool_yaml(report.matcher_flat_curve) << "\n";
    out << "matcher_multimodal=" << bool_yaml(report.matcher_multimodal) << "\n";
    out << "matcher_accepted=" << bool_yaml(report.matcher_accepted) << "\n";
    out << "matcher_status=" << report.matcher_status << "\n";
    out << "matcher_rejection_reason=" << report.matcher_rejection_reason << "\n";
    out << "matcher_delta_yaw_rad=" << report.matcher_delta_yaw_rad << "\n";
    out << "matcher_proposal_ready=" << bool_yaml(report.matcher_proposal_ready) << "\n";
    out << "map_from_odom_before_match_x_m=" << report.map_from_odom_before_match.x_m << "\n";
    out << "map_from_odom_before_match_y_m=" << report.map_from_odom_before_match.y_m << "\n";
    out << "map_from_odom_before_match_yaw_rad=" << report.map_from_odom_before_match.yaw_rad << "\n";
    out << "proposed_map_from_odom_x_m=" << report.proposed_map_from_odom.x_m << "\n";
    out << "proposed_map_from_odom_y_m=" << report.proposed_map_from_odom.y_m << "\n";
    out << "proposed_map_from_odom_yaw_rad=" << report.proposed_map_from_odom.yaw_rad << "\n";
    out << "map_from_odom_after_match_x_m=" << report.map_from_odom_after_match.x_m << "\n";
    out << "map_from_odom_after_match_y_m=" << report.map_from_odom_after_match.y_m << "\n";
    out << "map_from_odom_after_match_yaw_rad=" << report.map_from_odom_after_match.yaw_rad << "\n";
    out << "last_matcher_input_fault=" << report.last_matcher_input_fault << "\n";
    out << "native_multi_tof_consumed="
        << bool_yaml(report.native_multi_tof_consumed) << "\n";
    out << "measurement_time_pose_consumed="
        << bool_yaml(report.measurement_time_pose_consumed) << "\n";
    out << "single_pose_fallback_consumed="
        << bool_yaml(report.single_pose_fallback_consumed) << "\n";
    out << "legacy_projection_consumed="
        << bool_yaml(report.legacy_projection_consumed) << "\n";
    out << "old_chunked_grid_constructed="
        << bool_yaml(report.old_chunked_grid_constructed) << "\n";
    out << "old_matcher_constructed="
        << bool_yaml(report.old_matcher_constructed) << "\n";
    out << "old_correction_constructed="
        << bool_yaml(report.old_correction_constructed) << "\n";
    out << "real_hardware_accessed="
        << bool_yaml(report.real_hardware_accessed) << "\n";
    out << "real_motion_attempted="
        << bool_yaml(report.real_motion_attempted) << "\n";
    out << "real_map_write_attempted="
        << bool_yaml(report.real_map_write_attempted) << "\n";
    out << "real_pose_writeback_attempted="
        << bool_yaml(report.real_pose_writeback_attempted) << "\n";
    out << "last_odom_pose=" << report.last_odom_pose.x_m << ","
        << report.last_odom_pose.y_m << ","
        << report.last_odom_pose.yaw_rad << "\n";
    out << "last_predicted_map_pose="
        << report.last_predicted_map_pose.x_m << ","
        << report.last_predicted_map_pose.y_m << ","
        << report.last_predicted_map_pose.yaw_rad << "\n";
    out << "sparse_map_cell_count=" << report.sparse_map_cell_count << "\n";
    out << "last_fault=" << report.last_fault << "\n";
    out << "last_message=" << report.last_message << "\n";
    out << "summary=" << report.summary << "\n";
    return out.str();
}

class SparseShadowDeterministicSensorPort final : public RobotSlamSensorPort {
public:
    bool ready() const override { return true; }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        if (!std::isfinite(now_s) || now_s <= last_timestamp_s_) {
            now_s = last_timestamp_s_ + 0.10;
        }
        last_timestamp_s_ = now_s;
        sample_index_++;

        RobotSlamSensorSnapshot snapshot;
        snapshot.has_tof = false;
        snapshot.has_multi_tof = true;
        snapshot.multi_tof.has_front = true;
        snapshot.multi_tof.has_left = true;
        snapshot.multi_tof.has_right = true;
        snapshot.multi_tof.valid_tof_count = 3;
        snapshot.multi_tof.front = frame(1000, 0.0, "front", now_s);
        snapshot.multi_tof.left = frame(1200, kPi / 2.0, "left", now_s);
        snapshot.multi_tof.right = frame(1200, -kPi / 2.0, "right", now_s);
        snapshot.multi_tof.synchronized_time_s = now_s;
        snapshot.multi_tof.source = "sparse_shadow_deterministic_simulation";

        const bool startup_stationary_sample = sample_index_ == 1;
        snapshot.has_wheel = true;
        snapshot.wheel.timestamp_s = now_s;
        snapshot.wheel.linear_mps = startup_stationary_sample ? 0.0 : 0.05;
        snapshot.wheel.angular_rad_s = 0.0;
        snapshot.wheel.valid = true;

        snapshot.has_imu = true;
        snapshot.imu.timestamp_s = now_s;
        snapshot.imu.yaw_rate_rad_s = 0.0;
        snapshot.imu.az_mps2 = 9.8;
        return snapshot;
    }

    std::string status() const override {
        return "sparse_shadow_deterministic_sensor_ready";
    }

private:
    static ScalarTofSnapshotFrame frame(std::uint16_t distance_mm,
                                        double yaw_rad,
                                        const std::string &frame_id,
                                        double timestamp_s) {
        ScalarTofSnapshotFrame out;
        out.distance_mm = distance_mm;
        out.distance_m = static_cast<double>(distance_mm) / 1000.0;
        out.confidence = 100;
        out.echo_tag_u48 = 0x120000 + distance_mm;
        out.effective_timestamp_s = timestamp_s;
        out.protocol_valid = true;
        out.usable_for_mapping = true;
        out.full_fov_rad = 1.6 * kPi / 180.0;
        out.mount_yaw_rad = yaw_rad;
        out.frame_id = frame_id;
        out.source = "sparse_shadow_deterministic_simulation";
        return out;
    }

    double last_timestamp_s_ = 0.0;
    std::size_t sample_index_ = 0;
};

inline SlamBackendContractChecker sparse_shadow_backend_checker() {
    SlamBackendContractOptions options;
    options.require_tof = false;
    options.require_imu_or_wheel = true;
    options.allow_predicted_pose_missing = true;
    return SlamBackendContractChecker(options);
}

class SparseShadowRuntime final {
public:
    explicit SparseShadowRuntime(Config config)
        : config_(std::move(config)) {}

    SparseShadowRuntimeReport run(double duration_s,
                                  const std::string &run_dir) {
        SparseShadowRuntimeReport report;
        report.runtime_mode = "sparse_shadow";
        report.sensor_source = config_.sparse_shadow_sensor_source;
        report.initial_yaw_measured_by_imu = false;
        report.initial_yaw_defined_by_startup_frame = true;
        report.map_from_odom_assumed_identity = true;

        if (config_.sparse_shadow_sensor_source != "deterministic_simulation") {
            report.ok = false;
            report.last_fault = "sparse_shadow_sensor_source_unsupported";
            report.last_message = "SparseShadow hardware/replay source is fail-closed in M3-D2A0";
            report.summary = "sparse_shadow_runtime_rejected";
            write_report_file(run_dir, report);
            return report;
        }

        auto sensor_port = std::make_unique<SparseShadowDeterministicSensorPort>();
        report.sensor_port_constructed = true;

        SparseSlamRuntimeCore core(config_);
        const auto init_request = sparse_slam_initialization_request_from_config(config_);
        const auto init = core.initialize(init_request);
        (void)init;

        const double step_s = 0.10;
        const double clamped_duration_s =
            std::isfinite(duration_s) && duration_s > 0.0 ? duration_s : step_s;
        const int max_steps = std::max(2, std::min(20,
            static_cast<int>(std::ceil(clamped_duration_s / step_s))));

        for (int i = 0; i < max_steps; ++i) {
            const double now_s = static_cast<double>(i + 1) * step_s;
            auto snapshot = sensor_port->latest_snapshot(now_s);
            const auto step = core.step(snapshot, now_s);
            (void)step;
        }

        merge_core_report(core.report(), report);
        report.ok = report.sensor_port_constructed &&
                    report.runtime_core_constructed &&
                    report.initialization_complete &&
                    report.wheel_imu_estimator_constructed &&
                    report.sparse_backend_constructed &&
                    report.predicted_pose_handoff_count > 0 &&
                    report.backend_accept_count > 0 &&
                    report.native_multi_tof_consumed &&
                    report.measurement_time_pose_consumed &&
                    !report.single_pose_fallback_consumed &&
                    !report.legacy_projection_consumed;
        report.last_fault = report.ok ? "none" : report.last_fault;
        report.last_message = report.ok ? "sparse_shadow_runtime_ok"
                                        : report.last_message;
        report.summary = report.ok
            ? "Sparse Shadow wrapper reached sparse runtime core"
            : "Sparse Shadow wrapper did not reach sparse runtime core";
        write_report_file(run_dir, report);
        return report;
    }

private:
    static void merge_core_report(const SparseSlamRuntimeCoreReport &core,
                                  SparseShadowRuntimeReport &report) {
        report.runtime_core_constructed = core.runtime_core_constructed;
        report.initialization_attempted = core.initialization_attempted;
        report.initialization_complete = core.initialization_complete;
        report.initialization_status = core.initialization_status;
        report.map_startup_mode = core.map_startup_mode;
        report.initial_pose_mode = core.initial_pose_mode;
        report.startup_zero_used = core.startup_zero_used;
        report.configured_pose_used = core.configured_pose_used;
        report.initial_yaw_measured_by_imu = core.initial_yaw_measured_by_imu;
        report.initial_yaw_defined_by_startup_frame =
            core.initial_yaw_defined_by_startup_frame;
        report.gyro_bias_rad_s = core.gyro_bias_rad_s;
        report.gyro_sample_count = core.gyro_sample_count;
        report.stationary_check_passed = core.stationary_check_passed;
        report.wheel_baseline_established = core.wheel_baseline_established;
        report.map_from_odom_initialized = core.map_from_odom_initialized;
        report.map_from_odom_x_m = core.map_from_odom_x_m;
        report.map_from_odom_y_m = core.map_from_odom_y_m;
        report.map_from_odom_yaw_rad = core.map_from_odom_yaw_rad;
        report.wheel_imu_estimator_constructed =
            core.wheel_imu_estimator_constructed;
        report.sparse_backend_constructed = core.sparse_backend_constructed;
        report.sensor_snapshot_count = core.sensor_snapshot_count;
        report.wheel_imu_update_attempt_count = core.odom_update_attempt_count;
        report.wheel_imu_update_success_count = core.odom_update_success_count;
        report.wheel_imu_update_reject_count = core.odom_update_reject_count;
        report.backend_update_attempt_count = core.backend_update_attempt_count;
        report.backend_accept_count = core.backend_accept_count;
        report.backend_reject_count = core.backend_reject_count;
        report.backend_fault_count = core.backend_fault_count;
        report.predicted_pose_handoff_count = core.predicted_pose_handoff_count;
        report.missing_predicted_pose_count = core.missing_predicted_pose_count;
        report.native_multi_tof_snapshot_count = core.native_multi_tof_snapshot_count;
        report.pose_buffer_append_count = core.pose_buffer_append_count;
        report.pose_buffer_reject_count = core.pose_buffer_reject_count;
        report.pose_buffer_size = core.pose_buffer_size;
        report.front_pose_lookup_success_count = core.front_pose_lookup_success_count;
        report.left_pose_lookup_success_count = core.left_pose_lookup_success_count;
        report.right_pose_lookup_success_count = core.right_pose_lookup_success_count;
        report.measurement_pose_set_success_count =
            core.measurement_pose_set_success_count;
        report.measurement_pose_set_reject_count =
            core.measurement_pose_set_reject_count;
        report.map_update_before_initialization_count =
            core.map_update_before_initialization_count;
        report.matcher_attempt_count = core.matcher_attempt_count;
        report.keyframe_attempt_count = core.keyframe_attempt_count;
        report.pose_correction_attempt_count = core.pose_correction_attempt_count;
        report.active_bundle_state = core.active_bundle_state;
        report.bundle_id = core.bundle_id;
        report.bundle_begin_count = core.bundle_begin_count;
        report.bundle_end_motion_count = core.bundle_end_motion_count;
        report.bundle_freeze_attempt_count = core.bundle_freeze_attempt_count;
        report.bundle_freeze_success_count = core.bundle_freeze_success_count;
        report.bundle_abort_count = core.bundle_abort_count;
        report.bundle_discard_count = core.bundle_discard_count;
        report.bundle_invalid_transition_count = core.bundle_invalid_transition_count;
        report.bundle_snapshot_attempt_count = core.bundle_snapshot_attempt_count;
        report.bundle_snapshot_accept_count = core.bundle_snapshot_accept_count;
        report.bundle_snapshot_reject_count = core.bundle_snapshot_reject_count;
        report.bundle_front_ray_count = core.bundle_front_ray_count;
        report.bundle_left_ray_count = core.bundle_left_ray_count;
        report.bundle_right_ray_count = core.bundle_right_ray_count;
        report.bundle_hit_ray_count = core.bundle_hit_ray_count;
        report.bundle_no_return_ray_count = core.bundle_no_return_ray_count;
        report.bundle_invalid_ray_count = core.bundle_invalid_ray_count;
        report.bundle_unspecified_ray_count = core.bundle_unspecified_ray_count;
        report.bundle_matchable_ray_count = core.bundle_matchable_ray_count;
        report.bundle_start_timestamp_s = core.bundle_start_timestamp_s;
        report.bundle_end_timestamp_s = core.bundle_end_timestamp_s;
        report.bundle_duration_s = core.bundle_duration_s;
        report.bundle_accumulated_abs_yaw_rad = core.bundle_accumulated_abs_yaw_rad;
        report.bundle_yaw_span_rad = core.bundle_yaw_span_rad;
        report.reference_map_revision = core.reference_map_revision;
        report.current_map_revision = core.current_map_revision;
        report.reference_map_cell_count = core.reference_map_cell_count;
        report.map_commit_blocked_count = core.map_commit_blocked_count;
        report.map_update_during_bundle_count = core.map_update_during_bundle_count;
        report.odom_update_during_motion_count = core.odom_update_during_motion_count;
        report.odom_update_during_settle_count = core.odom_update_during_settle_count;
        report.odom_update_after_freeze_count = core.odom_update_after_freeze_count;
        report.tof_snapshot_during_motion_count = core.tof_snapshot_during_motion_count;
        report.tof_snapshot_during_settle_count = core.tof_snapshot_during_settle_count;
        report.measurement_pose_set_during_motion_count = core.measurement_pose_set_during_motion_count;
        report.measurement_pose_set_during_settle_count = core.measurement_pose_set_during_settle_count;
        report.settle_update_count = core.settle_update_count;
        report.settle_reset_count = core.settle_reset_count;
        report.settle_consecutive_sample_count = core.settle_consecutive_sample_count;
        report.settle_stable_duration_s = core.settle_stable_duration_s;
        report.settle_success_count = core.settle_success_count;
        report.frozen_bundle_available = core.frozen_bundle_available;
        report.frozen_bundle_immutable = core.frozen_bundle_immutable;
        report.last_bundle_fault = core.last_bundle_fault;
        report.reference_snapshot_capture_attempt_count = core.reference_snapshot_capture_attempt_count;
        report.reference_snapshot_capture_success_count = core.reference_snapshot_capture_success_count;
        report.reference_snapshot_capture_reject_count = core.reference_snapshot_capture_reject_count;
        report.reference_snapshot_revision = core.reference_snapshot_revision;
        report.reference_snapshot_cell_count = core.reference_snapshot_cell_count;
        report.reference_snapshot_occupied_cell_count = core.reference_snapshot_occupied_cell_count;
        report.reference_snapshot_free_cell_count = core.reference_snapshot_free_cell_count;
        report.reference_snapshot_uncertain_cell_count = core.reference_snapshot_uncertain_cell_count;
        report.reference_snapshot_resolution_m = core.reference_snapshot_resolution_m;
        report.reference_snapshot_ready = core.reference_snapshot_ready;
        report.matcher_input_prepare_attempt_count = core.matcher_input_prepare_attempt_count;
        report.matcher_input_prepare_success_count = core.matcher_input_prepare_success_count;
        report.matcher_input_prepare_reject_count = core.matcher_input_prepare_reject_count;
        report.matcher_input_ready = core.matcher_input_ready;
        report.matcher_input_bundle_id = core.matcher_input_bundle_id;
        report.matcher_input_reference_revision = core.matcher_input_reference_revision;
        report.matcher_input_bundle_frame_count = core.matcher_input_bundle_frame_count;
        report.matcher_input_matchable_ray_count = core.matcher_input_matchable_ray_count;
        report.matcher_input_requested_mode = core.matcher_input_requested_mode;
        report.matcher_input_status = core.matcher_input_status;
        report.matcher_executed = core.matcher_executed;
        report.matcher_execution_count = core.matcher_execution_count;
        report.matcher_accept_count = core.matcher_accept_count;
        report.matcher_reject_count = core.matcher_reject_count;
        report.matcher_fault_count = core.matcher_fault_count;
        report.matcher_evaluated_candidate_count = core.matcher_evaluated_candidate_count;
        report.matcher_coarse_candidate_count = core.matcher_coarse_candidate_count;
        report.matcher_fine_candidate_count = core.matcher_fine_candidate_count;
        report.matcher_used_ray_count = core.matcher_used_ray_count;
        report.matcher_known_cell_count = core.matcher_known_cell_count;
        report.matcher_unknown_cell_count = core.matcher_unknown_cell_count;
        report.matcher_unknown_ratio = core.matcher_unknown_ratio;
        report.matcher_contradiction_count = core.matcher_contradiction_count;
        report.matcher_best_yaw_rad = core.matcher_best_yaw_rad;
        report.matcher_best_score = core.matcher_best_score;
        report.matcher_runner_up_available = core.matcher_runner_up_available;
        report.matcher_runner_up_yaw_rad = core.matcher_runner_up_yaw_rad;
        report.matcher_runner_up_score = core.matcher_runner_up_score;
        report.matcher_score_margin = core.matcher_score_margin;
        report.matcher_score_min = core.matcher_score_min;
        report.matcher_score_max = core.matcher_score_max;
        report.matcher_score_mean = core.matcher_score_mean;
        report.matcher_score_range = core.matcher_score_range;
        report.matcher_best_at_search_edge = core.matcher_best_at_search_edge;
        report.matcher_flat_curve = core.matcher_flat_curve;
        report.matcher_multimodal = core.matcher_multimodal;
        report.matcher_accepted = core.matcher_accepted;
        report.matcher_status = core.matcher_status;
        report.matcher_rejection_reason = core.matcher_rejection_reason;
        report.matcher_delta_yaw_rad = core.matcher_delta_yaw_rad;
        report.matcher_proposal_ready = core.matcher_proposal_ready;
        report.map_from_odom_before_match = core.map_from_odom_before_match;
        report.proposed_map_from_odom = core.proposed_map_from_odom;
        report.map_from_odom_after_match = core.map_from_odom_after_match;
        report.last_matcher_input_fault = core.last_matcher_input_fault;
        report.native_multi_tof_consumed = core.native_multi_tof_consumed;
        report.measurement_time_pose_consumed = core.measurement_time_pose_consumed;
        report.single_pose_fallback_consumed = core.single_pose_fallback_consumed;
        report.legacy_projection_consumed = core.legacy_projection_consumed;
        report.real_hardware_accessed = core.real_hardware_accessed;
        report.real_motion_attempted = core.real_motion_attempted;
        report.real_map_write_attempted = core.real_map_write_attempted;
        report.real_pose_writeback_attempted = core.real_pose_writeback_attempted;
        report.last_odom_pose = core.last_odom_pose;
        report.last_predicted_map_pose = core.last_predicted_map_pose;
        report.sparse_map_cell_count = core.sparse_map_cell_count;
        report.last_fault = core.last_fault;
        report.last_message = core.last_message;
    }

    static void write_report_file(const std::string &run_dir,
                                  const SparseShadowRuntimeReport &report) {
        if (run_dir.empty()) {
            return;
        }
        std::ofstream out(run_dir + "/sparse_shadow_runtime_report.txt");
        out << write_sparse_shadow_runtime_report(report);
    }

    Config config_;
};

inline SparseShadowRuntimeReport run_sparse_shadow_runtime(
    const Config &config,
    double duration_s,
    const std::string &run_dir) {
    SparseShadowRuntime runtime(config);
    return runtime.run(duration_s, run_dir);
}

} // namespace robot_slamd
