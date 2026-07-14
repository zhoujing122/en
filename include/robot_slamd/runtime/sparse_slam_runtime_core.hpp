#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_contract_checker.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/localization/sparse_tof/sparse_tof_local_matcher.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_keyframe.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_map_artifact.hpp"
#include "robot_slamd/runtime/multi_tof_measurement_pose_resolver.hpp"
#include "robot_slamd/runtime/phase_aware_observation_controller.hpp"
#include "robot_slamd/runtime/sparse_slam_initialization.hpp"
#include "robot_slamd/runtime/timed_odom_pose_buffer.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace robot_slamd {

enum class SparseSlamRuntimeCorePhase {
    Uninitialized,
    CollectingStationarySamples,
    Ready,
    Fault
};

inline std::string to_string(SparseSlamRuntimeCorePhase phase) {
    switch (phase) {
    case SparseSlamRuntimeCorePhase::Uninitialized:
        return "uninitialized";
    case SparseSlamRuntimeCorePhase::CollectingStationarySamples:
        return "collecting_stationary_samples";
    case SparseSlamRuntimeCorePhase::Ready:
        return "ready";
    case SparseSlamRuntimeCorePhase::Fault:
        return "fault";
    }
    return "unknown";
}

struct SparseSlamRuntimeCoreReport {
    bool runtime_core_constructed = false;
    bool initialization_attempted = false;
    bool initialization_complete = false;
    std::string initialization_status = "uninitialized";
    std::string map_startup_mode = "create_new";
    std::string initial_pose_mode = "startup_zero";
    bool startup_zero_used = false;
    bool configured_pose_used = false;
    bool initial_yaw_measured_by_imu = false;
    bool initial_yaw_defined_by_startup_frame = false;
    double gyro_bias_rad_s = 0.0;
    std::size_t gyro_sample_count = 0;
    bool stationary_check_passed = false;
    bool wheel_baseline_established = false;
    bool map_from_odom_initialized = false;
    std::size_t sparse_map_load_attempt_count = 0;
    std::size_t sparse_map_load_success_count = 0;
    std::size_t sparse_map_load_reject_count = 0;
    std::size_t sparse_map_save_attempt_count = 0;
    std::size_t sparse_map_save_success_count = 0;
    std::size_t sparse_map_save_reject_count = 0;
    bool sparse_map_loaded = false;
    bool sparse_map_saved = false;
    std::string sparse_map_path;
    std::string last_map_lifecycle_fault = "none";
    double map_from_odom_x_m = 0.0;
    double map_from_odom_y_m = 0.0;
    double map_from_odom_yaw_rad = 0.0;
    bool wheel_imu_estimator_constructed = false;
    bool sparse_backend_constructed = false;
    std::size_t sensor_snapshot_count = 0;
    std::size_t odom_update_attempt_count = 0;
    std::size_t odom_update_success_count = 0;
    std::size_t odom_update_reject_count = 0;
    std::size_t pose_buffer_append_count = 0;
    std::size_t pose_buffer_reject_count = 0;
    std::size_t pose_buffer_size = 0;
    std::size_t front_pose_lookup_success_count = 0;
    std::size_t left_pose_lookup_success_count = 0;
    std::size_t right_pose_lookup_success_count = 0;
    std::size_t measurement_pose_set_success_count = 0;
    std::size_t measurement_pose_set_reject_count = 0;
    bool measurement_time_pose_consumed = false;
    bool single_pose_fallback_consumed = false;
    std::size_t backend_update_attempt_count = 0;
    std::size_t backend_accept_count = 0;
    std::size_t backend_reject_count = 0;
    std::size_t backend_fault_count = 0;
    std::size_t predicted_pose_handoff_count = 0;
    std::size_t missing_predicted_pose_count = 0;
    std::size_t native_multi_tof_snapshot_count = 0;
    bool native_multi_tof_consumed = false;
    bool legacy_projection_consumed = false;
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
    std::size_t atomic_commit_attempt_count = 0;
    std::size_t atomic_commit_success_count = 0;
    std::size_t atomic_commit_reject_count = 0;
    std::size_t correction_apply_success_count = 0;
    std::size_t keyframe_commit_count = 0;
    std::size_t keyframe_count = 0;
    std::size_t keyframe_total_ray_count = 0;
    std::size_t keyframe_transaction_changed_cell_count = 0;
    std::uint64_t committed_bundle_id = 0;
    std::uint64_t committed_keyframe_id = 0;
    std::uint64_t committed_revision_before = 0;
    std::uint64_t committed_revision_after = 0;
    std::string last_atomic_commit_fault = "none";
    std::string last_matcher_input_fault = "none";
    bool real_hardware_accessed = false;
    bool real_motion_attempted = false;
    bool real_map_write_attempted = false;
    bool real_pose_writeback_attempted = false;
    RobotPose2D last_odom_pose;
    RobotPose2D last_predicted_map_pose;
    std::size_t sparse_map_cell_count = 0;
    std::string last_fault = "none";
    std::string last_message;
};

struct SparseSlamRuntimeCoreStepResult {
    bool ok = false;
    bool initialized = false;
    bool odom_updated = false;
    bool map_updated = false;
    bool backend_called = false;
    std::string status;
    std::string message;
};

struct SparseSlamStepRequest {
    RobotSlamSensorSnapshot snapshot;
    double now_s = 0.0;
    ActiveObservationControl observation_control = ActiveObservationControl::None;
    std::uint64_t bundle_id = 0;
};

struct SparseMapLifecycleResult {
    bool ok = false;
    std::string reason;
};

inline SlamBackendContractChecker sparse_slam_runtime_core_backend_checker() {
    SlamBackendContractOptions options;
    options.require_tof = false;
    options.require_imu_or_wheel = true;
    options.allow_predicted_pose_missing = true;
    return SlamBackendContractChecker(options);
}

inline SparseSlamInitializationRequest
sparse_slam_initialization_request_from_config(const Config &config) {
    SparseSlamInitializationRequest request;
    MapStartupMode startup_mode = MapStartupMode::CreateNewMap;
    InitialPoseMode pose_mode = InitialPoseMode::StartupZero;
    (void)parse_map_startup_mode(config.sparse_slam_map_startup_mode,
                                 startup_mode);
    (void)parse_initial_pose_mode(config.sparse_slam_initial_pose_mode,
                                  pose_mode);
    request.map_startup_mode = startup_mode;
    request.initial_pose_mode = pose_mode;
    request.has_configured_map_pose = config.sparse_slam_has_configured_pose;
    request.configured_map_pose = make_map_pose({
        config.sparse_slam_configured_pose_x_m,
        config.sparse_slam_configured_pose_y_m,
        config.sparse_slam_configured_pose_yaw_rad});
    request.map_path = config.sparse_slam_map_path;
    return request;
}

class SparseSlamRuntimeCore final {
public:
    explicit SparseSlamRuntimeCore(Config config)
        : config_(std::move(config)),
          estimator_(make_odom_config(config_)),
          pose_buffer_(make_pose_buffer_config(config_)),
          resolver_(make_resolver_config(config_)),
          observation_controller_(make_observation_controller_config(config_)),
          keyframe_manager_(make_keyframe_config(config_)),
          sparse_backend_(std::make_shared<SparseMultiTofOccupancyBackendBinding>(
              make_backend_options(config_))),
          map_port_(sparse_backend_,
                    sparse_slam_runtime_core_backend_checker()) {
        report_.runtime_core_constructed = true;
        report_.wheel_imu_estimator_constructed = true;
        report_.sparse_backend_constructed = true;
    }

    SparseSlamInitializationResult initialize(
        const SparseSlamInitializationRequest &request) {
        report_.initialization_attempted = true;
        report_.map_startup_mode = to_string(request.map_startup_mode);
        report_.initial_pose_mode = to_string(request.initial_pose_mode);
        SparseSlamInitializationRequest staged_request = request;
        SparseMapArtifact loaded_artifact;
        bool install_loaded_map = false;
        if (request.map_startup_mode == MapStartupMode::LoadExistingMap &&
            request.initial_pose_mode == InitialPoseMode::ConfiguredPose) {
            report_.sparse_map_load_attempt_count++;
            report_.sparse_map_path = request.map_path;
            if (!request.has_configured_map_pose || request.map_path.empty() ||
                !config_.sparse_slam_planar_tof_extrinsics_configured) {
                return initialization_failure(
                    SparseSlamInitializationStatus::InitialPoseMissing,
                    "existing_map_requires_configured_pose_path_and_extrinsics");
            }
            const auto loaded = load_sparse_map_artifact(
                request.map_path, map_artifact_limits());
            std::string compatibility_reason;
            if (!loaded.ok ||
                !sparse_map_artifact_compatible(
                    loaded.artifact, sparse_backend_->options().grid,
                    make_planar_tof_extrinsics(config_), config_.wheel_base_m,
                    config_.wheel_radius_left_m,
                    config_.wheel_radius_right_m,
                    config_.encoder_ticks_per_rev, compatibility_reason)) {
                return initialization_failure(
                    SparseSlamInitializationStatus::Fault,
                    loaded.ok ? compatibility_reason : loaded.reason);
            }
            loaded_artifact = loaded.artifact;
            staged_request.existing_map_loaded = true;
            install_loaded_map = true;
        }
        const auto preflight = frame_state_.initialize(staged_request);
        if (!preflight.ok) {
            phase_ = SparseSlamRuntimeCorePhase::Fault;
            report_.initialization_status = to_string(preflight.status);
            report_.last_fault = report_.initialization_status;
            report_.last_message = preflight.message;
            return preflight;
        }
        if (install_loaded_map) {
            if (!sparse_backend_->install_map_cells_transactional(
                    loaded_artifact.cells)) {
                return initialization_failure(
                    SparseSlamInitializationStatus::Fault,
                    "existing_sparse_map_install_failed");
            }
            current_map_revision_ = loaded_artifact.map_revision;
            report_.current_map_revision = current_map_revision_;
            report_.sparse_map_cell_count = loaded_artifact.cells.size();
            report_.sparse_map_loaded = true;
            report_.sparse_map_load_success_count++;
            report_.last_map_lifecycle_fault = "none";
        }
        init_request_ = staged_request;
        frame_state_ = MapOdomFrameState{};
        gyro_samples_.clear();
        gyro_bias_rad_s_ = 0.0;
        phase_ = SparseSlamRuntimeCorePhase::CollectingStationarySamples;
        report_.initialization_status = to_string(
            SparseSlamInitializationStatus::WaitingForStationarySamples);
        return initialization_result(false,
            SparseSlamInitializationStatus::WaitingForStationarySamples,
            "sparse_slam_waiting_for_stationary_samples");
    }

    SparseSlamRuntimeCoreStepResult step(
        const RobotSlamSensorSnapshot &snapshot,
        double now_s) {
        SparseSlamStepRequest request;
        request.snapshot = snapshot;
        request.now_s = now_s;
        request.observation_control = ActiveObservationControl::None;
        return step(request);
    }

    SparseSlamRuntimeCoreStepResult step(const SparseSlamStepRequest &request) {
        (void)request.now_s;
        const auto control_result = apply_observation_control(request);
        if (!control_result.ok) {
            sync_observation_report();
            return control_result;
        }

        const RobotSlamSensorSnapshot &snapshot = request.snapshot;
        report_.sensor_snapshot_count++;
        if (snapshot.has_multi_tof) {
            report_.native_multi_tof_snapshot_count++;
        }
        if (phase_ == SparseSlamRuntimeCorePhase::Uninitialized) {
            return {false, false, false, false, false,
                    "not_initialized", "sparse_slam_core_not_initialized"};
        }
        if (phase_ == SparseSlamRuntimeCorePhase::Fault) {
            return {false, false, false, false, false,
                    report_.last_fault, report_.last_message};
        }
        if (phase_ == SparseSlamRuntimeCorePhase::CollectingStationarySamples) {
            const auto initialized = consume_startup_sample(snapshot);
            sync_observation_report();
            return {initialized.ok, report_.initialization_complete, false,
                    false, false, report_.initialization_status,
                    initialized.message};
        }

        if (!snapshot.has_wheel || !snapshot.has_imu) {
            report_.odom_update_reject_count++;
            report_.missing_predicted_pose_count++;
            report_.last_fault = "missing_wheel_or_imu";
            report_.last_message = "sparse_slam_core_missing_wheel_or_imu";
            return {false, true, false, false, false,
                    report_.last_fault, report_.last_message};
        }

        ImuFrame imu = snapshot.imu;
        imu.yaw_rate_rad_s -= gyro_bias_rad_s_;
        const auto active_state_before_odom = observation_controller_.state();
        report_.odom_update_attempt_count++;
        const auto odom_result = estimator_.update(snapshot.wheel, imu);
        if (!odom_result.ok) {
            report_.odom_update_reject_count++;
            report_.last_fault = to_string(odom_result.fault);
            report_.last_message = odom_result.message;
            return {false, true, false, false, false,
                    report_.last_fault, report_.last_message};
        }
        report_.odom_update_success_count++;
        if (active_state_before_odom == ActiveObservationBundleState::CollectingDuringMotion) {
            report_.odom_update_during_motion_count++;
        } else if (active_state_before_odom == ActiveObservationBundleState::WaitingForMotionSettle) {
            report_.odom_update_during_settle_count++;
        } else if (active_state_before_odom == ActiveObservationBundleState::FrozenReady) {
            report_.odom_update_after_freeze_count++;
        }

        const OdomPose2D odom_pose = make_odom_pose(estimator_.estimated_pose());
        report_.last_odom_pose = odom_pose.odom_T_base;
        const auto append = pose_buffer_.append(
            snapshot.wheel.timestamp_s, odom_pose);
        report_.pose_buffer_size = pose_buffer_.size();
        if (!append.ok) {
            report_.pose_buffer_reject_count++;
            report_.last_fault = "pose_buffer_append_rejected";
            report_.last_message = append.reason;
            return {false, true, true, false, false,
                    report_.last_fault, report_.last_message};
        }
        report_.pose_buffer_append_count++;
        report_.pose_buffer_size = pose_buffer_.size();

        const MapPose2D latest_map_pose =
            frame_state_.map_pose_from_odom(odom_pose);
        report_.last_predicted_map_pose = latest_map_pose.map_T_base;

        if (!snapshot.has_multi_tof) {
            report_.last_fault = "none";
            report_.last_message = "sparse_slam_core_odom_only_step";
            sync_observation_report();
            return {true, true, true, false, false,
                    "odom_only", report_.last_message};
        }

        const auto resolved = resolver_.resolve(
            snapshot.multi_tof, pose_buffer_, frame_state_);
        if (resolved.poses.front.valid) {
            report_.front_pose_lookup_success_count++;
        }
        if (resolved.poses.left.valid) {
            report_.left_pose_lookup_success_count++;
        }
        if (resolved.poses.right.valid) {
            report_.right_pose_lookup_success_count++;
        }
        if (!resolved.ok) {
            report_.measurement_pose_set_reject_count++;
            report_.last_fault = "measurement_pose_lookup_failed";
            report_.last_message = resolved.reason;
            sync_observation_report();
            return {false, true, true, false, false,
                    report_.last_fault, report_.last_message};
        }
        report_.measurement_pose_set_success_count++;

        auto active_state = observation_controller_.state();
        if (active_state == ActiveObservationBundleState::CollectingDuringMotion ||
            active_state == ActiveObservationBundleState::WaitingForMotionSettle ||
            active_state == ActiveObservationBundleState::FrozenReady ||
            active_state == ActiveObservationBundleState::Aborted) {
            report_.map_commit_blocked_count++;
            const auto guard = observation_controller_.verify_reference_map(
                current_map_revision_, report_.sparse_map_cell_count);
            if (!guard.ok) {
                sync_observation_report();
                report_.last_fault = "reference_map_changed_during_active_bundle";
                report_.last_message = guard.message;
                return {false, true, true, false, false,
                        report_.last_fault, report_.last_message};
            }
            if (active_state == ActiveObservationBundleState::CollectingDuringMotion ||
                active_state == ActiveObservationBundleState::WaitingForMotionSettle) {
                if (active_state == ActiveObservationBundleState::CollectingDuringMotion) {
                    report_.tof_snapshot_during_motion_count++;
                    report_.measurement_pose_set_during_motion_count++;
                } else {
                    report_.tof_snapshot_during_settle_count++;
                    report_.measurement_pose_set_during_settle_count++;
                }
                const auto append_input = make_observation_append_input(
                    snapshot, resolved.poses);
                if (!append_input.ok) {
                    report_.measurement_pose_set_reject_count++;
                    sync_observation_report();
                    report_.last_fault = append_input.status;
                    report_.last_message = append_input.message;
                    return {false, true, true, false, false,
                            append_input.status, append_input.message};
                }
                const auto bundle_append = observation_controller_.append_observation(
                    append_input.input);
                if (active_state == ActiveObservationBundleState::WaitingForMotionSettle) {
                    const auto settle = observation_controller_.update_settle(
                        make_motion_settle_sample(snapshot, imu),
                        current_map_revision_);
                    (void)settle;
                    if (observation_controller_.state() ==
                        ActiveObservationBundleState::FrozenReady) {
                        const auto prepared = prepare_local_match_input(
                            snapshot.multi_tof.synchronized_time_s);
                        if (!prepared.ready) {
                            sync_observation_report();
                            report_.last_fault = "matcher_input_rejected";
                            report_.last_message = prepared.reason;
                            return {false, true, true, false, false,
                                    report_.last_fault, report_.last_message};
                        }
                        execute_prepared_local_match();
                        if (observation_controller_.state() ==
                            ActiveObservationBundleState::Idle) {
                            sync_observation_report();
                            return {true, true, true, true, true,
                                    "atomic_local_slam_committed",
                                    report_.last_message};
                        }
                    }
                }
                sync_observation_report();
                report_.last_fault = bundle_append.ok ? "none" : to_string(bundle_append.fault);
                report_.last_message = bundle_append.message;
                return {bundle_append.ok, true, true, false, false,
                        bundle_append.ok ? "map_commit_blocked" : report_.last_fault,
                        report_.last_message};
            }
            sync_observation_report();
            report_.last_fault = "none";
            report_.last_message = "sparse_slam_core_map_commit_blocked";
            return {true, true, true, false, false,
                    "map_commit_blocked", report_.last_message};
        }

        RobotSlamMapUpdateInput update;
        update.timestamp_s = snapshot.multi_tof.synchronized_time_s;
        update.snapshot = snapshot;
        update.predicted_map_pose = latest_map_pose.map_T_base;
        update.has_predicted_map_pose = true;
        update.multi_tof_measurement_poses = resolved.poses;
        update.has_multi_tof_measurement_poses = true;
        update.mapping_commit_allowed = observation_controller_.map_commit_allowed();
        update.source = "sparse_slam_runtime_core";
        if (!update.mapping_commit_allowed) {
            report_.map_commit_blocked_count++;
            sync_observation_report();
            return {true, true, true, false, false,
                    "map_commit_blocked", "sparse_slam_core_map_commit_blocked"};
        }
        report_.predicted_pose_handoff_count++;
        report_.backend_update_attempt_count++;
        const bool accepted = map_port_.integrate_map_update(update);
        const auto backend_report = sparse_backend_->report();
        report_.measurement_time_pose_consumed =
            backend_report.measurement_time_pose_consumed;
        report_.single_pose_fallback_consumed =
            backend_report.single_pose_fallback_consumed;
        report_.native_multi_tof_consumed =
            backend_report.native_multi_tof_backend_consumption;
        report_.legacy_projection_consumed =
            backend_report.legacy_projection_consumed;
        report_.sparse_map_cell_count = backend_report.active_cell_count;
        if (accepted) {
            current_map_revision_++;
            report_.current_map_revision = current_map_revision_;
            report_.backend_accept_count++;
            report_.last_fault = "none";
            report_.last_message = "sparse_slam_core_backend_accepted";
        } else {
            report_.backend_reject_count++;
            report_.backend_fault_count++;
            report_.last_fault = "backend_rejected";
            report_.last_message = "sparse_slam_core_backend_rejected";
        }
        sync_observation_report();
        return {accepted, true, true, accepted, true,
                accepted ? "map_updated" : "backend_rejected",
                report_.last_message};
    }

    const SparseSlamRuntimeCoreReport &report() const { return report_; }
    const MapOdomFrameState &frame_state() const { return frame_state_; }
    const TimedOdomPoseBuffer &pose_buffer() const { return pose_buffer_; }
    const PreparedSparseTofLocalMatchInput &prepared_local_match_input() const {
        return prepared_local_match_input_;
    }
    const SparseOccupancyGridSnapshot *reference_map_snapshot() const {
        return reference_map_snapshot_.get();
    }
    const SparseTofLocalMatchResult *local_match_result() const {
        return local_match_result_ ? local_match_result_.get() : nullptr;
    }
    const SparseTofKeyframe *latest_keyframe() const {
        return keyframe_manager_.latest();
    }
    std::size_t keyframe_count() const { return keyframe_manager_.size(); }

    SparseOccupancyGridSnapshot sparse_map_snapshot() const {
        return sparse_backend_->capture_grid_snapshot(
            current_map_revision_, map_artifact_limits().maximum_cells).snapshot;
    }

    SparseMapLifecycleResult save_sparse_map(
        const std::string &path,
        SparseMapArtifactSaveOptions options = {}) {
        report_.sparse_map_save_attempt_count++;
        report_.sparse_map_path = path;
        if (!config_.sparse_slam_planar_tof_extrinsics_configured) {
            report_.sparse_map_save_reject_count++;
            report_.last_map_lifecycle_fault =
                "sparse_map_save_requires_configured_extrinsics";
            return {false, report_.last_map_lifecycle_fault};
        }
        if (phase_ != SparseSlamRuntimeCorePhase::Ready ||
            observation_controller_.state() !=
                ActiveObservationBundleState::Idle ||
            reference_map_snapshot_ || prepared_local_match_input_.is_ready() ||
            local_match_result_) {
            report_.sparse_map_save_reject_count++;
            report_.last_map_lifecycle_fault =
                "sparse_map_save_requires_stable_idle";
            return {false, report_.last_map_lifecycle_fault};
        }
        const auto captured = sparse_backend_->capture_grid_snapshot(
            current_map_revision_, map_artifact_limits().maximum_cells);
        if (!captured.ok) {
            report_.sparse_map_save_reject_count++;
            report_.last_map_lifecycle_fault = captured.message;
            return {false, captured.message};
        }
        SparseMapArtifact artifact;
        artifact.map_id = config_.sparse_slam_map_id;
        artifact.run_id = config_.sparse_slam_map_run_id;
        artifact.map_revision = current_map_revision_;
        artifact.grid = sparse_backend_->options().grid;
        artifact.tof_extrinsics = make_planar_tof_extrinsics(config_);
        artifact.wheel_base_m = config_.wheel_base_m;
        artifact.wheel_radius_left_m = config_.wheel_radius_left_m;
        artifact.wheel_radius_right_m = config_.wheel_radius_right_m;
        artifact.encoder_ticks_per_rev = config_.encoder_ticks_per_rev;
        artifact.cells = captured.snapshot.cells();
        std::string reason;
        if (!save_sparse_map_artifact_atomic(
                path, artifact, map_artifact_limits(), reason, options)) {
            report_.sparse_map_save_reject_count++;
            report_.last_map_lifecycle_fault = reason;
            return {false, reason};
        }
        report_.sparse_map_save_success_count++;
        report_.sparse_map_saved = true;
        report_.last_map_lifecycle_fault = "none";
        return {true, reason};
    }

private:
    SparseMapArtifactLimits map_artifact_limits() const {
        return {
            static_cast<std::size_t>(
                std::max(1, config_.sparse_slam_map_artifact_max_cells)),
            static_cast<std::size_t>(
                std::max(1024, config_.sparse_slam_map_artifact_max_file_bytes))};
    }

    SparseSlamInitializationResult initialization_failure(
        SparseSlamInitializationStatus status,
        const std::string &message) {
        phase_ = SparseSlamRuntimeCorePhase::Fault;
        report_.sparse_map_load_reject_count++;
        report_.initialization_status = to_string(status);
        report_.last_map_lifecycle_fault = message;
        report_.last_fault = report_.initialization_status;
        report_.last_message = message;
        SparseSlamInitializationResult result;
        result.status = status;
        result.message = message;
        return result;
    }

    static WheelImuDeadReckoningConfig make_odom_config(
        const Config &config) {
        WheelImuDeadReckoningConfig out;
        out.wheel_radius_m = config.wheel_radius_left_m;
        out.wheel_base_m = config.wheel_base_m;
        out.allow_imu_missing_wheel_yaw_fallback = false;
        return out;
    }

    static TimedOdomPoseBufferConfig make_pose_buffer_config(
        const Config &config) {
        TimedOdomPoseBufferConfig out;
        out.capacity = static_cast<std::size_t>(
            std::max(2, config.sparse_slam_pose_buffer_capacity));
        out.max_age_s = config.sparse_slam_pose_buffer_max_age_s;
        out.max_interpolation_gap_s =
            config.sparse_slam_pose_interpolation_max_gap_s;
        return out;
    }

    static MultiTofMeasurementPoseResolverConfig make_resolver_config(
        const Config &config) {
        MultiTofMeasurementPoseResolverConfig out;
        out.require_all_three_measurement_poses =
            config.sparse_slam_require_all_measurement_poses;
        return out;
    }

    static PhaseAwareObservationControllerConfig make_observation_controller_config(
        const Config &config) {
        PhaseAwareObservationControllerConfig out;
        out.bundle.max_snapshots = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_active_bundle_max_snapshots));
        out.bundle.max_rays = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_active_bundle_max_rays));
        out.bundle.max_duration_s = config.sparse_slam_active_bundle_max_duration_s;
        out.bundle.max_snapshot_gap_s = config.sparse_slam_active_bundle_max_snapshot_gap_s;
        out.bundle.min_snapshots = static_cast<std::size_t>(
            std::max(0, config.sparse_slam_active_bundle_min_snapshots));
        out.bundle.min_matchable_rays = static_cast<std::size_t>(
            std::max(0, config.sparse_slam_active_bundle_min_matchable_rays));
        out.bundle.min_yaw_span_rad = config.sparse_slam_active_bundle_min_yaw_span_rad;
        out.bundle.require_all_measurement_poses =
            config.sparse_slam_active_bundle_require_all_measurement_poses;
        out.settle.max_abs_linear_speed_mps =
            config.sparse_slam_settle_max_abs_linear_speed_mps;
        out.settle.max_abs_wheel_yaw_rate_rad_s =
            config.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s;
        out.settle.max_abs_imu_yaw_rate_rad_s =
            config.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s;
        out.settle.min_consecutive_samples = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_settle_min_consecutive_samples));
        out.settle.min_stable_duration_s =
            config.sparse_slam_settle_min_stable_duration_s;
        out.settle.max_sample_gap_s =
            config.sparse_slam_settle_max_sample_gap_s;
        return out;
    }

    static SparseTofLocalMatchConfig make_local_match_config(
        const Config &config) {
        SparseTofLocalMatchConfig out;
        out.use_planar_tof_extrinsics =
            config.sparse_slam_planar_tof_extrinsics_configured;
        out.planar_tof_extrinsics = make_planar_tof_extrinsics(config);
        (void)parse_sparse_tof_local_match_mode(
            config.sparse_slam_local_match_mode, out.mode);
        out.max_abs_yaw_rad = config.sparse_slam_local_match_max_abs_yaw_rad;
        out.coarse_yaw_step_rad = config.sparse_slam_local_match_coarse_yaw_step_rad;
        out.fine_yaw_step_rad = config.sparse_slam_local_match_fine_yaw_step_rad;
        out.max_abs_translation_x_m = config.sparse_slam_local_match_max_abs_translation_x_m;
        out.max_abs_translation_y_m = config.sparse_slam_local_match_max_abs_translation_y_m;
        out.max_candidate_count = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_max_candidate_count));
        out.max_coarse_candidates = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_max_coarse_candidates));
        out.max_fine_candidates = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_max_fine_candidates));
        out.max_total_candidates = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_max_total_candidates));
        out.max_scored_rays = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_max_scored_rays));
        out.max_cells_per_ray = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_max_cells_per_ray));
        out.min_bundle_frames = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_min_bundle_frames));
        out.min_matchable_rays = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_min_matchable_rays));
        out.min_reference_cells = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_min_reference_cells));
        out.min_reference_occupied_cells = static_cast<std::size_t>(
            std::max(0, config.sparse_slam_local_match_min_reference_occupied_cells));
        out.min_reference_free_cells = static_cast<std::size_t>(
            std::max(0, config.sparse_slam_local_match_min_reference_free_cells));
        out.max_bundle_duration_s = config.sparse_slam_active_bundle_max_duration_s;
        out.no_return_free_space_range_m =
            config.sparse_slam_local_match_no_return_free_space_range_m;
        out.min_used_rays = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_min_used_rays));
        out.min_known_cells = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_local_match_min_known_cells));
        out.max_unknown_ratio = config.sparse_slam_local_match_max_unknown_ratio;
        out.max_contradiction_ratio =
            config.sparse_slam_local_match_max_contradiction_ratio;
        out.minimum_normalized_score =
            config.sparse_slam_local_match_min_normalized_score;
        out.minimum_score_margin = config.sparse_slam_local_match_min_score_margin;
        out.minimum_score_range = config.sparse_slam_local_match_min_score_range;
        out.runner_up_exclusion_yaw_rad =
            config.sparse_slam_local_match_runner_up_exclusion_yaw_rad;
        out.multimodal_max_score_drop =
            config.sparse_slam_local_match_multimodal_max_score_drop;
        out.free_agreement_weight =
            config.sparse_slam_local_match_free_agreement_weight;
        out.free_contradiction_weight =
            config.sparse_slam_local_match_free_contradiction_weight;
        out.occupied_endpoint_agreement_weight =
            config.sparse_slam_local_match_occupied_endpoint_agreement_weight;
        out.occupied_endpoint_contradiction_weight =
            config.sparse_slam_local_match_occupied_endpoint_contradiction_weight;
        out.require_revision_match = config.sparse_slam_local_match_require_revision_match;
        out.require_immutable_snapshot = config.sparse_slam_local_match_require_immutable_snapshot;
        return out;
    }

    static PlanarThreeTofExtrinsics make_planar_tof_extrinsics(
        const Config &config) {
        return {
            {config.sparse_slam_front_tof_x_m,
             config.sparse_slam_front_tof_y_m,
             config.sparse_slam_front_tof_yaw_rad},
            {config.sparse_slam_left_tof_x_m,
             config.sparse_slam_left_tof_y_m,
             config.sparse_slam_left_tof_yaw_rad},
            {config.sparse_slam_right_tof_x_m,
             config.sparse_slam_right_tof_y_m,
             config.sparse_slam_right_tof_yaw_rad}};
    }

    static SparseMultiTofOccupancyBackendOptions make_backend_options(
        const Config &config) {
        SparseMultiTofOccupancyBackendOptions out;
        out.minimum_accepted_snapshots_for_good_quality = 1;
        out.minimum_valid_rays_for_good_quality = 1;
        out.minimum_observed_cells_for_good_quality = 1;
        out.minimum_angular_bins_for_good_quality = 1;
        out.require_multi_tof_measurement_poses = true;
        out.allow_single_pose_fallback = false;
        out.use_planar_tof_extrinsics =
            config.sparse_slam_planar_tof_extrinsics_configured;
        out.planar_tof_extrinsics = make_planar_tof_extrinsics(config);
        return out;
    }

    static SparseTofKeyframeConfig make_keyframe_config(
        const Config &config) {
        SparseTofKeyframeConfig out;
        out.max_keyframes =
            static_cast<std::size_t>(std::max(1, config.sparse_slam_max_keyframes));
        out.max_total_rays = static_cast<std::size_t>(
            std::max(1, config.sparse_slam_max_total_keyframe_rays));
        return out;
    }

    SparseSlamRuntimeCoreStepResult apply_observation_control(
        const SparseSlamStepRequest &request) {
        if (request.observation_control == ActiveObservationControl::None) {
            return {true, phase_ == SparseSlamRuntimeCorePhase::Ready,
                    false, false, false, "control_none", "control_none"};
        }
        if (phase_ != SparseSlamRuntimeCorePhase::Ready) {
            report_.bundle_invalid_transition_count++;
            report_.last_bundle_fault = "invalid_state";
            report_.last_fault = "active_bundle_control_before_ready";
            report_.last_message = "active_bundle_control_requires_ready_core";
            return {false, report_.initialization_complete, false, false,
                    false, report_.last_fault, report_.last_message};
        }
        ActiveMultiTofObservationResult result;
        if (request.observation_control == ActiveObservationControl::BeginCollection) {
            result = observation_controller_.begin_collection(
                current_map_revision_, report_.sparse_map_cell_count,
                frame_state_.current_map_from_odom());
        } else if (request.observation_control == ActiveObservationControl::EndMotionAndWaitForSettle) {
            result = observation_controller_.end_motion_and_wait_for_settle();
        } else if (request.observation_control == ActiveObservationControl::DiscardFrozenBundle) {
            const std::uint64_t id = request.bundle_id != 0
                ? request.bundle_id
                : observation_controller_.report().bundle_id;
            result = observation_controller_.discard(id);
            if (result.ok) {
                clear_prepared_local_match_input();
            }
        }
        sync_observation_report();
        if (!result.ok) {
            report_.last_fault = to_string(result.fault);
            report_.last_message = result.message;
            return {false, true, false, false, false,
                    report_.last_fault, report_.last_message};
        }
        report_.last_fault = "none";
        report_.last_message = result.message;
        return {true, true, false, false, false,
                to_string(request.observation_control), result.message};
    }


    struct PrepareLocalMatchInputResult {
        bool ready = false;
        std::string reason;
    };

    PrepareLocalMatchInputResult prepare_local_match_input(
        double match_request_timestamp_s) {
        report_.matcher_input_prepare_attempt_count++;
        report_.reference_snapshot_capture_attempt_count++;
        const auto &frozen = observation_controller_.frozen_bundle();
        const auto &summary = frozen.summary();
        if (!frozen.available() ||
            current_map_revision_ != summary.reference_map_revision ||
            report_.sparse_map_cell_count != summary.reference_map_cell_count) {
            return reject_local_match_input(
                SparseTofLocalMatchInputStatus::ReferenceRevisionMismatch,
                "matcher_input_reference_metadata_mismatch", true);
        }

        const auto capture = sparse_backend_->capture_grid_snapshot(
            current_map_revision_, static_cast<std::size_t>(
                std::max(1, config_.sparse_slam_reference_snapshot_max_cells)));
        if (!capture.ok || capture.snapshot.revision() != current_map_revision_ ||
            capture.snapshot.cell_count() != summary.reference_map_cell_count) {
            return reject_local_match_input(
                SparseTofLocalMatchInputStatus::ReferenceMapInvalid,
                capture.ok ? "matcher_input_snapshot_metadata_mismatch"
                           : capture.message,
                true);
        }
        report_.reference_snapshot_capture_success_count++;

        SparseTofLocalMatchInput input;
        input.bundle_id = summary.bundle_id;
        input.frozen_bundle =
            std::make_shared<const FrozenMultiTofObservationBundle>(frozen);
        input.reference_map =
            std::make_shared<const SparseOccupancyGridSnapshot>(capture.snapshot);
        input.expected_reference_map_revision = summary.reference_map_revision;
        input.current_runtime_map_revision = current_map_revision_;
        input.predicted_map_from_odom = frame_state_.current_map_from_odom();
        input.map_from_odom_at_collection_start = summary.map_from_odom_at_start;
        input.match_request_timestamp_s = match_request_timestamp_s;
        input.source = "sparse_slam_runtime_core";
        input.config = make_local_match_config(config_);

        const auto validation = local_match_input_validator_.validate(input);
        if (!validation.ready) {
            return reject_local_match_input(
                validation.status, validation.reason, false);
        }

        reference_map_snapshot_ = input.reference_map;
        prepared_local_match_input_ =
            PreparedSparseTofLocalMatchInput::ready(std::move(input));
        const auto *prepared = prepared_local_match_input_.input();
        report_.matcher_input_prepare_success_count++;
        report_.reference_snapshot_ready = true;
        report_.reference_snapshot_revision = reference_map_snapshot_->revision();
        report_.reference_snapshot_cell_count = reference_map_snapshot_->cell_count();
        report_.reference_snapshot_occupied_cell_count =
            reference_map_snapshot_->occupied_cell_count();
        report_.reference_snapshot_free_cell_count =
            reference_map_snapshot_->free_cell_count();
        report_.reference_snapshot_uncertain_cell_count =
            reference_map_snapshot_->uncertain_cell_count();
        report_.reference_snapshot_resolution_m =
            reference_map_snapshot_->resolution_m();
        report_.matcher_input_ready = true;
        report_.matcher_input_bundle_id = prepared->bundle_id;
        report_.matcher_input_reference_revision =
            prepared->expected_reference_map_revision;
        report_.matcher_input_bundle_frame_count =
            prepared->frozen_bundle->frames().size();
        report_.matcher_input_matchable_ray_count =
            prepared->frozen_bundle->summary().matchable_ray_count;
        report_.matcher_input_requested_mode = to_string(prepared->config.mode);
        report_.matcher_input_status = "ready";
        report_.last_matcher_input_fault = "none";
        return {true, "local_match_input_ready"};
    }

    PrepareLocalMatchInputResult reject_local_match_input(
        SparseTofLocalMatchInputStatus status,
        const std::string &reason,
        bool snapshot_rejected) {
        if (snapshot_rejected) {
            report_.reference_snapshot_capture_reject_count++;
        }
        report_.matcher_input_prepare_reject_count++;
        reference_map_snapshot_.reset();
        prepared_local_match_input_ =
            PreparedSparseTofLocalMatchInput::rejected(status, reason);
        report_.reference_snapshot_ready = false;
        report_.matcher_input_ready = false;
        report_.matcher_input_status = to_string(status);
        report_.last_matcher_input_fault = reason;
        (void)observation_controller_.abort_matcher_input(reason);
        return {false, reason};
    }

    void execute_prepared_local_match() {
        if (local_match_result_ || !prepared_local_match_input_.is_ready() ||
            prepared_local_match_input_.input() == nullptr) {
            return;
        }
        report_.matcher_attempt_count++;
        report_.map_from_odom_before_match =
            frame_state_.current_map_from_odom().map_T_odom;
        const auto result = local_matcher_.match(
            *prepared_local_match_input_.input());
        local_match_result_ =
            std::make_shared<const SparseTofLocalMatchResult>(result);
        report_.matcher_executed = result.matcher_executed;
        if (result.matcher_executed) report_.matcher_execution_count++;
        report_.matcher_evaluated_candidate_count =
            result.evaluated_candidate_count;
        report_.matcher_coarse_candidate_count = result.coarse_candidate_count;
        report_.matcher_fine_candidate_count = result.fine_candidate_count;
        report_.matcher_used_ray_count = result.best_metrics.used_ray_count;
        report_.matcher_known_cell_count = result.best_metrics.known_cell_count;
        report_.matcher_unknown_cell_count = result.best_metrics.unknown_cell_count;
        report_.matcher_unknown_ratio = result.best_metrics.unknown_ratio;
        report_.matcher_contradiction_count =
            result.best_metrics.contradiction_count;
        report_.matcher_best_yaw_rad = result.best_yaw_rad.value_or(0.0);
        report_.matcher_best_score = result.best_score.value_or(0.0);
        report_.matcher_runner_up_available = result.runner_up_available;
        report_.matcher_runner_up_yaw_rad =
            result.runner_up_yaw_rad.value_or(0.0);
        report_.matcher_runner_up_score =
            result.second_best_score.value_or(0.0);
        report_.matcher_score_margin = result.score_margin.value_or(0.0);
        report_.matcher_score_min = result.score_min;
        report_.matcher_score_max = result.score_max;
        report_.matcher_score_mean = result.score_mean;
        report_.matcher_score_range = result.score_range;
        report_.matcher_best_at_search_edge = result.best_at_search_edge;
        report_.matcher_flat_curve = result.flat_curve;
        report_.matcher_multimodal = result.multimodal;
        report_.matcher_accepted = result.accepted;
        report_.matcher_status = to_string(result.status);
        report_.matcher_rejection_reason = result.accepted ? "none" : result.reason;
        report_.matcher_delta_yaw_rad = result.best_delta_yaw_rad.value_or(0.0);
        report_.matcher_proposal_ready = result.proposal_ready;
        report_.proposed_map_from_odom = result.proposed_map_from_odom
            ? result.proposed_map_from_odom->map_T_odom
            : RobotPose2D{};
        report_.map_from_odom_after_match =
            frame_state_.current_map_from_odom().map_T_odom;
        if (result.status == SparseTofLocalMatchStatus::Fault) {
            report_.matcher_fault_count++;
            (void)observation_controller_.abort_matcher_input(result.reason);
        } else if (result.accepted) {
            report_.matcher_accept_count++;
            try_commit_accepted_local_slam(result);
        } else {
            report_.matcher_reject_count++;
        }
    }

    void reject_atomic_local_slam_commit(const std::string &reason) {
        report_.atomic_commit_reject_count++;
        report_.last_atomic_commit_fault = reason;
        report_.last_fault = "atomic_local_slam_commit_rejected";
        report_.last_message = reason;
        if (observation_controller_.state() ==
            ActiveObservationBundleState::FrozenReady) {
            (void)observation_controller_.abort_matcher_input(reason);
        }
    }

    void try_commit_accepted_local_slam(
        const SparseTofLocalMatchResult &match) {
        report_.atomic_commit_attempt_count++;
        const auto &frozen = observation_controller_.frozen_bundle();
        const auto current_transform = frame_state_.current_map_from_odom();
        if (!config_.sparse_slam_enable_atomic_local_slam_commit ||
            match.status != SparseTofLocalMatchStatus::AcceptedYawOnly ||
            !match.matcher_executed || !match.accepted ||
            !match.proposal_ready || !match.proposed_map_from_odom ||
            !match.correction_delta || !prepared_local_match_input_.is_ready() ||
            prepared_local_match_input_.input() == nullptr ||
            !frozen.available() ||
            !observation_controller_.can_commit_frozen_bundle(match.bundle_id) ||
            match.bundle_id != frozen.summary().bundle_id ||
            match.reference_map_revision != current_map_revision_ ||
            frozen.summary().reference_map_revision != current_map_revision_ ||
            !sparse_slam_frame_pose_valid(match.predicted_map_from_odom) ||
            !sparse_slam_frame_pose_valid(*match.proposed_map_from_odom) ||
            std::fabs(match.correction_delta->yaw_rad) >
                config_.sparse_slam_max_abs_yaw_correction_rad ||
            (keyframe_manager_.latest() != nullptr &&
             keyframe_manager_.latest()->bundle_id == match.bundle_id) ||
            !frame_state_.can_apply_local_slam_transform(
                *match.proposed_map_from_odom) ||
            current_transform.map_T_odom.x_m !=
                match.predicted_map_from_odom.map_T_odom.x_m ||
            current_transform.map_T_odom.y_m !=
                match.predicted_map_from_odom.map_T_odom.y_m ||
            current_transform.map_T_odom.yaw_rad !=
                match.predicted_map_from_odom.map_T_odom.yaw_rad) {
            reject_atomic_local_slam_commit(
                "atomic_local_slam_match_contract_rejected");
            return;
        }

        report_.pose_correction_attempt_count++;
        auto map_prepare = sparse_backend_->prepare_keyframe_map_transaction(
            frozen, *match.proposed_map_from_odom,
            frozen.summary().reference_map_revision, current_map_revision_,
            static_cast<std::size_t>(std::max(
                1, config_.sparse_slam_max_cells_per_keyframe_transaction)));
        if (!map_prepare.ok) {
            reject_atomic_local_slam_commit(map_prepare.reason);
            return;
        }

        report_.keyframe_attempt_count++;
        SparseTofKeyframe keyframe;
        keyframe.bundle_id = match.bundle_id;
        keyframe.reference_map_revision = current_map_revision_;
        keyframe.committed_map_revision = current_map_revision_ + 1;
        keyframe.collection_start_timestamp_s =
            frozen.summary().collection_start_timestamp_s;
        keyframe.collection_end_timestamp_s =
            frozen.summary().collection_end_timestamp_s;
        keyframe.map_from_odom_before = current_transform;
        keyframe.map_from_odom_after = *match.proposed_map_from_odom;
        keyframe.matcher_delta_yaw_rad = match.correction_delta->yaw_rad;
        keyframe.matcher_score = match.best_score.value_or(0.0);
        keyframe.matcher_margin = match.score_margin.value_or(0.0);
        keyframe.matcher_status = match.status;
        keyframe.snapshot_count = frozen.frames().size();
        keyframe.ray_count = frozen.summary().matchable_ray_count;
        keyframe.hit_ray_count = frozen.summary().hit_ray_count;
        keyframe.no_return_ray_count = frozen.summary().no_return_ray_count;
        keyframe.changed_cell_count =
            map_prepare.transaction.stats.changed_cell_count;
        keyframe.frozen_bundle =
            prepared_local_match_input_.input()->frozen_bundle;
        auto keyframe_prepare = keyframe_manager_.prepare(std::move(keyframe));
        if (!keyframe_prepare.ok) {
            reject_atomic_local_slam_commit(keyframe_prepare.reason);
            return;
        }

        const std::uint64_t revision_before = current_map_revision_;
        const auto changed_cells =
            map_prepare.transaction.stats.changed_cell_count;
        const auto bundle_id = match.bundle_id;
        const auto keyframe_id =
            keyframe_prepare.prepared.keyframe()->keyframe_id;
        if (!sparse_backend_->commit_keyframe_map_transaction(
                std::move(map_prepare.transaction))) {
            reject_atomic_local_slam_commit(
                "atomic_local_slam_map_commit_precondition_changed");
            return;
        }

        keyframe_manager_.commit(std::move(keyframe_prepare.prepared));
        frame_state_.commit_local_slam_transform(
            *match.proposed_map_from_odom);
        current_map_revision_ = revision_before + 1;
        const auto consumed =
            observation_controller_.commit_frozen_bundle(bundle_id);
        if (!consumed.ok) {
            phase_ = SparseSlamRuntimeCorePhase::Fault;
            report_.last_fault = "atomic_local_slam_finalize_fault";
            report_.last_message = consumed.message;
            return;
        }

        report_.atomic_commit_success_count++;
        report_.correction_apply_success_count++;
        report_.keyframe_commit_count++;
        report_.keyframe_count = keyframe_manager_.size();
        report_.keyframe_total_ray_count = keyframe_manager_.total_rays();
        report_.keyframe_transaction_changed_cell_count = changed_cells;
        report_.committed_bundle_id = bundle_id;
        report_.committed_keyframe_id = keyframe_id;
        report_.committed_revision_before = revision_before;
        report_.committed_revision_after = current_map_revision_;
        report_.current_map_revision = current_map_revision_;
        report_.sparse_map_cell_count =
            sparse_backend_->report().active_cell_count;
        report_.map_from_odom_after_match =
            frame_state_.current_map_from_odom().map_T_odom;
        report_.last_atomic_commit_fault = "none";
        report_.last_fault = "none";
        report_.last_message = "atomic_local_slam_commit_accepted";
        clear_prepared_local_match_objects();
    }

    void clear_prepared_local_match_objects() {
        reference_map_snapshot_.reset();
        prepared_local_match_input_ = PreparedSparseTofLocalMatchInput{};
        local_match_result_.reset();
        report_.reference_snapshot_ready = false;
        report_.matcher_input_ready = false;
    }

    void clear_prepared_local_match_input() {
        reference_map_snapshot_.reset();
        prepared_local_match_input_ = PreparedSparseTofLocalMatchInput{};
        local_match_result_.reset();
        report_.matcher_executed = false;
        report_.matcher_evaluated_candidate_count = 0;
        report_.matcher_coarse_candidate_count = 0;
        report_.matcher_fine_candidate_count = 0;
        report_.matcher_used_ray_count = 0;
        report_.matcher_known_cell_count = 0;
        report_.matcher_unknown_cell_count = 0;
        report_.matcher_unknown_ratio = 0.0;
        report_.matcher_contradiction_count = 0;
        report_.matcher_best_yaw_rad = 0.0;
        report_.matcher_best_score = 0.0;
        report_.matcher_runner_up_available = false;
        report_.matcher_runner_up_yaw_rad = 0.0;
        report_.matcher_runner_up_score = 0.0;
        report_.matcher_score_margin = 0.0;
        report_.matcher_score_min = 0.0;
        report_.matcher_score_max = 0.0;
        report_.matcher_score_mean = 0.0;
        report_.matcher_score_range = 0.0;
        report_.matcher_best_at_search_edge = false;
        report_.matcher_flat_curve = false;
        report_.matcher_multimodal = false;
        report_.matcher_accepted = false;
        report_.matcher_status = "not_run";
        report_.matcher_rejection_reason = "none";
        report_.matcher_delta_yaw_rad = 0.0;
        report_.matcher_proposal_ready = false;
        report_.map_from_odom_before_match = RobotPose2D{};
        report_.proposed_map_from_odom = RobotPose2D{};
        report_.map_from_odom_after_match = RobotPose2D{};
        report_.reference_snapshot_ready = false;
        report_.reference_snapshot_revision = 0;
        report_.reference_snapshot_cell_count = 0;
        report_.reference_snapshot_occupied_cell_count = 0;
        report_.reference_snapshot_free_cell_count = 0;
        report_.reference_snapshot_uncertain_cell_count = 0;
        report_.reference_snapshot_resolution_m = 0.0;
        report_.matcher_input_ready = false;
        report_.matcher_input_bundle_id = 0;
        report_.matcher_input_reference_revision = 0;
        report_.matcher_input_bundle_frame_count = 0;
        report_.matcher_input_matchable_ray_count = 0;
        report_.matcher_input_status = "not_prepared";
        report_.last_matcher_input_fault = "none";
    }

    struct ObservationAppendBuildResult {
        bool ok = false;
        ActiveMultiTofObservationAppendInput input;
        std::string status;
        std::string message;
    };

    ObservationAppendBuildResult make_observation_append_input(
        const RobotSlamSensorSnapshot &snapshot,
        const MultiTofMeasurementPoseSet &poses) const {
        const auto front = pose_buffer_.lookup(
            snapshot.multi_tof.front.effective_timestamp_s);
        const auto left = pose_buffer_.lookup(
            snapshot.multi_tof.left.effective_timestamp_s);
        const auto right = pose_buffer_.lookup(
            snapshot.multi_tof.right.effective_timestamp_s);
        if (!front.ok || !left.ok || !right.ok) {
            return {false, {}, "measurement_odom_lookup_failed",
                    "active_bundle_measurement_odom_lookup_failed"};
        }
        ActiveMultiTofObservationAppendInput input;
        input.snapshot = snapshot;
        input.measurement_poses = poses;
        input.front_odom_pose_at_measurement = front.pose;
        input.left_odom_pose_at_measurement = left.pose;
        input.right_odom_pose_at_measurement = right.pose;
        input.sequence_index = report_.bundle_snapshot_attempt_count + 1;
        return {true, input, "ok", "active_bundle_append_input_ready"};
    }

    static MotionSettleSample make_motion_settle_sample(
        const RobotSlamSensorSnapshot &snapshot,
        const ImuFrame &corrected_imu) {
        MotionSettleSample sample;
        sample.timestamp_s = snapshot.wheel.timestamp_s;
        sample.wheel_fresh = snapshot.has_wheel && snapshot.wheel.valid;
        sample.linear_mps = snapshot.wheel.linear_mps;
        sample.wheel_angular_rad_s = snapshot.wheel.angular_rad_s;
        sample.imu_fresh = snapshot.has_imu;
        sample.imu_yaw_rate_rad_s = corrected_imu.yaw_rate_rad_s;
        return sample;
    }

    void sync_observation_report() {
        const auto &controller = observation_controller_.report();
        const auto &summary = observation_controller_.bundle_summary();
        report_.active_bundle_state = to_string(controller.state);
        report_.bundle_id = controller.bundle_id;
        report_.bundle_begin_count = controller.begin_count;
        report_.bundle_end_motion_count = controller.end_motion_count;
        report_.bundle_freeze_attempt_count = controller.freeze_attempt_count;
        report_.bundle_freeze_success_count = controller.freeze_success_count;
        report_.bundle_abort_count = controller.abort_count;
        report_.bundle_discard_count = controller.discard_count;
        report_.bundle_invalid_transition_count = controller.invalid_transition_count;
        report_.bundle_snapshot_attempt_count = controller.snapshot_attempt_count;
        report_.bundle_snapshot_accept_count = controller.snapshot_accept_count;
        report_.bundle_snapshot_reject_count = controller.snapshot_reject_count;
        report_.bundle_front_ray_count = summary.front_ray_count;
        report_.bundle_left_ray_count = summary.left_ray_count;
        report_.bundle_right_ray_count = summary.right_ray_count;
        report_.bundle_hit_ray_count = summary.hit_ray_count;
        report_.bundle_no_return_ray_count = summary.no_return_ray_count;
        report_.bundle_invalid_ray_count = summary.invalid_ray_count;
        report_.bundle_unspecified_ray_count = summary.unspecified_ray_count;
        report_.bundle_matchable_ray_count = summary.matchable_ray_count;
        report_.bundle_start_timestamp_s = summary.collection_start_timestamp_s;
        report_.bundle_end_timestamp_s = summary.collection_end_timestamp_s;
        report_.bundle_duration_s = summary.duration_s;
        report_.bundle_accumulated_abs_yaw_rad = summary.accumulated_abs_yaw_rad;
        report_.bundle_yaw_span_rad = summary.yaw_span_rad;
        report_.reference_map_revision = controller.reference_map_revision;
        report_.reference_map_cell_count = controller.reference_map_cell_count;
        report_.current_map_revision = current_map_revision_;
        report_.settle_update_count = controller.settle_update_count;
        report_.settle_reset_count = controller.settle_reset_count;
        report_.settle_consecutive_sample_count = controller.settle_consecutive_sample_count;
        report_.settle_stable_duration_s = controller.settle_stable_duration_s;
        report_.settle_success_count = controller.settle_success_count;
        report_.frozen_bundle_available = controller.frozen_bundle_available;
        report_.frozen_bundle_immutable = controller.frozen_bundle_immutable;
        report_.last_bundle_fault = controller.last_fault;
    }

    SparseSlamInitializationResult consume_startup_sample(
        const RobotSlamSensorSnapshot &snapshot) {
        if (!snapshot.has_wheel || !snapshot.has_imu ||
            !snapshot.wheel.valid ||
            !std::isfinite(snapshot.wheel.timestamp_s) ||
            !std::isfinite(snapshot.imu.timestamp_s) ||
            !std::isfinite(snapshot.wheel.linear_mps) ||
            !std::isfinite(snapshot.wheel.angular_rad_s) ||
            !std::isfinite(snapshot.imu.yaw_rate_rad_s)) {
            return startup_reject(
                SparseSlamInitializationStatus::SensorNotReady,
                "sparse_slam_startup_sensor_not_ready");
        }
        if (std::fabs(snapshot.wheel.linear_mps) >
                config_.sparse_slam_startup_max_abs_linear_speed_mps ||
            std::fabs(snapshot.wheel.angular_rad_s) >
                config_.sparse_slam_startup_max_abs_wheel_yaw_rate_rad_s ||
            std::fabs(snapshot.imu.yaw_rate_rad_s) >
                config_.sparse_slam_startup_max_abs_imu_yaw_rate_rad_s) {
            return startup_reject(
                SparseSlamInitializationStatus::SensorNotReady,
                "sparse_slam_startup_not_stationary");
        }

        gyro_samples_.push_back(snapshot.imu.yaw_rate_rad_s);
        report_.gyro_sample_count = gyro_samples_.size();
        const std::size_t required = static_cast<std::size_t>(
            std::max(1, config_.sparse_slam_startup_min_imu_samples));
        if (gyro_samples_.size() < required) {
            report_.stationary_check_passed = true;
            report_.initialization_status = to_string(
                SparseSlamInitializationStatus::WaitingForStationarySamples);
            return initialization_result(false,
                SparseSlamInitializationStatus::WaitingForStationarySamples,
                "sparse_slam_startup_collecting_stationary_samples");
        }

        const double sum = std::accumulate(
            gyro_samples_.begin(), gyro_samples_.end(), 0.0);
        gyro_bias_rad_s_ = sum / static_cast<double>(gyro_samples_.size());
        const auto minmax = std::minmax_element(
            gyro_samples_.begin(), gyro_samples_.end());
        const double spread = *minmax.second - *minmax.first;
        if (std::fabs(gyro_bias_rad_s_) >
            config_.sparse_slam_startup_max_gyro_bias_abs_rad_s) {
            return startup_reject(
                SparseSlamInitializationStatus::GyroCalibrationFailed,
                "sparse_slam_startup_gyro_bias_rejected");
        }
        if (spread > config_.sparse_slam_startup_max_gyro_spread_rad_s) {
            return startup_reject(
                SparseSlamInitializationStatus::GyroCalibrationFailed,
                "sparse_slam_startup_gyro_spread_rejected");
        }

        const auto frame_init = frame_state_.initialize(init_request_);
        if (!frame_init.ok) {
            return startup_reject(frame_init.status, frame_init.message);
        }
        const RobotPose2D initial_odom_pose{};
        const auto reset = estimator_.reset(initial_odom_pose);
        if (!reset.ok) {
            return startup_reject(
                SparseSlamInitializationStatus::WheelBaselineFailed,
                reset.message);
        }
        pose_buffer_.clear();
        phase_ = SparseSlamRuntimeCorePhase::Ready;
        report_.initialization_complete = true;
        report_.initialization_status =
            to_string(SparseSlamInitializationStatus::Ready);
        report_.startup_zero_used =
            init_request_.map_startup_mode == MapStartupMode::CreateNewMap &&
            init_request_.initial_pose_mode == InitialPoseMode::StartupZero;
        report_.configured_pose_used =
            init_request_.initial_pose_mode == InitialPoseMode::ConfiguredPose;
        report_.initial_yaw_measured_by_imu = false;
        report_.initial_yaw_defined_by_startup_frame =
            report_.startup_zero_used;
        report_.gyro_bias_rad_s = gyro_bias_rad_s_;
        report_.stationary_check_passed = true;
        report_.wheel_baseline_established = true;
        report_.map_from_odom_initialized = true;
        const auto map_from_odom = frame_state_.current_map_from_odom();
        report_.map_from_odom_x_m = map_from_odom.map_T_odom.x_m;
        report_.map_from_odom_y_m = map_from_odom.map_T_odom.y_m;
        report_.map_from_odom_yaw_rad = map_from_odom.map_T_odom.yaw_rad;
        report_.last_fault = "none";
        report_.last_message = "sparse_slam_startup_ready";
        return initialization_result(true,
            SparseSlamInitializationStatus::Ready,
            "sparse_slam_startup_ready");
    }

    static SparseSlamInitializationResult initialization_result(
        bool ok,
        SparseSlamInitializationStatus status,
        const std::string &message) {
        SparseSlamInitializationResult result;
        result.ok = ok;
        result.status = status;
        result.initial_yaw_measured_by_imu = false;
        result.initial_yaw_defined_by_startup_frame = false;
        result.message = message;
        return result;
    }

    SparseSlamInitializationResult startup_reject(
        SparseSlamInitializationStatus status,
        const std::string &message) {
        phase_ = SparseSlamRuntimeCorePhase::Fault;
        report_.initialization_status = to_string(status);
        report_.last_fault = report_.initialization_status;
        report_.last_message = message;
        return initialization_result(false, status, message);
    }

    Config config_;
    WheelImuDeadReckoning2D estimator_;
    MapOdomFrameState frame_state_;
    TimedOdomPoseBuffer pose_buffer_;
    MultiTofMeasurementPoseResolver resolver_;
    PhaseAwareObservationController observation_controller_;
    SparseTofLocalMatchInputValidator local_match_input_validator_;
    SparseTofLocalMatcher local_matcher_;
    SparseTofKeyframeManager keyframe_manager_;
    std::shared_ptr<const SparseTofLocalMatchResult> local_match_result_;
    std::shared_ptr<const SparseOccupancyGridSnapshot> reference_map_snapshot_;
    PreparedSparseTofLocalMatchInput prepared_local_match_input_;
    std::shared_ptr<SparseMultiTofOccupancyBackendBinding> sparse_backend_;
    SlamBackendMapPortAdapter map_port_;
    SparseSlamRuntimeCoreReport report_;
    SparseSlamRuntimeCorePhase phase_ = SparseSlamRuntimeCorePhase::Uninitialized;
    SparseSlamInitializationRequest init_request_;
    std::vector<double> gyro_samples_;
    double gyro_bias_rad_s_ = 0.0;
    std::uint64_t current_map_revision_ = 0;
};

} // namespace robot_slamd
