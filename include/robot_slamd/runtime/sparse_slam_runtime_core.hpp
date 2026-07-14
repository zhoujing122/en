#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_contract_checker.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"
#include "robot_slamd/config/config.hpp"
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
          sparse_backend_(std::make_shared<SparseMultiTofOccupancyBackendBinding>(
              make_backend_options())),
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
        init_request_ = request;
        const auto preflight = frame_state_.initialize(request);
        if (!preflight.ok) {
            phase_ = SparseSlamRuntimeCorePhase::Fault;
            report_.initialization_status = to_string(preflight.status);
            report_.last_fault = report_.initialization_status;
            report_.last_message = preflight.message;
            return preflight;
        }
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

private:
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

    static SparseMultiTofOccupancyBackendOptions make_backend_options() {
        SparseMultiTofOccupancyBackendOptions out;
        out.minimum_accepted_snapshots_for_good_quality = 1;
        out.minimum_valid_rays_for_good_quality = 1;
        out.minimum_observed_cells_for_good_quality = 1;
        out.minimum_angular_bins_for_good_quality = 1;
        out.require_multi_tof_measurement_poses = true;
        out.allow_single_pose_fallback = false;
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
            init_request_.map_startup_mode == MapStartupMode::CreateNewMap &&
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
