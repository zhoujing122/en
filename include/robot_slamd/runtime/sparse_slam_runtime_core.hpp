#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_contract_checker.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/runtime/multi_tof_measurement_pose_resolver.hpp"
#include "robot_slamd/runtime/sparse_slam_initialization.hpp"
#include "robot_slamd/runtime/timed_odom_pose_buffer.hpp"

#include <algorithm>
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
        (void)now_s;
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
            return {false, true, true, false, false,
                    report_.last_fault, report_.last_message};
        }
        report_.measurement_pose_set_success_count++;

        RobotSlamMapUpdateInput update;
        update.timestamp_s = snapshot.multi_tof.synchronized_time_s;
        update.snapshot = snapshot;
        update.predicted_map_pose = latest_map_pose.map_T_base;
        update.has_predicted_map_pose = true;
        update.multi_tof_measurement_poses = resolved.poses;
        update.has_multi_tof_measurement_poses = true;
        update.mapping_commit_allowed = true;
        update.source = "sparse_slam_runtime_core";
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
            report_.backend_accept_count++;
            report_.last_fault = "none";
            report_.last_message = "sparse_slam_core_backend_accepted";
        } else {
            report_.backend_reject_count++;
            report_.backend_fault_count++;
            report_.last_fault = "backend_rejected";
            report_.last_message = "sparse_slam_core_backend_rejected";
        }
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
    std::shared_ptr<SparseMultiTofOccupancyBackendBinding> sparse_backend_;
    SlamBackendMapPortAdapter map_port_;
    SparseSlamRuntimeCoreReport report_;
    SparseSlamRuntimeCorePhase phase_ = SparseSlamRuntimeCorePhase::Uninitialized;
    SparseSlamInitializationRequest init_request_;
    std::vector<double> gyro_samples_;
    double gyro_bias_rad_s_ = 0.0;
};

} // namespace robot_slamd
