#pragma once

#include "robot_slamd/runtime/sparse_slam_frames.hpp"

#include <string>

namespace robot_slamd {

enum class MapStartupMode {
    CreateNewMap,
    LoadExistingMap
};

enum class InitialPoseMode {
    StartupZero,
    ConfiguredPose,
    Relocalization
};

enum class SparseSlamInitializationStatus {
    Ready,
    WaitingForStationarySamples,
    InvalidConfiguration,
    InitialPoseMissing,
    ExistingMapLoadNotImplemented,
    RelocalizationNotImplemented,
    SensorNotReady,
    GyroCalibrationFailed,
    WheelBaselineFailed,
    Fault
};

struct SparseSlamInitializationRequest {
    MapStartupMode map_startup_mode = MapStartupMode::CreateNewMap;
    InitialPoseMode initial_pose_mode = InitialPoseMode::StartupZero;
    bool has_configured_map_pose = false;
    MapPose2D configured_map_pose;
    std::string map_path;
    bool existing_map_loaded = false;
};

struct SparseSlamInitializationResult {
    bool ok = false;
    SparseSlamInitializationStatus status =
        SparseSlamInitializationStatus::InvalidConfiguration;
    bool initial_yaw_measured_by_imu = false;
    bool initial_yaw_defined_by_startup_frame = false;
    bool startup_zero_used = false;
    bool configured_pose_used = false;
    std::string message;
};

inline const char *to_string(MapStartupMode mode) {
    switch (mode) {
    case MapStartupMode::CreateNewMap:
        return "create_new";
    case MapStartupMode::LoadExistingMap:
        return "load_existing";
    }
    return "unknown";
}

inline const char *to_string(InitialPoseMode mode) {
    switch (mode) {
    case InitialPoseMode::StartupZero:
        return "startup_zero";
    case InitialPoseMode::ConfiguredPose:
        return "configured_pose";
    case InitialPoseMode::Relocalization:
        return "relocalization";
    }
    return "unknown";
}

inline const char *to_string(SparseSlamInitializationStatus status) {
    switch (status) {
    case SparseSlamInitializationStatus::Ready:
        return "ready";
    case SparseSlamInitializationStatus::WaitingForStationarySamples:
        return "waiting_for_stationary_samples";
    case SparseSlamInitializationStatus::InvalidConfiguration:
        return "invalid_configuration";
    case SparseSlamInitializationStatus::InitialPoseMissing:
        return "initial_pose_missing";
    case SparseSlamInitializationStatus::ExistingMapLoadNotImplemented:
        return "existing_map_load_not_implemented";
    case SparseSlamInitializationStatus::RelocalizationNotImplemented:
        return "relocalization_not_implemented";
    case SparseSlamInitializationStatus::SensorNotReady:
        return "sensor_not_ready";
    case SparseSlamInitializationStatus::GyroCalibrationFailed:
        return "gyro_calibration_failed";
    case SparseSlamInitializationStatus::WheelBaselineFailed:
        return "wheel_baseline_failed";
    case SparseSlamInitializationStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline bool parse_map_startup_mode(const std::string &text,
                                   MapStartupMode &mode) {
    if (text == "create_new") {
        mode = MapStartupMode::CreateNewMap;
        return true;
    }
    if (text == "load_existing") {
        mode = MapStartupMode::LoadExistingMap;
        return true;
    }
    return false;
}

inline bool parse_initial_pose_mode(const std::string &text,
                                    InitialPoseMode &mode) {
    if (text == "startup_zero") {
        mode = InitialPoseMode::StartupZero;
        return true;
    }
    if (text == "configured_pose") {
        mode = InitialPoseMode::ConfiguredPose;
        return true;
    }
    if (text == "relocalization") {
        mode = InitialPoseMode::Relocalization;
        return true;
    }
    return false;
}

class MapOdomFrameState {
public:
    SparseSlamInitializationResult initialize_startup_zero() {
        SparseSlamInitializationRequest request;
        request.map_startup_mode = MapStartupMode::CreateNewMap;
        request.initial_pose_mode = InitialPoseMode::StartupZero;
        return initialize(request);
    }

    SparseSlamInitializationResult initialize_configured_pose(
        const MapPose2D &map_pose) {
        SparseSlamInitializationRequest request;
        request.map_startup_mode = MapStartupMode::CreateNewMap;
        request.initial_pose_mode = InitialPoseMode::ConfiguredPose;
        request.has_configured_map_pose = true;
        request.configured_map_pose = map_pose;
        return initialize(request);
    }

    SparseSlamInitializationResult initialize(
        const SparseSlamInitializationRequest &request) {
        if (request.map_startup_mode == MapStartupMode::CreateNewMap &&
            request.initial_pose_mode == InitialPoseMode::StartupZero) {
            return commit(identity_map_from_odom(),
                          true,
                          false,
                          "startup_zero_ready");
        }

        if (request.map_startup_mode == MapStartupMode::CreateNewMap &&
            request.initial_pose_mode == InitialPoseMode::ConfiguredPose) {
            if (!request.has_configured_map_pose ||
                !sparse_slam_frame_pose_valid(request.configured_map_pose)) {
                return failure(SparseSlamInitializationStatus::InitialPoseMissing,
                               "configured_pose_missing_or_invalid");
            }
            return commit(make_map_from_odom(request.configured_map_pose.map_T_base),
                          false,
                          true,
                          "configured_pose_ready");
        }

        if (request.map_startup_mode == MapStartupMode::CreateNewMap &&
            request.initial_pose_mode == InitialPoseMode::Relocalization) {
            return failure(SparseSlamInitializationStatus::RelocalizationNotImplemented,
                           "relocalization_not_implemented_for_new_map");
        }

        if (request.map_startup_mode == MapStartupMode::LoadExistingMap &&
            request.initial_pose_mode == InitialPoseMode::ConfiguredPose) {
            if (!request.has_configured_map_pose ||
                !sparse_slam_frame_pose_valid(request.configured_map_pose)) {
                return failure(SparseSlamInitializationStatus::InitialPoseMissing,
                               "existing_map_configured_pose_missing");
            }
            if (!request.existing_map_loaded) {
                return failure(
                    SparseSlamInitializationStatus::ExistingMapLoadNotImplemented,
                    "existing_sparse_map_not_loaded");
            }
            return commit(make_map_from_odom(
                              request.configured_map_pose.map_T_base),
                          false, true,
                          "existing_map_configured_pose_ready");
        }

        if (request.map_startup_mode == MapStartupMode::LoadExistingMap &&
            request.initial_pose_mode == InitialPoseMode::Relocalization) {
            return failure(SparseSlamInitializationStatus::RelocalizationNotImplemented,
                           "existing_map_relocalization_not_implemented");
        }

        if (request.map_startup_mode == MapStartupMode::LoadExistingMap) {
            return failure(SparseSlamInitializationStatus::InitialPoseMissing,
                           "existing_map_requires_initial_pose");
        }

        return failure(SparseSlamInitializationStatus::InvalidConfiguration,
                       "invalid_initialization_combination");
    }

    bool initialized() const { return initialized_; }

    MapFromOdom2D current_map_from_odom() const {
        return map_from_odom_;
    }

    MapPose2D map_pose_from_odom(const OdomPose2D &odom_pose) const {
        return compose(map_from_odom_, odom_pose);
    }

    bool can_apply_local_slam_transform(
        const MapFromOdom2D &transform) const {
        return initialized_ && sparse_slam_frame_pose_valid(transform);
    }

    void commit_local_slam_transform(
        const MapFromOdom2D &transform) noexcept {
        map_from_odom_ = transform;
    }

    bool startup_zero_used() const { return startup_zero_used_; }
    bool configured_pose_used() const { return configured_pose_used_; }

private:
    SparseSlamInitializationResult commit(const MapFromOdom2D &transform,
                                          bool startup_zero,
                                          bool configured_pose,
                                          const std::string &message) {
        SparseSlamInitializationResult result;
        if (!sparse_slam_frame_pose_valid(transform)) {
            return failure(SparseSlamInitializationStatus::InvalidConfiguration,
                           "map_from_odom_invalid");
        }
        map_from_odom_ = transform;
        initialized_ = true;
        startup_zero_used_ = startup_zero;
        configured_pose_used_ = configured_pose;
        result.ok = true;
        result.status = SparseSlamInitializationStatus::Ready;
        result.initial_yaw_measured_by_imu = false;
        result.initial_yaw_defined_by_startup_frame = startup_zero;
        result.startup_zero_used = startup_zero;
        result.configured_pose_used = configured_pose;
        result.message = message;
        return result;
    }

    static SparseSlamInitializationResult failure(
        SparseSlamInitializationStatus status,
        const std::string &message) {
        SparseSlamInitializationResult result;
        result.ok = false;
        result.status = status;
        result.initial_yaw_measured_by_imu = false;
        result.initial_yaw_defined_by_startup_frame = false;
        result.message = message;
        return result;
    }

    bool initialized_ = false;
    bool startup_zero_used_ = false;
    bool configured_pose_used_ = false;
    MapFromOdom2D map_from_odom_;
};

} // namespace robot_slamd
