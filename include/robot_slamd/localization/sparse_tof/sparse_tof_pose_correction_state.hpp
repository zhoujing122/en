#pragma once

#include "robot_slamd/localization/sparse_tof/se2_pose_transform.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace robot_slamd {

enum class InitialPoseMode {
    StartupZero,
    ConfiguredPose,
    Relocalization
};

enum class SparseTofPoseCorrectionFault {
    None,
    InvalidConfig,
    InvalidInitialPose,
    ExistingMapRequiresInitialPose,
    NotInitialized,
    InvalidOdomPose,
    InvalidMatchedPose,
    TranslationJumpRejected,
    YawJumpRejected,
    NoPreparedCorrection
};

inline std::string to_string(InitialPoseMode mode) {
    switch (mode) {
    case InitialPoseMode::StartupZero: return "startup_zero";
    case InitialPoseMode::ConfiguredPose: return "configured_pose";
    case InitialPoseMode::Relocalization: return "relocalization";
    }
    return "unknown";
}

inline std::string to_string(SparseTofPoseCorrectionFault fault) {
    switch (fault) {
    case SparseTofPoseCorrectionFault::None: return "none";
    case SparseTofPoseCorrectionFault::InvalidConfig: return "invalid_config";
    case SparseTofPoseCorrectionFault::InvalidInitialPose: return "invalid_initial_pose";
    case SparseTofPoseCorrectionFault::ExistingMapRequiresInitialPose: return "existing_map_requires_initial_pose";
    case SparseTofPoseCorrectionFault::NotInitialized: return "not_initialized";
    case SparseTofPoseCorrectionFault::InvalidOdomPose: return "invalid_odom_pose";
    case SparseTofPoseCorrectionFault::InvalidMatchedPose: return "invalid_matched_pose";
    case SparseTofPoseCorrectionFault::TranslationJumpRejected: return "translation_jump_rejected";
    case SparseTofPoseCorrectionFault::YawJumpRejected: return "yaw_jump_rejected";
    case SparseTofPoseCorrectionFault::NoPreparedCorrection: return "no_prepared_correction";
    }
    return "unknown";
}

struct SparseTofPoseCorrectionConfig {
    double maximum_translation_jump_m = 0.35;
    double maximum_yaw_jump_rad = 0.35;
    std::size_t maximum_history_count = 16;
};

struct SparseTofPoseCorrectionResult {
    bool ok = false;
    SparseTofPoseCorrectionFault fault = SparseTofPoseCorrectionFault::None;
    std::string message;
    RobotPose2D map_from_odom;
    RobotPose2D corrected_pose;
};

class SparseTofPoseCorrectionState {
public:
    explicit SparseTofPoseCorrectionState(SparseTofPoseCorrectionConfig config = {})
        : config_(config) {}

    bool valid_config() const {
        return std::isfinite(config_.maximum_translation_jump_m) &&
               config_.maximum_translation_jump_m >= 0.0 &&
               std::isfinite(config_.maximum_yaw_jump_rad) &&
               config_.maximum_yaw_jump_rad >= 0.0 &&
               config_.maximum_history_count > 0;
    }

    SparseTofPoseCorrectionResult reset_new_map() {
        RobotPose2D zero;
        return reset_impl(InitialPoseMode::StartupZero, zero, zero, false);
    }

    SparseTofPoseCorrectionResult reset_configured_pose(
        const RobotPose2D &initial_map_pose,
        const RobotPose2D &initial_odom_pose = RobotPose2D{}) {
        return reset_impl(InitialPoseMode::ConfiguredPose, initial_map_pose,
                          initial_odom_pose, false);
    }

    SparseTofPoseCorrectionResult reset_existing_map_without_pose() {
        SparseTofPoseCorrectionResult result;
        result.fault = SparseTofPoseCorrectionFault::ExistingMapRequiresInitialPose;
        result.message = "existing_map_requires_initial_pose";
        return result;
    }

    bool initialized() const { return initialized_; }
    InitialPoseMode initial_pose_mode() const { return mode_; }
    const RobotPose2D &map_from_odom() const { return map_from_odom_; }
    std::size_t correction_count() const { return correction_count_; }
    std::size_t history_count() const { return history_.size(); }
    bool has_prepared_correction() const { return has_prepared_; }

    RobotPose2D corrected_pose(const RobotPose2D &odom_pose) const {
        return se2_compose(map_from_odom_, odom_pose);
    }

    SparseTofPoseCorrectionResult prepare_correction(
        const RobotPose2D &current_odom_pose,
        const RobotPose2D &matched_map_pose) {
        SparseTofPoseCorrectionResult result;
        if (!valid_config()) {
            result.fault = SparseTofPoseCorrectionFault::InvalidConfig;
            result.message = "pose_correction_invalid_config";
            return result;
        }
        if (!initialized_) {
            result.fault = SparseTofPoseCorrectionFault::NotInitialized;
            result.message = "pose_correction_not_initialized";
            return result;
        }
        if (!se2_pose_finite(current_odom_pose)) {
            result.fault = SparseTofPoseCorrectionFault::InvalidOdomPose;
            result.message = "pose_correction_invalid_odom_pose";
            return result;
        }
        if (!se2_pose_finite(matched_map_pose)) {
            result.fault = SparseTofPoseCorrectionFault::InvalidMatchedPose;
            result.message = "pose_correction_invalid_matched_pose";
            return result;
        }
        const RobotPose2D candidate = se2_compose(matched_map_pose,
                                                  se2_inverse(current_odom_pose));
        const RobotPose2D jump = se2_between(map_from_odom_, candidate);
        if (std::sqrt(se2_squared_translation_norm(jump)) > config_.maximum_translation_jump_m) {
            result.fault = SparseTofPoseCorrectionFault::TranslationJumpRejected;
            result.message = "pose_correction_translation_jump_rejected";
            return result;
        }
        if (se2_abs_yaw(jump.yaw_rad) > config_.maximum_yaw_jump_rad) {
            result.fault = SparseTofPoseCorrectionFault::YawJumpRejected;
            result.message = "pose_correction_yaw_jump_rejected";
            return result;
        }
        prepared_map_from_odom_ = candidate;
        prepared_corrected_pose_ = matched_map_pose;
        has_prepared_ = true;
        result.ok = true;
        result.fault = SparseTofPoseCorrectionFault::None;
        result.message = "pose_correction_prepared";
        result.map_from_odom = candidate;
        result.corrected_pose = matched_map_pose;
        return result;
    }

    SparseTofPoseCorrectionResult commit_prepared_correction() {
        SparseTofPoseCorrectionResult result;
        if (!has_prepared_) {
            result.fault = SparseTofPoseCorrectionFault::NoPreparedCorrection;
            result.message = "pose_correction_no_prepared_correction";
            return result;
        }
        map_from_odom_ = prepared_map_from_odom_;
        has_prepared_ = false;
        correction_count_++;
        history_.push_back(map_from_odom_);
        if (history_.size() > config_.maximum_history_count) {
            history_.erase(history_.begin());
        }
        result.ok = true;
        result.fault = SparseTofPoseCorrectionFault::None;
        result.message = "pose_correction_committed";
        result.map_from_odom = map_from_odom_;
        result.corrected_pose = prepared_corrected_pose_;
        return result;
    }

    void abort_prepared_correction() {
        has_prepared_ = false;
    }

private:
    SparseTofPoseCorrectionConfig config_;
    bool initialized_ = false;
    InitialPoseMode mode_ = InitialPoseMode::StartupZero;
    RobotPose2D map_from_odom_;
    RobotPose2D prepared_map_from_odom_;
    RobotPose2D prepared_corrected_pose_;
    bool has_prepared_ = false;
    std::size_t correction_count_ = 0;
    std::vector<RobotPose2D> history_;

    SparseTofPoseCorrectionResult reset_impl(InitialPoseMode mode,
                                             const RobotPose2D &initial_map_pose,
                                             const RobotPose2D &initial_odom_pose,
                                             bool relocalized) {
        (void)relocalized;
        SparseTofPoseCorrectionResult result;
        if (!valid_config()) {
            result.fault = SparseTofPoseCorrectionFault::InvalidConfig;
            result.message = "pose_correction_invalid_config";
            return result;
        }
        if (!se2_pose_finite(initial_map_pose) || !se2_pose_finite(initial_odom_pose)) {
            result.fault = SparseTofPoseCorrectionFault::InvalidInitialPose;
            result.message = "pose_correction_invalid_initial_pose";
            return result;
        }
        mode_ = mode;
        map_from_odom_ = se2_compose(initial_map_pose, se2_inverse(initial_odom_pose));
        map_from_odom_.yaw_rad = se2_normalize_yaw(map_from_odom_.yaw_rad);
        initialized_ = true;
        has_prepared_ = false;
        correction_count_ = 0;
        history_.clear();
        result.ok = true;
        result.fault = SparseTofPoseCorrectionFault::None;
        result.message = "pose_correction_reset";
        result.map_from_odom = map_from_odom_;
        result.corrected_pose = initial_map_pose;
        return result;
    }
};

} // namespace robot_slamd
