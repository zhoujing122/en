#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <cmath>
#include <sstream>
#include <string>

namespace robot_slamd {

enum class WheelImuDeadReckoningFault {
    None,
    InvalidConfig,
    InvalidInitialPose,
    WheelMissing,
    WheelNotFresh,
    WheelTimestampInvalid,
    WheelTimestampNotMonotonic,
    WheelDeltaRejected,
    ImuMissing,
    ImuNotFresh,
    ImuTimestampInvalid,
    ImuTimestampNotMonotonic,
    TimestampMismatch,
    DeltaTimeRejected,
    YawDisagreementRejected,
    PoseNonFinite
};

enum class WheelImuDeadReckoningStatus {
    Reset,
    InitializedWithoutMotion,
    Updated,
    Rejected
};

struct WheelImuDeadReckoningConfig {
    double wheel_radius_m = 0.032;
    double wheel_base_m = 0.160;
    double maximum_dt_s = 0.50;
    double maximum_abs_wheel_linear_mps = 2.0;
    double maximum_abs_wheel_angular_rad_s = 8.0;
    double maximum_abs_imu_yaw_rate_rad_s = 8.0;
    double timestamp_tolerance_s = 0.02;
    bool allow_imu_missing_wheel_yaw_fallback = false;
    bool enable_wheel_imu_yaw_consistency_gate = true;
    double maximum_yaw_rate_disagreement_rad_s = 2.0;
};

struct WheelImuDeadReckoningResult {
    bool ok = false;
    WheelImuDeadReckoningStatus status =
        WheelImuDeadReckoningStatus::Rejected;
    WheelImuDeadReckoningFault fault = WheelImuDeadReckoningFault::None;
    std::string message;
};

inline std::string to_string(WheelImuDeadReckoningFault fault) {
    switch (fault) {
    case WheelImuDeadReckoningFault::None:
        return "none";
    case WheelImuDeadReckoningFault::InvalidConfig:
        return "invalid_config";
    case WheelImuDeadReckoningFault::InvalidInitialPose:
        return "invalid_initial_pose";
    case WheelImuDeadReckoningFault::WheelMissing:
        return "wheel_missing";
    case WheelImuDeadReckoningFault::WheelNotFresh:
        return "wheel_not_fresh";
    case WheelImuDeadReckoningFault::WheelTimestampInvalid:
        return "wheel_timestamp_invalid";
    case WheelImuDeadReckoningFault::WheelTimestampNotMonotonic:
        return "wheel_timestamp_not_monotonic";
    case WheelImuDeadReckoningFault::WheelDeltaRejected:
        return "wheel_delta_rejected";
    case WheelImuDeadReckoningFault::ImuMissing:
        return "imu_missing";
    case WheelImuDeadReckoningFault::ImuNotFresh:
        return "imu_not_fresh";
    case WheelImuDeadReckoningFault::ImuTimestampInvalid:
        return "imu_timestamp_invalid";
    case WheelImuDeadReckoningFault::ImuTimestampNotMonotonic:
        return "imu_timestamp_not_monotonic";
    case WheelImuDeadReckoningFault::TimestampMismatch:
        return "timestamp_mismatch";
    case WheelImuDeadReckoningFault::DeltaTimeRejected:
        return "delta_time_rejected";
    case WheelImuDeadReckoningFault::YawDisagreementRejected:
        return "yaw_disagreement_rejected";
    case WheelImuDeadReckoningFault::PoseNonFinite:
        return "pose_non_finite";
    }
    return "unknown";
}

inline std::string to_string(WheelImuDeadReckoningStatus status) {
    switch (status) {
    case WheelImuDeadReckoningStatus::Reset:
        return "reset";
    case WheelImuDeadReckoningStatus::InitializedWithoutMotion:
        return "initialized_without_motion";
    case WheelImuDeadReckoningStatus::Updated:
        return "updated";
    case WheelImuDeadReckoningStatus::Rejected:
        return "rejected";
    }
    return "unknown";
}

class WheelImuDeadReckoning2D {
public:
    explicit WheelImuDeadReckoning2D(
        WheelImuDeadReckoningConfig config = {})
        : config_(config) {}

    bool config_valid() const {
        return finite(config_.wheel_radius_m) && config_.wheel_radius_m > 0.0 &&
               finite(config_.wheel_base_m) && config_.wheel_base_m > 0.0 &&
               finite(config_.maximum_dt_s) && config_.maximum_dt_s > 0.0 &&
               finite(config_.maximum_abs_wheel_linear_mps) &&
               config_.maximum_abs_wheel_linear_mps >= 0.0 &&
               finite(config_.maximum_abs_wheel_angular_rad_s) &&
               config_.maximum_abs_wheel_angular_rad_s >= 0.0 &&
               finite(config_.maximum_abs_imu_yaw_rate_rad_s) &&
               config_.maximum_abs_imu_yaw_rate_rad_s >= 0.0 &&
               finite(config_.timestamp_tolerance_s) &&
               config_.timestamp_tolerance_s >= 0.0 &&
               finite(config_.maximum_yaw_rate_disagreement_rad_s) &&
               config_.maximum_yaw_rate_disagreement_rad_s >= 0.0;
    }

    WheelImuDeadReckoningResult reset(
        const RobotPose2D &initial_pose = {}) {
        if (!config_valid()) {
            return reject(WheelImuDeadReckoningFault::InvalidConfig,
                          "wheel_imu_dr_invalid_config");
        }
        if (!pose_finite(initial_pose)) {
            return reject(WheelImuDeadReckoningFault::InvalidInitialPose,
                          "wheel_imu_dr_invalid_initial_pose");
        }
        pose_ = initial_pose;
        pose_.yaw_rad = normalize_yaw(pose_.yaw_rad);
        previous_wheel_ = WheelOdomFrame{};
        previous_imu_ = ImuFrame{};
        have_previous_ = false;
        initialized_ = true;
        last_fault_ = WheelImuDeadReckoningFault::None;
        last_message_ = "wheel_imu_dr_reset";
        return {true, WheelImuDeadReckoningStatus::Reset,
                WheelImuDeadReckoningFault::None, last_message_};
    }

    WheelImuDeadReckoningResult update(
        const WheelOdomFrame &wheel,
        const ImuFrame &imu,
        bool imu_fresh = true) {
        if (!initialized_) {
            auto reset_result = reset();
            if (!reset_result.ok) {
                return reset_result;
            }
        }
        if (!config_valid()) {
            return reject(WheelImuDeadReckoningFault::InvalidConfig,
                          "wheel_imu_dr_invalid_config");
        }
        const auto wheel_validation = validate_wheel(wheel);
        if (!wheel_validation.ok) {
            return wheel_validation;
        }
        const auto imu_validation = validate_imu(imu, imu_fresh);
        if (!imu_validation.ok) {
            return imu_validation;
        }
        if (std::fabs(wheel.timestamp_s - imu.timestamp_s) >
            config_.timestamp_tolerance_s) {
            return reject(WheelImuDeadReckoningFault::TimestampMismatch,
                          "wheel_imu_dr_timestamp_mismatch");
        }

        if (!have_previous_) {
            previous_wheel_ = wheel;
            previous_imu_ = imu;
            have_previous_ = true;
            last_fault_ = WheelImuDeadReckoningFault::None;
            last_message_ = "wheel_imu_dr_initialized_without_motion";
            return {true, WheelImuDeadReckoningStatus::InitializedWithoutMotion,
                    WheelImuDeadReckoningFault::None, last_message_};
        }

        if (wheel.timestamp_s <= previous_wheel_.timestamp_s) {
            return reject(WheelImuDeadReckoningFault::WheelTimestampNotMonotonic,
                          "wheel_imu_dr_wheel_time_not_monotonic");
        }
        if (imu.timestamp_s <= previous_imu_.timestamp_s) {
            return reject(WheelImuDeadReckoningFault::ImuTimestampNotMonotonic,
                          "wheel_imu_dr_imu_time_not_monotonic");
        }

        const double dt_s = wheel.timestamp_s - previous_wheel_.timestamp_s;
        if (!finite(dt_s) || dt_s <= 0.0 || dt_s > config_.maximum_dt_s) {
            return reject(WheelImuDeadReckoningFault::DeltaTimeRejected,
                          "wheel_imu_dr_dt_rejected");
        }

        if (config_.enable_wheel_imu_yaw_consistency_gate && imu_fresh &&
            std::fabs(wheel.angular_rad_s - imu.yaw_rate_rad_s) >
                config_.maximum_yaw_rate_disagreement_rad_s) {
            return reject(WheelImuDeadReckoningFault::YawDisagreementRejected,
                          "wheel_imu_dr_yaw_disagreement");
        }

        const double delta_s = wheel.linear_mps * dt_s;
        const double yaw_rate = imu_fresh ? imu.yaw_rate_rad_s
                                          : wheel.angular_rad_s;
        const double delta_yaw = yaw_rate * dt_s;
        RobotPose2D candidate = pose_;
        const double yaw_mid = pose_.yaw_rad + 0.5 * delta_yaw;
        candidate.x_m += delta_s * std::cos(yaw_mid);
        candidate.y_m += delta_s * std::sin(yaw_mid);
        candidate.yaw_rad = normalize_yaw(pose_.yaw_rad + delta_yaw);
        if (!pose_finite(candidate)) {
            return reject(WheelImuDeadReckoningFault::PoseNonFinite,
                          "wheel_imu_dr_pose_non_finite");
        }

        pose_ = candidate;
        previous_wheel_ = wheel;
        previous_imu_ = imu;
        last_fault_ = WheelImuDeadReckoningFault::None;
        last_message_ = "wheel_imu_dr_updated";
        return {true, WheelImuDeadReckoningStatus::Updated,
                WheelImuDeadReckoningFault::None, last_message_};
    }

    const RobotPose2D &estimated_pose() const { return pose_; }
    bool initialized() const { return initialized_; }
    bool has_previous_sample() const { return have_previous_; }
    WheelImuDeadReckoningFault last_fault() const { return last_fault_; }
    std::string last_message() const { return last_message_; }

    std::string status() const {
        std::ostringstream out;
        out << "wheel_imu_dead_reckoning initialized="
            << (initialized_ ? 1 : 0)
            << " has_previous=" << (have_previous_ ? 1 : 0)
            << " fault=" << to_string(last_fault_)
            << " message=" << last_message_;
        return out.str();
    }

private:
    static bool finite(double value) {
        return std::isfinite(value);
    }

    static bool pose_finite(const RobotPose2D &pose) {
        return finite(pose.x_m) && finite(pose.y_m) && finite(pose.yaw_rad);
    }

    static double normalize_yaw(double yaw_rad) {
        const double pi = 3.14159265358979323846;
        double yaw = std::fmod(yaw_rad + pi, 2.0 * pi);
        if (yaw < 0.0) {
            yaw += 2.0 * pi;
        }
        return yaw - pi;
    }

    WheelImuDeadReckoningResult validate_wheel(
        const WheelOdomFrame &wheel) const {
        if (!wheel.valid) {
            return {false, WheelImuDeadReckoningStatus::Rejected,
                    WheelImuDeadReckoningFault::WheelNotFresh,
                    "wheel_imu_dr_wheel_not_fresh"};
        }
        if (!finite(wheel.timestamp_s)) {
            return {false, WheelImuDeadReckoningStatus::Rejected,
                    WheelImuDeadReckoningFault::WheelTimestampInvalid,
                    "wheel_imu_dr_wheel_timestamp_invalid"};
        }
        if (!finite(wheel.linear_mps) || !finite(wheel.angular_rad_s) ||
            std::fabs(wheel.linear_mps) >
                config_.maximum_abs_wheel_linear_mps ||
            std::fabs(wheel.angular_rad_s) >
                config_.maximum_abs_wheel_angular_rad_s) {
            return {false, WheelImuDeadReckoningStatus::Rejected,
                    WheelImuDeadReckoningFault::WheelDeltaRejected,
                    "wheel_imu_dr_wheel_delta_rejected"};
        }
        return {true, WheelImuDeadReckoningStatus::Updated,
                WheelImuDeadReckoningFault::None, ""};
    }

    WheelImuDeadReckoningResult validate_imu(
        const ImuFrame &imu,
        bool imu_fresh) const {
        if (!imu_fresh && !config_.allow_imu_missing_wheel_yaw_fallback) {
            return {false, WheelImuDeadReckoningStatus::Rejected,
                    WheelImuDeadReckoningFault::ImuNotFresh,
                    "wheel_imu_dr_imu_not_fresh"};
        }
        if (!finite(imu.timestamp_s)) {
            return {false, WheelImuDeadReckoningStatus::Rejected,
                    WheelImuDeadReckoningFault::ImuTimestampInvalid,
                    "wheel_imu_dr_imu_timestamp_invalid"};
        }
        if (imu_fresh &&
            (!finite(imu.yaw_rate_rad_s) ||
             std::fabs(imu.yaw_rate_rad_s) >
                 config_.maximum_abs_imu_yaw_rate_rad_s)) {
            return {false, WheelImuDeadReckoningStatus::Rejected,
                    WheelImuDeadReckoningFault::ImuNotFresh,
                    "wheel_imu_dr_imu_not_fresh"};
        }
        return {true, WheelImuDeadReckoningStatus::Updated,
                WheelImuDeadReckoningFault::None, ""};
    }

    WheelImuDeadReckoningResult reject(
        WheelImuDeadReckoningFault fault,
        const std::string &message) {
        last_fault_ = fault;
        last_message_ = message;
        return {false, WheelImuDeadReckoningStatus::Rejected, fault, message};
    }

    WheelImuDeadReckoningConfig config_;
    RobotPose2D pose_;
    WheelOdomFrame previous_wheel_;
    ImuFrame previous_imu_;
    bool initialized_ = false;
    bool have_previous_ = false;
    WheelImuDeadReckoningFault last_fault_ =
        WheelImuDeadReckoningFault::None;
    std::string last_message_ = "wheel_imu_dr_not_initialized";
};

} // namespace robot_slamd
