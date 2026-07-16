#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <cmath>

namespace robot_slamd {

inline bool se2_finite(double value) {
    return std::isfinite(value);
}

inline bool se2_pose_finite(const RobotPose2D &pose) {
    return se2_finite(pose.x_m) &&
           se2_finite(pose.y_m) &&
           se2_finite(pose.yaw_rad);
}

inline double se2_normalize_yaw(double yaw_rad) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    if (!std::isfinite(yaw_rad)) {
        return yaw_rad;
    }
    while (yaw_rad > kPi) {
        yaw_rad -= kTwoPi;
    }
    while (yaw_rad < -kPi) {
        yaw_rad += kTwoPi;
    }
    return yaw_rad;
}

inline RobotPose2D se2_compose(const RobotPose2D &a,
                               const RobotPose2D &b) {
    RobotPose2D out;
    const double c = std::cos(a.yaw_rad);
    const double s = std::sin(a.yaw_rad);
    out.x_m = a.x_m + c * b.x_m - s * b.y_m;
    out.y_m = a.y_m + s * b.x_m + c * b.y_m;
    out.yaw_rad = se2_normalize_yaw(a.yaw_rad + b.yaw_rad);
    return out;
}

inline RobotPose2D se2_inverse(const RobotPose2D &pose) {
    RobotPose2D out;
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    out.x_m = -c * pose.x_m - s * pose.y_m;
    out.y_m = s * pose.x_m - c * pose.y_m;
    out.yaw_rad = se2_normalize_yaw(-pose.yaw_rad);
    return out;
}

inline RobotPose2D se2_between(const RobotPose2D &from,
                               const RobotPose2D &to) {
    return se2_compose(se2_inverse(from), to);
}

inline double se2_squared_translation_norm(const RobotPose2D &pose) {
    return pose.x_m * pose.x_m + pose.y_m * pose.y_m;
}

inline double se2_abs_yaw(double yaw_rad) {
    return std::fabs(se2_normalize_yaw(yaw_rad));
}

} // namespace robot_slamd
