#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <cmath>
#include <string>

namespace robot_slamd {

struct OdomPose2D {
    RobotPose2D odom_T_base;
};

struct MapPose2D {
    RobotPose2D map_T_base;
};

struct MapFromOdom2D {
    RobotPose2D map_T_odom;
};

inline double sparse_slam_normalize_yaw(double yaw_rad) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    if (!std::isfinite(yaw_rad)) {
        return yaw_rad;
    }
    double wrapped = std::fmod(yaw_rad + kPi, kTwoPi);
    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }
    return wrapped - kPi;
}

inline bool sparse_slam_pose_finite(const RobotPose2D &pose) {
    return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
           std::isfinite(pose.yaw_rad);
}

inline RobotPose2D sparse_slam_normalized_pose(RobotPose2D pose) {
    pose.yaw_rad = sparse_slam_normalize_yaw(pose.yaw_rad);
    return pose;
}

inline bool sparse_slam_frame_pose_valid(const OdomPose2D &pose) {
    return sparse_slam_pose_finite(pose.odom_T_base);
}

inline bool sparse_slam_frame_pose_valid(const MapPose2D &pose) {
    return sparse_slam_pose_finite(pose.map_T_base);
}

inline bool sparse_slam_frame_pose_valid(const MapFromOdom2D &transform) {
    return sparse_slam_pose_finite(transform.map_T_odom);
}

inline OdomPose2D make_odom_pose(RobotPose2D pose) {
    return OdomPose2D{sparse_slam_normalized_pose(pose)};
}

inline MapPose2D make_map_pose(RobotPose2D pose) {
    return MapPose2D{sparse_slam_normalized_pose(pose)};
}

inline MapFromOdom2D make_map_from_odom(RobotPose2D pose) {
    return MapFromOdom2D{sparse_slam_normalized_pose(pose)};
}

inline MapFromOdom2D identity_map_from_odom() {
    return MapFromOdom2D{RobotPose2D{}};
}

inline RobotPose2D compose_robot_poses(const RobotPose2D &a,
                                       const RobotPose2D &b) {
    RobotPose2D out;
    const double c = std::cos(a.yaw_rad);
    const double s = std::sin(a.yaw_rad);
    out.x_m = a.x_m + c * b.x_m - s * b.y_m;
    out.y_m = a.y_m + s * b.x_m + c * b.y_m;
    out.yaw_rad = sparse_slam_normalize_yaw(a.yaw_rad + b.yaw_rad);
    return out;
}

inline RobotPose2D inverse_robot_pose(const RobotPose2D &pose) {
    RobotPose2D out;
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    out.x_m = -c * pose.x_m - s * pose.y_m;
    out.y_m = s * pose.x_m - c * pose.y_m;
    out.yaw_rad = sparse_slam_normalize_yaw(-pose.yaw_rad);
    return out;
}

inline MapPose2D compose(const MapFromOdom2D &map_from_odom,
                         const OdomPose2D &odom_pose) {
    return make_map_pose(compose_robot_poses(map_from_odom.map_T_odom,
                                             odom_pose.odom_T_base));
}

inline MapFromOdom2D inverse(const MapFromOdom2D &map_from_odom) {
    return make_map_from_odom(inverse_robot_pose(map_from_odom.map_T_odom));
}

inline MapFromOdom2D between(const MapPose2D &map_pose,
                             const OdomPose2D &odom_pose) {
    return make_map_from_odom(compose_robot_poses(
        map_pose.map_T_base,
        inverse_robot_pose(odom_pose.odom_T_base)));
}

inline double sparse_slam_shortest_yaw_delta(double from_rad, double to_rad) {
    return sparse_slam_normalize_yaw(to_rad - from_rad);
}

} // namespace robot_slamd
