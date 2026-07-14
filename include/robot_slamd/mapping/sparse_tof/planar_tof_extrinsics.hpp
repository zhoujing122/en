#pragma once

#include "robot_slamd/runtime/sparse_slam_frames.hpp"

#include <cmath>

namespace robot_slamd {

enum class PlanarTofRoute { Front, Left, Right };

struct PlanarTofExtrinsic {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

struct PlanarThreeTofExtrinsics {
    PlanarTofExtrinsic front;
    PlanarTofExtrinsic left;
    PlanarTofExtrinsic right;
};

inline bool planar_tof_extrinsic_valid(const PlanarTofExtrinsic &extrinsic) {
    return std::isfinite(extrinsic.x_m) && std::isfinite(extrinsic.y_m) &&
           std::isfinite(extrinsic.yaw_rad);
}

inline const PlanarTofExtrinsic &planar_tof_extrinsic_for(
    const PlanarThreeTofExtrinsics &extrinsics, PlanarTofRoute route) {
    if (route == PlanarTofRoute::Front) return extrinsics.front;
    if (route == PlanarTofRoute::Left) return extrinsics.left;
    return extrinsics.right;
}

inline RobotPose2D compose_base_with_tof_extrinsic(
    const RobotPose2D &map_T_base,
    const PlanarTofExtrinsic &base_T_sensor) {
    return compose_robot_poses(
        map_T_base,
        RobotPose2D{base_T_sensor.x_m, base_T_sensor.y_m,
                    base_T_sensor.yaw_rad});
}

} // namespace robot_slamd
