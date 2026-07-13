#pragma once

#include "robot_slamd/core/common.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_slamd {

inline double scalar_tof_full_fov_from_spot_width(
    double spot_width_m,
    double distance_m) {
    if (!std::isfinite(spot_width_m) || !std::isfinite(distance_m) ||
        spot_width_m <= 0.0 || distance_m <= 0.0) {
        throw std::invalid_argument("invalid_scalar_tof_spot_geometry");
    }
    const double fov_rad = 2.0 * std::atan(spot_width_m / (2.0 * distance_m));
    if (!std::isfinite(fov_rad) || fov_rad <= 0.0) {
        throw std::invalid_argument("invalid_scalar_tof_full_fov_result");
    }
    return fov_rad;
}

inline double scalar_tof_spot_width_from_full_fov(
    double full_fov_rad,
    double distance_m) {
    if (!std::isfinite(full_fov_rad) || !std::isfinite(distance_m) ||
        full_fov_rad <= 0.0 || distance_m <= 0.0) {
        throw std::invalid_argument("invalid_scalar_tof_fov_geometry");
    }
    const double width_m = 2.0 * distance_m * std::tan(full_fov_rad * 0.5);
    if (!std::isfinite(width_m) || width_m <= 0.0) {
        throw std::invalid_argument("invalid_scalar_tof_spot_width_result");
    }
    return width_m;
}

inline double scalar_tof_default_full_fov_rad() {
    return deg2rad(1.6);
}

} // namespace robot_slamd
