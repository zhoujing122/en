#pragma once

#include <cmath>

namespace robot_slamd {

constexpr double kSimPi = 3.14159265358979323846;
constexpr double kSimTwoPi = 2.0 * kSimPi;

inline bool sim_finite(double value) {
    return std::isfinite(value);
}

inline double sim_clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

inline double sim_normalize_yaw(double yaw_rad) {
    if (!std::isfinite(yaw_rad)) {
        return 0.0;
    }
    while (yaw_rad > kSimPi) {
        yaw_rad -= kSimTwoPi;
    }
    while (yaw_rad <= -kSimPi) {
        yaw_rad += kSimTwoPi;
    }
    return yaw_rad;
}

inline double sim_angular_difference(double a_rad, double b_rad) {
    return sim_normalize_yaw(a_rad - b_rad);
}

inline double sim_step_toward(double current, double target, double max_delta) {
    if (current < target) {
        return current + sim_clamp(target - current, 0.0, max_delta);
    }
    return current - sim_clamp(current - target, 0.0, max_delta);
}

inline double sim_degrees_to_radians(double degrees) {
    return degrees * kSimPi / 180.0;
}

} // namespace robot_slamd
