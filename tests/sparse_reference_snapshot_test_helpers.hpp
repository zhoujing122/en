#pragma once

#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace snapshot_test {

inline void expect(bool condition, const std::string &message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

inline robot_slamd::SparseTofRayObservation observation(
    robot_slamd::ScalarTofReturnKind kind,
    double yaw_rad,
    double range_m) {
    robot_slamd::SparseTofRayObservation out;
    out.return_kind = kind;
    out.sensor_origin_x_m = 0.0;
    out.sensor_origin_y_m = 0.0;
    out.ray_yaw_rad = yaw_rad;
    out.measured_range_m = range_m;
    out.free_space_range_m = range_m;
    out.confidence = 100;
    out.protocol_valid = true;
    out.synchronized = true;
    out.usable_for_mapping = true;
    return out;
}

inline robot_slamd::SparseOccupancyGridConfig config() {
    robot_slamd::SparseOccupancyGridConfig out;
    out.resolution_m = 0.5;
    out.maximum_active_cells = 64;
    return out;
}

} // namespace snapshot_test
