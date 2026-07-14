#pragma once

#include "robot_slamd/mapping/sparse_tof/sparse_grid_types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace robot_slamd {

struct SparseGridRayTraversalResult {
    bool ok = false;
    SparseOccupancyGridFault fault = SparseOccupancyGridFault::None;
    std::vector<SparseGridCellKey> cells;
    std::string message;
};

inline bool sparse_grid_config_valid(
    const SparseOccupancyGridConfig &config) {
    return std::isfinite(config.resolution_m) &&
           config.resolution_m > 0.0 &&
           std::isfinite(config.minimum_mapping_range_m) &&
           std::isfinite(config.maximum_mapping_range_m) &&
           config.minimum_mapping_range_m > 0.0 &&
           config.maximum_mapping_range_m > config.minimum_mapping_range_m &&
           config.minimum_evidence <= config.free_threshold &&
           config.free_threshold < 0 &&
           config.occupied_threshold > 0 &&
           config.occupied_threshold <= config.maximum_evidence &&
           config.minimum_evidence < config.maximum_evidence &&
           config.maximum_active_cells > 0 &&
           config.maximum_ray_cells > 0;
}

inline bool sparse_key_present(const std::vector<SparseGridCellKey> &cells,
                               const SparseGridCellKey &key) {
    return std::find(cells.begin(), cells.end(), key) != cells.end();
}

inline SparseGridRayTraversalResult sparse_grid_traverse_ray(
    double origin_x_m,
    double origin_y_m,
    double ray_yaw_rad,
    double range_m,
    const SparseOccupancyGridConfig &config,
    bool skip_origin_cell = true) {
    SparseGridRayTraversalResult result;
    if (!sparse_grid_config_valid(config) ||
        !std::isfinite(origin_x_m) ||
        !std::isfinite(origin_y_m) ||
        !std::isfinite(ray_yaw_rad) ||
        !std::isfinite(range_m)) {
        result.fault = SparseOccupancyGridFault::InvalidConfig;
        result.message = "sparse_grid_ray_invalid_input";
        return result;
    }
    if (range_m < config.minimum_mapping_range_m ||
        range_m > config.maximum_mapping_range_m) {
        result.fault = SparseOccupancyGridFault::InvalidObservation;
        result.message = "sparse_grid_ray_range_out_of_bounds";
        return result;
    }

    const double end_x = origin_x_m + std::cos(ray_yaw_rad) * range_m;
    const double end_y = origin_y_m + std::sin(ray_yaw_rad) * range_m;
    if (!std::isfinite(end_x) || !std::isfinite(end_y)) {
        result.fault = SparseOccupancyGridFault::InvalidObservation;
        result.message = "sparse_grid_ray_endpoint_invalid";
        return result;
    }

    const auto origin_key =
        sparse_grid_cell_for_world(origin_x_m, origin_y_m, config.resolution_m);
    const auto end_key =
        sparse_grid_cell_for_world(end_x, end_y, config.resolution_m);

    const double samples_per_cell = 2.0;
    const auto sample_count = static_cast<std::size_t>(
        std::ceil((range_m / config.resolution_m) * samples_per_cell));
    const std::size_t bounded_samples =
        std::max<std::size_t>(sample_count, 1U);
    if (bounded_samples > config.maximum_ray_cells * 2U + 2U) {
        result.fault = SparseOccupancyGridFault::RayTraversalFailed;
        result.message = "sparse_grid_ray_traversal_limit";
        return result;
    }

    for (std::size_t i = 0; i <= bounded_samples; ++i) {
        const double t = static_cast<double>(i) /
                         static_cast<double>(bounded_samples);
        const double x = origin_x_m + (end_x - origin_x_m) * t;
        const double y = origin_y_m + (end_y - origin_y_m) * t;
        const auto key = sparse_grid_cell_for_world(x, y, config.resolution_m);
        if (skip_origin_cell && key == origin_key) {
            continue;
        }
        if (!sparse_key_present(result.cells, key)) {
            result.cells.push_back(key);
            if (result.cells.size() > config.maximum_ray_cells) {
                result.cells.clear();
                result.fault = SparseOccupancyGridFault::RayTraversalFailed;
                result.message = "sparse_grid_ray_cell_limit";
                return result;
            }
        }
    }

    if (!sparse_key_present(result.cells, end_key) &&
        !(skip_origin_cell && end_key == origin_key)) {
        result.cells.push_back(end_key);
    }
    result.ok = true;
    result.fault = SparseOccupancyGridFault::None;
    result.message = "sparse_grid_ray_ok";
    return result;
}

} // namespace robot_slamd
