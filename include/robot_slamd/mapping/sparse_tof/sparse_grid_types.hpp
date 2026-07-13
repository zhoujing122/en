#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <string>
#include <vector>

namespace robot_slamd {

struct SparseGridCellKey {
    std::int32_t x = 0;
    std::int32_t y = 0;
};

inline bool operator<(const SparseGridCellKey &a,
                      const SparseGridCellKey &b) {
    if (a.y != b.y) {
        return a.y < b.y;
    }
    return a.x < b.x;
}

inline bool operator==(const SparseGridCellKey &a,
                       const SparseGridCellKey &b) {
    return a.x == b.x && a.y == b.y;
}

struct SparseOccupancyGridConfig {
    double resolution_m = 0.05;
    std::int16_t free_delta = -3;
    std::int16_t occupied_delta = 6;
    std::int16_t minimum_evidence = -30;
    std::int16_t maximum_evidence = 30;
    std::int16_t free_threshold = -6;
    std::int16_t occupied_threshold = 6;
    std::size_t maximum_active_cells = 100000;
    double minimum_mapping_range_m = 0.05;
    double maximum_mapping_range_m = 12.0;
    double no_return_free_space_range_m = 12.0;
    std::size_t maximum_ray_cells = 512;
};

enum class SparseOccupancyGridFault {
    None,
    InvalidConfig,
    InvalidPose,
    InvalidObservation,
    NoUsableRay,
    RayTraversalFailed,
    MapCapacityReached,
    EvidenceOverflow
};

inline std::string to_string(SparseOccupancyGridFault fault) {
    switch (fault) {
    case SparseOccupancyGridFault::None:
        return "none";
    case SparseOccupancyGridFault::InvalidConfig:
        return "invalid_config";
    case SparseOccupancyGridFault::InvalidPose:
        return "invalid_pose";
    case SparseOccupancyGridFault::InvalidObservation:
        return "invalid_observation";
    case SparseOccupancyGridFault::NoUsableRay:
        return "no_usable_ray";
    case SparseOccupancyGridFault::RayTraversalFailed:
        return "ray_traversal_failed";
    case SparseOccupancyGridFault::MapCapacityReached:
        return "map_capacity_reached";
    case SparseOccupancyGridFault::EvidenceOverflow:
        return "evidence_overflow";
    }
    return "unknown";
}

struct SparseOccupancyCellState {
    std::int16_t evidence = 0;
};

struct SparseOccupancyCell {
    SparseGridCellKey key;
    std::int16_t evidence = 0;
    bool free = false;
    bool occupied = false;
};

struct SparseOccupancyGridSnapshot {
    double resolution_m = 0.0;
    std::vector<SparseOccupancyCell> cells;
    std::size_t free_cell_count = 0;
    std::size_t occupied_cell_count = 0;
    std::size_t uncertain_cell_count = 0;
    std::int32_t min_x = 0;
    std::int32_t max_x = 0;
    std::int32_t min_y = 0;
    std::int32_t max_y = 0;
};

struct SparseOccupancyUpdateStats {
    bool accepted = false;
    SparseOccupancyGridFault fault = SparseOccupancyGridFault::None;
    std::string message;
    int valid_ray_count = 0;
    int hit_ray_count = 0;
    int no_return_ray_count = 0;
    int invalid_ray_count = 0;
    int free_cell_update_count = 0;
    int occupied_cell_update_count = 0;
    int duplicate_cell_suppression_count = 0;
    int occupied_over_free_resolution_count = 0;
    int angular_bin = -1;
};

inline bool sparse_pose_finite(const RobotPose2D &pose) {
    return std::isfinite(pose.x_m) &&
           std::isfinite(pose.y_m) &&
           std::isfinite(pose.yaw_rad);
}

} // namespace robot_slamd
