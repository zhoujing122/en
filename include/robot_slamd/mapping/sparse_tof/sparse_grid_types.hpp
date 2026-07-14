#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <cstdint>
#include <cstddef>
#include <algorithm>
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
};

enum class SparseReferenceCellClass {
    Unknown,
    Free,
    Occupied,
    Uncertain
};

struct SparseReferenceCellQuery {
    bool ok = false;
    bool found = false;
    SparseReferenceCellClass classification = SparseReferenceCellClass::Unknown;
    std::int16_t evidence = 0;
    std::string reason;
};

inline SparseGridCellKey sparse_grid_cell_for_world(double x_m,
                                                     double y_m,
                                                     double resolution_m) {
    SparseGridCellKey key;
    key.x = static_cast<std::int32_t>(std::floor(x_m / resolution_m));
    key.y = static_cast<std::int32_t>(std::floor(y_m / resolution_m));
    return key;
}

class SparseOccupancyGrid;

class SparseOccupancyGridSnapshot {
public:
    SparseOccupancyGridSnapshot() = default;

    bool valid() const { return valid_; }
    std::uint64_t revision() const { return revision_; }
    double resolution_m() const { return resolution_m_; }
    std::int16_t free_threshold() const { return free_threshold_; }
    std::int16_t occupied_threshold() const { return occupied_threshold_; }
    std::size_t cell_count() const { return cells_.size(); }
    std::size_t free_cell_count() const { return free_cell_count_; }
    std::size_t occupied_cell_count() const { return occupied_cell_count_; }
    std::size_t uncertain_cell_count() const { return uncertain_cell_count_; }
    bool has_bounds() const { return has_bounds_; }
    std::int32_t min_x() const { return min_x_; }
    std::int32_t max_x() const { return max_x_; }
    std::int32_t min_y() const { return min_y_; }
    std::int32_t max_y() const { return max_y_; }
    const std::vector<SparseOccupancyCell> &cells() const { return cells_; }

    SparseReferenceCellQuery query_cell(const SparseGridCellKey &key) const {
        if (!valid_) {
            return {false, false, SparseReferenceCellClass::Unknown, 0,
                    "reference_snapshot_invalid"};
        }
        const auto it = std::lower_bound(
            cells_.begin(), cells_.end(), key,
            [](const SparseOccupancyCell &cell, const SparseGridCellKey &candidate) {
                return cell.key < candidate;
            });
        if (it == cells_.end() || !(it->key == key)) {
            return {true, false, SparseReferenceCellClass::Unknown, 0,
                    "reference_cell_unknown"};
        }
        return {true, true, classify(it->evidence), it->evidence,
                "reference_cell_found"};
    }

    SparseReferenceCellQuery query_world(double x_m, double y_m) const {
        if (!valid_ || !std::isfinite(x_m) || !std::isfinite(y_m)) {
            return {false, false, SparseReferenceCellClass::Unknown, 0,
                    "reference_world_query_invalid"};
        }
        return query_cell(sparse_grid_cell_for_world(x_m, y_m, resolution_m_));
    }

private:
    friend class SparseOccupancyGrid;

    SparseReferenceCellClass classify(std::int16_t evidence) const {
        if (evidence <= free_threshold_) {
            return SparseReferenceCellClass::Free;
        }
        if (evidence >= occupied_threshold_) {
            return SparseReferenceCellClass::Occupied;
        }
        return SparseReferenceCellClass::Uncertain;
    }

    bool valid_ = false;
    std::uint64_t revision_ = 0;
    double resolution_m_ = 0.0;
    std::int16_t free_threshold_ = 0;
    std::int16_t occupied_threshold_ = 0;
    std::vector<SparseOccupancyCell> cells_;
    std::size_t free_cell_count_ = 0;
    std::size_t occupied_cell_count_ = 0;
    std::size_t uncertain_cell_count_ = 0;
    bool has_bounds_ = false;
    std::int32_t min_x_ = 0;
    std::int32_t max_x_ = 0;
    std::int32_t min_y_ = 0;
    std::int32_t max_y_ = 0;
};

enum class SparseOccupancyGridSnapshotFault {
    None,
    InvalidGrid,
    InvalidLimit,
    CapacityExceeded
};

struct SparseOccupancyGridSnapshotCaptureResult {
    bool ok = false;
    SparseOccupancyGridSnapshot snapshot;
    SparseOccupancyGridSnapshotFault fault =
        SparseOccupancyGridSnapshotFault::None;
    std::string message;
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
