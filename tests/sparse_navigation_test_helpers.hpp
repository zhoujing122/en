#pragma once

#include "robot_slamd/exploration/sparse_frontier_planner.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

namespace sparse_navigation_test {

inline void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << "\n";
        std::exit(1);
    }
}

inline robot_slamd::SparseOccupancyGridSnapshot snapshot_from_cells(
    const std::vector<robot_slamd::SparseOccupancyCell> &cells,
    double resolution = 1.0) {
    robot_slamd::SparseOccupancyGridConfig config;
    config.resolution_m = resolution;
    config.free_threshold = -1;
    config.occupied_threshold = 1;
    config.maximum_active_cells = 1000;
    robot_slamd::SparseOccupancyGrid grid(config);
    expect(grid.replace_cells_transactional(cells), "install cells");
    const auto captured = grid.capture_snapshot(7, 1000);
    expect(captured.ok, "capture cells");
    return captured.snapshot;
}

inline robot_slamd::SparseOccupancyCell free_cell(int x, int y) {
    return {{x, y}, -5};
}

inline robot_slamd::SparseOccupancyCell occupied_cell(int x, int y) {
    return {{x, y}, 5};
}

} // namespace sparse_navigation_test
