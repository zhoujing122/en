#include "sparse_navigation_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace sparse_navigation_test;
    std::vector<SparseOccupancyCell> cells = {
        free_cell(0, 0), free_cell(1, 0), occupied_cell(2, 0),
        free_cell(3, 0), {{4, 0}, 0}};
    NavigationGridViewConfig config;
    config.min_x_m = 0.0;
    config.max_x_m = 6.0;
    config.min_y_m = 0.0;
    config.max_y_m = 3.0;
    config.robot_radius_m = 1.0;
    config.safety_margin_m = 0.0;
    NavigationGridView grid;
    expect(grid.build(snapshot_from_cells(cells), config).ok,
           "navigation view builds");
    expect(grid.cell({0, 0}) == NavigationCellClass::Free,
           "free remains traversable");
    expect(grid.cell({2, 0}) == NavigationCellClass::Occupied,
           "occupied retained");
    expect(grid.cell({1, 0}) == NavigationCellClass::InflatedOccupied &&
           grid.cell({3, 0}) == NavigationCellClass::InflatedOccupied,
           "footprint inflation blocks neighbors");
    expect(grid.cell({4, 0}) == NavigationCellClass::Uncertain,
           "uncertain remains blocked");
    expect(grid.cell({5, 0}) == NavigationCellClass::Unknown,
           "missing snapshot cell is unknown");
    config.maximum_cells = 4;
    expect(!grid.build(snapshot_from_cells(cells), config).ok,
           "planning grid hard cap enforced");
    return 0;
}
