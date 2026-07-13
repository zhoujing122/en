#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    SparseOccupancyGridConfig config;
    config.resolution_m = 0.5;
    SparseOccupancyGrid grid(config);
    expect(grid.valid(), "default sparse grid config valid");

    const auto zero = grid.cell_for_world(0.0, 0.0);
    expect(zero.x == 0 && zero.y == 0, "zero index");
    const auto positive = grid.cell_for_world(0.99, 1.01);
    expect(positive.x == 1 && positive.y == 2, "positive floor index");
    const auto negative = grid.cell_for_world(-0.01, -0.51);
    expect(negative.x == -1 && negative.y == -2,
           "negative coordinates use floor not truncation");

    config.resolution_m = 0.0;
    expect(!SparseOccupancyGrid(config).valid(), "reject zero resolution");
    config.resolution_m = 0.5;
    config.maximum_active_cells = 0;
    expect(!SparseOccupancyGrid(config).valid(), "reject zero capacity");
    return failures ? 1 : 0;
}
