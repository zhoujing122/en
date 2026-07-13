#include "robot_slamd/mapping/sparse_tof/sparse_grid_ray_traversal.hpp"

#include <cmath>
#include <iostream>
#include <set>

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
    config.maximum_ray_cells = 64;

    auto horizontal = sparse_grid_traverse_ray(0.0, 0.0, 0.0, 1.5, config);
    expect(horizontal.ok, "horizontal traversal ok");
    expect(!horizontal.cells.empty(), "horizontal produces cells");
    expect(horizontal.cells.front().x == 1 && horizontal.cells.front().y == 0,
           "origin cell skipped");
    expect(horizontal.cells.back().x == 3 && horizontal.cells.back().y == 0,
           "endpoint included");

    auto vertical = sparse_grid_traverse_ray(0.0, 0.0, 3.14159265358979323846 / 2.0,
                                             1.0, config);
    expect(vertical.ok, "vertical traversal ok");
    expect(vertical.cells.back().x == 0 && vertical.cells.back().y == 2,
           "vertical endpoint");

    auto negative = sparse_grid_traverse_ray(-0.1, -0.1,
                                             3.14159265358979323846,
                                             1.0, config);
    expect(negative.ok, "negative traversal ok");
    expect(negative.cells.back().x <= -3 && negative.cells.back().y == -1,
           "negative coordinate floor traversal");

    auto diagonal = sparse_grid_traverse_ray(0.0, 0.0,
                                             3.14159265358979323846 / 4.0,
                                             1.5, config);
    expect(diagonal.ok, "diagonal traversal ok");
    std::set<SparseGridCellKey> unique(diagonal.cells.begin(),
                                       diagonal.cells.end());
    expect(unique.size() == diagonal.cells.size(), "duplicate-free traversal");

    auto invalid = sparse_grid_traverse_ray(0.0, 0.0, 0.0, 0.01, config);
    expect(!invalid.ok, "reject below mapping min");
    config.maximum_ray_cells = 1;
    auto limited = sparse_grid_traverse_ray(0.0, 0.0, 0.0, 2.0, config);
    expect(!limited.ok, "ray traversal limit enforced");
    return failures ? 1 : 0;
}
