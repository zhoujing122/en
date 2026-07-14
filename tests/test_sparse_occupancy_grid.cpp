#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::SparseTofRayObservation hit(double yaw, double range) {
    robot_slamd::SparseTofRayObservation obs;
    obs.return_kind = robot_slamd::ScalarTofReturnKind::Hit;
    obs.sensor_origin_x_m = 0.0;
    obs.sensor_origin_y_m = 0.0;
    obs.ray_yaw_rad = yaw;
    obs.measured_range_m = range;
    obs.free_space_range_m = range;
    obs.confidence = 100;
    obs.protocol_valid = true;
    obs.synchronized = true;
    obs.usable_for_mapping = true;
    return obs;
}

robot_slamd::SparseTofRayObservation no_return(double yaw, double range) {
    auto obs = hit(yaw, range);
    obs.return_kind = robot_slamd::ScalarTofReturnKind::NoReturn;
    obs.measured_range_m = range;
    obs.free_space_range_m = range;
    return obs;
}
}

int main() {
    using namespace robot_slamd;
    SparseOccupancyGridConfig config;
    config.resolution_m = 0.5;
    SparseOccupancyGrid grid(config);

    auto stats = grid.apply_observations({hit(0.0, 1.0)});
    expect(stats.accepted, "front hit accepted");
    auto free_cell = grid.cell_for_world(0.5, 0.0);
    auto occupied_cell = grid.cell_for_world(1.0, 0.0);
    expect(grid.evidence(free_cell) < 0, "hit path free");
    expect(grid.evidence(occupied_cell) > 0, "hit endpoint occupied");
    expect(!grid.contains(grid.cell_for_world(0.0, 0.0)),
           "origin cell not updated");

    SparseOccupancyGrid no_return_grid(config);
    auto nr = no_return_grid.apply_observations({no_return(0.0, 1.0)});
    expect(nr.accepted, "no return accepted");
    expect(no_return_grid.evidence(free_cell) < 0, "no return free path");
    expect(no_return_grid.evidence(occupied_cell) < 0,
           "no return endpoint is not occupied");

    SparseOccupancyGrid invalid_grid(config);
    auto invalid_obs = hit(0.0, 1.0);
    invalid_obs.return_kind = ScalarTofReturnKind::Invalid;
    invalid_obs.usable_for_mapping = false;
    auto inv = invalid_grid.apply_observations({invalid_obs});
    expect(!inv.accepted, "invalid-only snapshot rejected");
    expect(invalid_grid.snapshot().cells().empty(), "invalid does not update map");

    SparseOccupancyGrid overlap_a(config);
    SparseOccupancyGrid overlap_b(config);
    const auto a1 = hit(0.0, 1.0);
    const auto a2 = no_return(0.0, 1.0);
    expect(overlap_a.apply_observations({a1, a2}).accepted,
           "overlap order A accepted");
    expect(overlap_b.apply_observations({a2, a1}).accepted,
           "overlap order B accepted");
    expect(overlap_a.snapshot().cells().size() == overlap_b.snapshot().cells().size(),
           "overlap cell count order independent");
    expect(overlap_a.evidence(occupied_cell) == overlap_b.evidence(occupied_cell),
           "occupied over free order independent");

    SparseOccupancyGridConfig tiny = config;
    tiny.maximum_active_cells = 1;
    SparseOccupancyGrid tiny_grid(tiny);
    auto cap = tiny_grid.apply_observations({hit(0.0, 2.0)});
    expect(!cap.accepted &&
               cap.fault == SparseOccupancyGridFault::MapCapacityReached,
           "capacity failure rejected");
    expect(tiny_grid.snapshot().cells().empty(), "capacity failure transactional");

    return failures ? 1 : 0;
}
