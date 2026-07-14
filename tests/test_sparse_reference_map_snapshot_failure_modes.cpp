#include "sparse_reference_snapshot_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace snapshot_test;

    SparseOccupancyGrid grid(config());
    expect(grid.apply_observations({
        observation(ScalarTofReturnKind::Hit, 0.0, 1.0)})
        .accepted, "source map update accepted");

    const auto invalid_limit = grid.capture_snapshot(1, 0);
    expect(!invalid_limit.ok, "zero hard cap rejected");
    expect(!invalid_limit.snapshot.valid(), "limit failure has no partial snapshot");
    expect(invalid_limit.snapshot.cell_count() == 0, "limit failure empty");

    const auto capacity = grid.capture_snapshot(1, 1);
    expect(!capacity.ok, "capacity overflow rejected");
    expect(capacity.fault == SparseOccupancyGridSnapshotFault::CapacityExceeded,
           "capacity fault explicit");
    expect(!capacity.snapshot.valid(), "capacity failure has no partial snapshot");

    auto bad_config = config();
    bad_config.resolution_m = 0.0;
    SparseOccupancyGrid invalid_grid(bad_config);
    const auto invalid_grid_capture = invalid_grid.capture_snapshot(1, 64);
    expect(!invalid_grid_capture.ok, "invalid grid rejected");
    expect(!invalid_grid_capture.snapshot.valid(), "invalid grid no partial snapshot");

    const auto first = grid.capture_snapshot(11, 64);
    const auto second = grid.capture_snapshot(11, 64);
    expect(first.ok && second.ok, "repeat capture succeeds");
    expect(first.snapshot.cell_count() == second.snapshot.cell_count(),
           "repeat deterministic");
    return 0;
}
