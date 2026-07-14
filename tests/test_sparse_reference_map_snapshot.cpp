#include "sparse_reference_snapshot_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace snapshot_test;

    SparseOccupancyGrid empty(config());
    const auto empty_capture = empty.capture_snapshot(4, 64);
    expect(empty_capture.ok, "empty map capture succeeds");
    expect(empty_capture.snapshot.valid(), "empty snapshot valid");
    expect(empty_capture.snapshot.revision() == 4, "revision retained");
    expect(empty_capture.snapshot.resolution_m() == 0.5, "resolution retained");
    expect(empty_capture.snapshot.cell_count() == 0, "empty cell count");
    expect(!empty_capture.snapshot.has_bounds(), "empty map has no bounds");

    SparseOccupancyGrid grid(config());
    expect(grid.apply_observations({
        observation(ScalarTofReturnKind::Hit, 0.0, 1.0),
        observation(ScalarTofReturnKind::Hit, 1.5707963267948966, 1.0)})
        .accepted, "map update accepted");
    const auto captured = grid.capture_snapshot(9, 64);
    expect(captured.ok, "multi-cell capture succeeds");
    const auto &snapshot = captured.snapshot;
    expect(snapshot.revision() == 9, "multi-cell revision retained");
    expect(snapshot.cell_count() == grid.active_cell_count(), "cell count retained");
    expect(snapshot.occupied_cell_count() == 2, "occupied statistics");
    expect(snapshot.free_cell_count() + snapshot.uncertain_cell_count() > 0,
           "non-occupied statistics");
    expect(snapshot.has_bounds(), "bounds available");
    expect(snapshot.min_x() <= snapshot.max_x(), "x bounds ordered");
    expect(snapshot.min_y() <= snapshot.max_y(), "y bounds ordered");
    expect(snapshot.cell_count() ==
               snapshot.free_cell_count() +
               snapshot.occupied_cell_count() +
               snapshot.uncertain_cell_count(),
           "classification counts cover cells");
    return 0;
}
