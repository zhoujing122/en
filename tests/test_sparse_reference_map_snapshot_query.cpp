#include "sparse_reference_snapshot_test_helpers.hpp"

#include <limits>
#include <type_traits>

int main() {
    using namespace robot_slamd;
    using namespace snapshot_test;

    SparseOccupancyGrid grid(config());
    expect(grid.apply_observations({
        observation(ScalarTofReturnKind::Hit, 0.0, 1.0)})
        .accepted, "hit update accepted");
    const auto captured = grid.capture_snapshot(3, 64);
    expect(captured.ok, "capture succeeds");
    const auto &snapshot = captured.snapshot;

    const auto occupied_key = grid.cell_for_world(1.0, 0.0);
    const auto occupied = snapshot.query_cell(occupied_key);
    expect(occupied.ok && occupied.found, "existing cell found");
    expect(occupied.classification == SparseReferenceCellClass::Occupied,
           "occupied classification");
    expect(occupied.evidence == grid.evidence(occupied_key), "raw evidence retained");

    const SparseGridCellKey missing{50, 50};
    const auto unknown = snapshot.query_cell(missing);
    expect(unknown.ok && !unknown.found, "missing cell not found");
    expect(unknown.classification == SparseReferenceCellClass::Unknown,
           "missing cell is unknown");
    expect(snapshot.cell_count() == captured.snapshot.cell_count(),
           "query does not modify snapshot");

    const auto world = snapshot.query_world(1.0, 0.0);
    expect(world.ok && world.found, "world query reuses grid indexing");
    const auto invalid = snapshot.query_world(
        std::numeric_limits<double>::quiet_NaN(), 0.0);
    expect(!invalid.ok, "non-finite world coordinate rejected");

    static_assert(std::is_same<
        decltype(snapshot.cells()),
        const std::vector<SparseOccupancyCell> &>::value,
        "snapshot cells must be const-only");
    return 0;
}
