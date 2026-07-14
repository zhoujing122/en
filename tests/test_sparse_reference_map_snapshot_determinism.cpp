#include "sparse_reference_snapshot_test_helpers.hpp"

int main() {
    using namespace robot_slamd;
    using namespace snapshot_test;

    const auto front = observation(ScalarTofReturnKind::Hit, 0.0, 1.0);
    const auto left = observation(
        ScalarTofReturnKind::NoReturn, 1.5707963267948966, 1.0);
    SparseOccupancyGrid a(config());
    SparseOccupancyGrid b(config());
    expect(a.apply_observations({front, left}).accepted, "order A accepted");
    expect(b.apply_observations({left, front}).accepted, "order B accepted");

    const auto sa = a.capture_snapshot(7, 64);
    const auto sb = b.capture_snapshot(7, 64);
    expect(sa.ok && sb.ok, "both captures succeed");
    expect(sa.snapshot.cell_count() == sb.snapshot.cell_count(),
           "deterministic cell count");
    for (std::size_t i = 0; i < sa.snapshot.cell_count(); ++i) {
        expect(sa.snapshot.cells()[i].key == sb.snapshot.cells()[i].key,
               "deterministic key order");
        expect(sa.snapshot.cells()[i].evidence == sb.snapshot.cells()[i].evidence,
               "deterministic evidence");
        if (i > 0) {
            expect(sa.snapshot.cells()[i - 1].key < sa.snapshot.cells()[i].key,
                   "strict stable y-x ordering");
        }
    }

    const auto frozen_count = sa.snapshot.cell_count();
    expect(a.apply_observations({
        observation(ScalarTofReturnKind::Hit, 3.141592653589793, 1.0)})
        .accepted, "later live update accepted");
    expect(sa.snapshot.cell_count() == frozen_count,
           "captured snapshot isolated from live map");
    expect(sa.snapshot.query_cell(a.cell_for_world(-1.0, 0.0)).classification ==
               SparseReferenceCellClass::Unknown,
           "later live cell absent from old snapshot");
    return 0;
}
