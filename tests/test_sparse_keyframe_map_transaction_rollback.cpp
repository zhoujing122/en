#include "sparse_tof_yaw_match_test_helpers.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_keyframe_transaction.hpp"

int main() {
    using namespace robot_slamd;
    using namespace yaw_match_test;
    const auto frames = asymmetric_frames();
    const auto bundle = frozen_bundle(frames, frames.size() * 3);
    SparseOccupancyGrid grid;
    SparseTofKeyframeMapTransactionBuilder builder;

    const auto before = grid.snapshot();
    const auto rejected = builder.prepare(
        bundle, make_map_from_odom({0.0, 0.0, 0.1}),
        11, 11, grid, 1);
    expect(!rejected.ok, "changed-cell limit rejects prepare");
    const auto after = grid.snapshot();
    expect(before.cell_count() == after.cell_count(),
           "failed prepare rolls back all map state");

    const auto revision_rejected = builder.prepare(
        bundle, identity_map_from_odom(), 11, 12, grid, 4096);
    expect(!revision_rejected.ok, "revision mismatch rejects prepare");
    expect(grid.active_cell_count() == 0,
           "revision failure leaves map untouched");
    return 0;
}
