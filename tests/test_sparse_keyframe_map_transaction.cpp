#include "sparse_tof_yaw_match_test_helpers.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_keyframe_transaction.hpp"

int main() {
    using namespace robot_slamd;
    using namespace yaw_match_test;
    const auto frames = asymmetric_frames(true);
    const auto bundle = frozen_bundle(frames, frames.size() * 3);
    SparseOccupancyGrid grid;
    SparseTofKeyframeMapTransactionBuilder builder;
    const auto proposed = make_map_from_odom({0.0, 0.0, 0.20});

    auto prepared = builder.prepare(bundle, proposed, 11, 11, grid, 4096);
    expect(prepared.ok && prepared.transaction.ready,
           "bundle map transaction prepares");
    expect(grid.active_cell_count() == 0,
           "prepare does not mutate live map");
    expect(prepared.transaction.stats.batch_count == frames.size(),
           "one evidence batch per frozen frame");
    expect(prepared.transaction.stats.hit_ray_count > 0 &&
               prepared.transaction.stats.no_return_ray_count > 0,
           "hit and no-return semantics retained");

    const auto corrected_endpoint = grid.cell_for_world(
        std::cos(0.20) * frames.front().front.frame.distance_m,
        std::sin(0.20) * frames.front().front.frame.distance_m);
    const auto old_endpoint = grid.cell_for_world(
        frames.front().front.frame.distance_m, 0.0);
    expect(grid.commit_prepared_batch(
               std::move(prepared.transaction.map_batch)),
           "prepared batch commits");
    expect(grid.evidence(corrected_endpoint) > 0,
           "corrected map pose determines occupied endpoint");
    expect(!(corrected_endpoint == old_endpoint) &&
               grid.evidence(old_endpoint) <= 0,
           "stale bundle map pose is not used for endpoint");
    return 0;
}
