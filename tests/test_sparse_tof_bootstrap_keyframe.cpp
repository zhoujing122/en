#include "robot_slamd/mapping/sparse_tof/sparse_tof_keyframe_manager.hpp"
#include "sparse_tof_d2_test_helpers.hpp"

#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    SparseTofKeyframeConfig cfg;
    cfg.minimum_snapshot_count = 2;
    cfg.minimum_valid_ray_count = 3;
    cfg.minimum_hit_ray_count = 1;
    cfg.minimum_translation_m = 0.01;
    SparseTofKeyframeManager manager(cfg);
    RobotPose2D pose;
    manager.add_observation(d2test::observation(1.0, pose), true);
    pose.x_m = 0.05;
    manager.add_observation(d2test::observation(1.1, pose), true);

    SparseOccupancyGrid grid(d2test::grid_config());
    const auto committed = manager.commit_bootstrap(grid, RobotPose2D{});
    if (!committed.ok || !committed.metadata.bootstrap || !committed.metadata.map_committed) {
        return fail("bootstrap keyframe did not commit");
    }
    if (grid.active_cell_count() == 0 || manager.active_snapshot_count() != 0 || manager.committed_keyframe_count() != 1) {
        return fail("bootstrap did not update map or clear active bundle");
    }

    SparseTofKeyframeManager not_ready(cfg);
    SparseOccupancyGrid empty(d2test::grid_config());
    auto rejected = not_ready.commit_bootstrap(empty, RobotPose2D{});
    if (rejected.ok || rejected.fault != SparseTofKeyframeFault::BootstrapRejected) {
        return fail("empty bundle bootstrap accepted");
    }

    return 0;
}
