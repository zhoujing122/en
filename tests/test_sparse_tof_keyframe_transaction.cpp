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

    auto tiny_cfg = d2test::grid_config();
    tiny_cfg.maximum_active_cells = 1;
    SparseOccupancyGrid tiny(tiny_cfg);
    auto failed = manager.commit_bootstrap(tiny, RobotPose2D{});
    if (failed.ok || failed.fault != SparseTofKeyframeFault::MapUpdateRejected) {
        return fail("capacity failure should reject keyframe transaction");
    }
    if (tiny.active_cell_count() != 0 || manager.active_snapshot_count() != 2 || manager.committed_keyframe_count() != 0) {
        return fail("failed map transaction changed map or cleared active bundle");
    }

    SparseOccupancyGrid ok(d2test::grid_config());
    auto committed = manager.commit_bootstrap(ok, RobotPose2D{});
    if (!committed.ok || ok.active_cell_count() == 0 || manager.active_snapshot_count() != 0) {
        return fail("retry after failed transaction did not commit cleanly");
    }

    return 0;
}
