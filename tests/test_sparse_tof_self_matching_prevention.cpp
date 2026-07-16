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

    SparseOccupancyGrid reference(d2test::grid_config());
    const auto frozen = reference.snapshot();
    SparseTofLocalScanMatcher matcher(d2test::matcher_config());
    const auto match = matcher.match(frozen, manager.active_bundle(), RobotPose2D{});
    if (match.accepted || match.fault != SparseTofMatcherFault::ReferenceMapUnavailable) {
        return fail("empty frozen reference should prevent self matching");
    }
    if (reference.active_cell_count() != 0) {
        return fail("matcher changed reference map");
    }

    auto boot = manager.commit_bootstrap(reference, RobotPose2D{});
    if (!boot.ok || reference.active_cell_count() == 0) {
        return fail("bootstrap should commit only after explicit map transaction");
    }

    return 0;
}
