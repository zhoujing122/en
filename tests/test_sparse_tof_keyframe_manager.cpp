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
    cfg.minimum_rotation_rad = 0.01;
    SparseTofKeyframeManager manager(cfg);
    RobotPose2D pose;
    auto a = manager.add_observation(d2test::observation(1.0, pose), false);
    if (!a.ok || manager.ready_for_bootstrap(true, false)) return fail("first observation readiness wrong");
    pose.x_m = 0.02;
    auto b = manager.add_observation(d2test::observation(1.1, pose), true);
    if (!b.ok || !manager.ready_for_bootstrap(true, true)) return fail("motion settled trigger failed");
    auto dup = manager.add_observation(d2test::observation(1.1, pose), true);
    if (dup.ok || dup.fault != SparseTofKeyframeFault::DuplicateTimestamp) return fail("duplicate timestamp accepted");

    SparseTofKeyframeConfig small = cfg;
    small.maximum_snapshot_count = 2;
    SparseTofKeyframeManager full(small);
    full.add_observation(d2test::observation(2.0, RobotPose2D{}), true);
    full.add_observation(d2test::observation(2.1, pose), true);
    auto overflow = full.add_observation(d2test::observation(2.2, pose), true);
    if (overflow.ok || overflow.fault != SparseTofKeyframeFault::BundleFull) return fail("bundle max size not enforced");

    return 0;
}
