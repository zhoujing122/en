#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    SparseTofLocalSlamPipeline pipeline(d2pipe::config());
    if (!pipeline.reset_startup_zero().ok) return fail("startup reset failed");

    RobotPose2D pose;
    auto moving = pipeline.process(d2pipe::input(0.5, pose, false, true));
    if (!moving.ok || pipeline.grid().active_cell_count() != 0 || !pipeline.report().phase_aware_publish_verified) {
        return fail("motion phase should suppress mapping while staying ok");
    }

    if (!d2pipe::bootstrap(pipeline)) return fail("bootstrap failed");
    if (!pipeline.report().self_matching_prevented || pipeline.report().bootstrap_keyframe_count != 1) {
        return fail("bootstrap report missing self-match prevention");
    }

    SparseTofLocalSlamInput missing;
    missing.has_odom_pose = false;
    auto rejected = pipeline.process(missing);
    if (rejected.ok || rejected.fault != SparseTofLocalSlamFault::OdomPoseMissing) {
        return fail("missing odom pose accepted");
    }

    return 0;
}
