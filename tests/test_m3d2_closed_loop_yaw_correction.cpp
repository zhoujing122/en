#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <cmath>
#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    SparseTofLocalSlamPipeline pipeline(d2pipe::config());
    pipeline.reset_startup_zero();
    if (!d2pipe::bootstrap(pipeline)) return fail("bootstrap failed");

    RobotPose2D drift;
    drift.yaw_rad = 0.08;
    auto a = pipeline.process(d2pipe::input(2.0, drift));
    drift.x_m = 0.05;
    auto b = pipeline.process(d2pipe::input(2.1, drift));
    if (!a.ok || !b.ok || !b.correction_applied) return fail("yaw drift correction did not apply");
    if (std::fabs(b.corrected_map_pose.yaw_rad) >= std::fabs(drift.yaw_rad)) {
        return fail("corrected yaw not closer to bootstrap frame than raw odom yaw");
    }
    if (pipeline.report().matcher_used_ground_truth || pipeline.report().command_derived_pose_used) {
        return fail("report indicates forbidden pose source");
    }

    return 0;
}
