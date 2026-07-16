#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    auto cfg = d2pipe::config();
    cfg.matcher.score.minimum_score_margin = 0.95;
    SparseTofLocalSlamPipeline pipeline(cfg);
    pipeline.reset_startup_zero();
    if (!d2pipe::bootstrap(pipeline)) return fail("bootstrap failed");
    RobotPose2D pose;
    pose.yaw_rad = 0.03;
    pipeline.process(d2pipe::input(5.0, pose));
    pose.x_m = 0.05;
    const auto out = pipeline.process(d2pipe::input(5.1, pose));
    if (out.ok || out.map_updated || pipeline.report().matcher_reject_count == 0) {
        return fail("ambiguous match should be rejected without map update");
    }
    return 0;
}
