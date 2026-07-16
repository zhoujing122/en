#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    auto cfg = d2pipe::config();
    cfg.matcher.score.minimum_observability_contrast = 2.0;
    SparseTofLocalSlamPipeline pipeline(cfg);
    pipeline.reset_startup_zero();
    if (!d2pipe::bootstrap(pipeline)) return fail("bootstrap failed");
    RobotPose2D drift;
    drift.yaw_rad = 0.04;
    pipeline.process(d2pipe::input(4.0, drift));
    drift.x_m = 0.05;
    const auto out = pipeline.process(d2pipe::input(4.1, drift));
    if (out.ok || out.correction_applied || out.map_updated) {
        return fail("degenerate match should not update map or correction");
    }
    if (pipeline.report().matcher_reject_count == 0) return fail("degenerate rejection not reported");
    return 0;
}
