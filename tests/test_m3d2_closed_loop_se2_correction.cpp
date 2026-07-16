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
    drift.x_m = 0.08;
    drift.y_m = -0.04;
    drift.yaw_rad = 0.05;
    auto a = pipeline.process(d2pipe::input(3.0, drift));
    drift.x_m += 0.05;
    auto b = pipeline.process(d2pipe::input(3.1, drift));
    if (!a.ok || !b.ok || !b.correction_applied) return fail("SE2 drift correction did not apply");
    if (pipeline.report().matcher_accept_count == 0 || !pipeline.report().bounded_search_verified) {
        return fail("matcher acceptance or bounded search not reported");
    }
    if (std::fabs(pipeline.report().last_correction_dx_m) < 1e-9 && std::fabs(pipeline.report().last_correction_dyaw_rad) < 1e-9) {
        return fail("SE2 correction report did not record correction");
    }

    return 0;
}
