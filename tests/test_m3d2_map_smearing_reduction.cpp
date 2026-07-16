#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <cmath>
#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    SparseTofLocalSlamPipeline corrected(d2pipe::config());
    corrected.reset_startup_zero();
    if (!d2pipe::bootstrap(corrected)) return fail("bootstrap failed");

    RobotPose2D drift;
    drift.yaw_rad = 0.08;
    corrected.process(d2pipe::input(6.0, drift));
    drift.x_m = 0.05;
    const auto out = corrected.process(d2pipe::input(6.1, drift));
    if (!out.correction_applied) return fail("correction missing in smearing scenario");
    const double raw_error = std::fabs(drift.yaw_rad);
    const double corrected_error = std::fabs(out.corrected_map_pose.yaw_rad);
    if (!(corrected_error < raw_error)) return fail("corrected mapping did not reduce yaw dispersion proxy");
    if (!corrected.report().batch_map_transaction_verified) return fail("batch transaction not verified");
    return 0;
}
