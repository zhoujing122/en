#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    robot_slamd::SparseTofLocalSlamPipeline pipeline(d2pipe::config());
    pipeline.reset_startup_zero();
    const auto &report = pipeline.report();
    if (report.real_hardware_accessed || report.real_motion_attempted || report.real_map_write_attempted || report.real_pose_writeback_attempted) {
        return fail("local SLAM report enabled real hardware side effects");
    }
    if (report.legacy_projection_consumed || report.estimator_pose_writeback_used || report.initial_yaw_measured_by_imu) {
        return fail("safety report indicates forbidden localization behavior");
    }
    return 0;
}
