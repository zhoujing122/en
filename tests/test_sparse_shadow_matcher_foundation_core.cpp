#include "robot_slamd/runtime/sparse_shadow_runtime.hpp"

#include <iostream>

int main() {
    robot_slamd::Config cfg;
    cfg.sparse_slam_startup_min_imu_samples = 1;
    robot_slamd::SparseShadowRuntime runtime(cfg);
    const auto report = runtime.run(0.5, "");
    if (!report.runtime_core_constructed || report.matcher_attempt_count != 0 ||
        report.matcher_executed || report.matcher_evaluated_candidate_count != 0 ||
        report.reference_snapshot_capture_attempt_count != 0) {
        std::cerr << "shadow wrapper matcher foundation ownership failed\n";
        return 1;
    }
    return 0;
}
