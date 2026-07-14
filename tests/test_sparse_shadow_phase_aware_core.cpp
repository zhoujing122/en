#include "robot_slamd/runtime/sparse_shadow_runtime.hpp"

#include <iostream>

int main() {
    robot_slamd::Config cfg;
    cfg.sparse_shadow_sensor_source = "deterministic_simulation";
    cfg.sparse_slam_pose_interpolation_max_gap_s = 1.0;
    const auto report = robot_slamd::run_sparse_shadow_runtime(cfg, 0.4, "");
    int failures = 0;
    auto expect = [&](bool condition, const char *message) {
        if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; }
    };
    expect(report.runtime_core_constructed, "shadow constructs runtime core");
    expect(report.active_bundle_state == "idle", "shadow wrapper does not own active bundle");
    expect(report.bundle_begin_count == 0, "shadow did not begin bundle by itself");
    expect(report.matcher_attempt_count == 0, "shadow matcher attempts zero");
    expect(report.keyframe_attempt_count == 0, "shadow keyframe attempts zero");
    expect(report.pose_correction_attempt_count == 0, "shadow correction attempts zero");
    expect(!report.real_hardware_accessed, "shadow no hardware");
    expect(!report.real_motion_attempted, "shadow no motion");
    return failures ? 1 : 0;
}
