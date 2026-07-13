
#include "robot_slamd/runtime/sparse_shadow_runtime.hpp"

#include <iostream>

namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }

int main() {
    using namespace robot_slamd;
    Config cfg;
    cfg.slam_runtime_mode = "sparse_shadow";
    cfg.sparse_shadow_sensor_source = "deterministic_simulation";
    const auto report = run_sparse_shadow_runtime(cfg, 0.2, "/tmp/m3d2a0_shadow_core");
    expect(report.ok, "shadow wrapper succeeds through core");
    expect(report.runtime_core_constructed, "core constructed");
    expect(report.initialization_complete, "core initialized");
    expect(report.measurement_time_pose_consumed, "measurement poses consumed");
    expect(!report.single_pose_fallback_consumed, "single pose fallback not consumed");
    expect(report.matcher_attempt_count == 0, "matcher not attempted");
    expect(report.keyframe_attempt_count == 0, "keyframe not attempted");
    expect(!report.old_chunked_grid_constructed, "old grid not constructed");
    expect(!report.old_matcher_constructed, "old matcher not constructed");
    expect(!report.old_correction_constructed, "old correction not constructed");
    return failures ? 1 : 0;
}
