
#include "robot_slamd/runtime/sparse_shadow_runtime.hpp"

#include <iostream>

namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }

int main() {
    using namespace robot_slamd;
    Config cfg;
    expect(resolved_slam_runtime_mode(cfg.slam_runtime_mode) == SlamRuntimeMode::Legacy, "default remains legacy");
    cfg.slam_runtime_mode = "sparse_shadow";
    cfg.sparse_shadow_sensor_source = "deterministic_simulation";
    const auto report = run_sparse_shadow_runtime(cfg, 0.1, "/tmp/m3d2a0_composition");
    expect(report.ok, "short sparse shadow run succeeds via min two steps");
    expect(report.runtime_core_constructed, "runtime core reachable");
    expect(report.map_update_before_initialization_count == 0, "no map before init");
    expect(!report.real_hardware_accessed, "no hardware");
    expect(!report.real_motion_attempted, "no motion");
    return failures ? 1 : 0;
}
