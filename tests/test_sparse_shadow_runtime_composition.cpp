#include "robot_slamd/runtime/sparse_shadow_runtime.hpp"

#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; }
}
}

int main() {
    using namespace robot_slamd;
    Config legacy;
    expect(resolved_slam_runtime_mode(legacy.slam_runtime_mode) == SlamRuntimeMode::Legacy,
           "legacy remains selected by default");

    Config cfg;
    cfg.slam_runtime_mode = "sparse_shadow";
    cfg.sparse_shadow_sensor_source = "deterministic_simulation";
    const auto report = run_sparse_shadow_runtime(cfg, 0.20, "/tmp/m3d11_composition");
    expect(report.ok, "sparse shadow runtime succeeds");
    expect(report.sensor_port_constructed, "sensor port constructed");
    expect(report.wheel_imu_estimator_constructed, "wheel imu estimator constructed");
    expect(report.sparse_backend_constructed, "sparse backend constructed");
    expect(!report.old_chunked_grid_constructed, "old chunked grid not constructed");
    expect(!report.old_matcher_constructed, "old matcher not constructed");
    expect(!report.old_correction_constructed, "old correction not constructed");
    return failures ? 1 : 0;
}
