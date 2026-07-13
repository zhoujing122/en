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
    Config cfg;
    cfg.slam_runtime_mode = "sparse_shadow";
    cfg.sparse_shadow_sensor_source = "deterministic_simulation";
    const auto report = run_sparse_shadow_runtime(cfg, 0.30, "/tmp/m3d11_success");
    expect(report.ok, "runtime succeeds");
    expect(report.sensor_snapshot_count >= 1, "snapshot consumed");
    expect(report.wheel_imu_update_attempt_count >= 1, "odometry update attempted");
    expect(report.wheel_imu_update_success_count >= 1, "odometry update succeeded");
    expect(report.predicted_pose_handoff_count >= 1, "predicted pose handed off");
    expect(report.backend_update_attempt_count >= 1, "backend update attempted");
    expect(report.backend_accept_count >= 1, "backend accepted update");
    expect(report.native_multi_tof_snapshot_count >= 1, "native multi tof snapshot seen");
    expect(report.native_multi_tof_consumed, "native multi tof consumed");
    expect(!report.legacy_projection_consumed, "legacy projection not consumed");
    expect(report.sparse_map_cell_count > 0, "sparse map cells created");
    return failures ? 1 : 0;
}
