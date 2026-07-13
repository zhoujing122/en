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
    Config hardware;
    hardware.slam_runtime_mode = "sparse_shadow";
    hardware.sparse_shadow_sensor_source = "hardware";
    const auto report = run_sparse_shadow_runtime(hardware, 0.10, "/tmp/m3d11_failclosed");
    expect(!report.ok, "hardware source fail closed");
    expect(report.last_fault == "sparse_shadow_sensor_source_unsupported", "specific hardware fault");
    expect(!report.real_hardware_accessed, "hardware not accessed");
    expect(!report.real_motion_attempted, "motion not attempted");
    expect(report.sensor_snapshot_count == 0, "no snapshots from unsupported source");
    expect(report.predicted_pose_handoff_count == 0, "no pose handoff on unsupported source");
    return failures ? 1 : 0;
}
