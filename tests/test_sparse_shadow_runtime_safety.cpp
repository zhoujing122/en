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
    expect(cfg.motion_execution_mode == "dry_run", "motion mode remains dry_run");
    expect(!cfg.motion_execution_hardware_write_enabled, "hardware write disabled");
    expect(!cfg.motion_execution_allow_writer_dispatch, "writer dispatch disabled");
    expect(cfg.motion_execution_writer_backend == "null", "writer backend null");
    expect(!cfg.motion_execution_software_motion_production_interface_enabled,
           "production interface disabled");
    expect(!cfg.autonomous_slam_allow_forward_backward, "forward backward disabled");
    expect(!cfg.real_adapter_stubs_allow_real_hardware_adapters, "real hardware adapters disabled");
    expect(!cfg.real_adapter_stubs_enabled, "real adapter stubs disabled");
    expect(!cfg.fake_relocalization_allow_pose_writeback, "pose writeback disabled");
    expect(!cfg.yaw_correction_writeback_enabled, "yaw correction writeback disabled");

    cfg.slam_runtime_mode = "sparse_shadow";
    cfg.sparse_shadow_sensor_source = "deterministic_simulation";
    const auto report = run_sparse_shadow_runtime(cfg, 0.10, "/tmp/m3d11_safety");
    expect(report.ok, "runtime succeeds for safety report");
    expect(!report.real_hardware_accessed, "runtime did not access hardware");
    expect(!report.real_motion_attempted, "runtime did not attempt motion");
    expect(!report.real_map_write_attempted, "runtime did not write map");
    expect(!report.real_pose_writeback_attempted, "runtime did not write pose");
    expect(report.initial_yaw_defined_by_startup_frame, "initial yaw defined by startup frame");
    expect(!report.initial_yaw_measured_by_imu, "initial yaw not measured by imu");
    expect(report.map_from_odom_assumed_identity, "identity frame assumption reported");
    return failures ? 1 : 0;
}
