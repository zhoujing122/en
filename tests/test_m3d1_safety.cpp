#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"
#include "robot_slamd/core/common.hpp"

#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    const Config config;
    expect(config.motion_execution_mode == "dry_run",
           "motion_execution_mode dry_run");
    expect(!config.motion_execution_hardware_write_enabled, "hardware writes disabled");
    expect(!config.motion_execution_allow_writer_dispatch, "writer dispatch disabled");
    expect(config.motion_execution_writer_backend == "null", "writer backend null");
    expect(!config.motion_execution_software_motion_production_interface_enabled, "production interface disabled");
    expect(!config.motion_execution_software_motion_allow_translation_commands, "translation disabled");
    expect(!config.motion_execution_algorithm_motion_allow_translation_commands, "forward backward disabled");
    expect(!config.real_adapter_stubs_allow_real_hardware_adapters, "real adapters disabled");
    expect(!config.real_adapter_stubs_enabled, "real adapter stubs disabled");
    expect(!config.yaw_correction_writeback_enabled, "yaw pose writeback disabled");
    expect(!config.fake_relocalization_allow_pose_writeback,
           "relocalization writeback disabled");

    SparseMultiTofOccupancyBackendBinding backend;
    const auto report = backend.report();
    expect(!report.real_hardware_accessed, "backend no real hardware");
    expect(!report.real_motion_attempted, "backend no real motion");
    expect(!report.real_map_write_attempted, "backend no real map write");
    expect(!report.real_pose_writeback_attempted, "backend no pose writeback");
    return failures ? 1 : 0;
}
