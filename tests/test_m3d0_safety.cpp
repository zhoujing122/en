#include "robot_slamd/core/common.hpp"
#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"

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
    auto config = Config{};
    expect(config.motion_execution_mode == "dry_run", "motion mode dry_run");
    expect(!config.motion_execution_hardware_write_enabled, "hardware write false");
    expect(!config.motion_execution_allow_writer_dispatch, "writer dispatch false");
    expect(config.motion_execution_writer_backend == "null", "writer backend null");
    expect(!config.motion_execution_software_motion_production_interface_enabled,
           "production interface false");
    expect(!config.motion_execution_software_motion_allow_translation_commands,
           "software translation false");
    expect(!config.motion_execution_algorithm_motion_allow_translation_commands,
           "algorithm translation false");
    expect(!config.autonomous_slam_allow_forward_backward, "forward backward false");
    expect(!config.real_adapter_stubs_allow_real_hardware_adapters, "real adapters false");
    expect(!config.fake_relocalization_allow_pose_writeback, "pose writeback false");

    RobotPose2D pose;
    auto plant = std::make_shared<SimRobotPlant>();
    plant->reset(pose, 0.0);
    SimMotionPortConfig motion_config;
    motion_config.allow_translation = false;
    SimMotionPort port(plant, motion_config);
    port.update(0.0);
    AlgorithmMotionCommandBuilder builder;
    auto forward = port.send_algorithm_command(builder.manual_forward(0.0));
    expect(!forward.accepted, "sim forward gated off by default");

    motion_config.allow_translation = true;
    SimMotionPort sim_only(plant, motion_config);
    sim_only.update(0.0);
    auto sim_forward = sim_only.send_algorithm_command(builder.manual_forward(0.0));
    expect(sim_forward.accepted, "sim forward allowed only by sim config");
    expect(!Config{}.autonomous_slam_allow_forward_backward,
           "production forward remains disabled");
    return failures ? 1 : 0;
}
