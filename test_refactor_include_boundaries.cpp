#include "software_motion_command.hpp"
#include "software_motion_transport.hpp"
#include "loopback_software_motion_transport.hpp"
#include "software_direction_speed_motion_command_writer.hpp"
#include "motion_hardware_preflight.hpp"
#include "manual_motion_arm_gate.hpp"
#include "config.hpp"
#include "metrics.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

std::string source_dir() {
    std::string file = __FILE__;
    size_t slash = file.find_last_of("/\\");
    return slash == std::string::npos ? "." : file.substr(0, slash);
}

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}
} // namespace

int main() {
    using namespace robot_slamd;

    SoftwareMotionCommand command;
    expect(command.direction == SoftwareMotionDirection::Stop, "default software motion command is stop");

    LoopbackSoftwareMotionCommandTransport loopback(true);
    auto result = loopback.send_command(command);
    expect(result.ok && result.accepted, "loopback shadow accepts default stop");
    expect(loopback.sent_commands.size() == 1, "loopback records command");

    MotionHardwarePreflightInput preflight;
    expect(preflight.mode == MotionHardwarePreflightMode::ConfirmedLiftedLive, "default preflight mode is confirmed live");

    ManualMotionArmGate arm_gate;
    ManualMotionArmRequest arm_request;
    auto arm = arm_gate.evaluate(arm_request);
    expect(!arm.armed, "manual arm defaults closed");

    Config cfg;
    RunMetrics metrics;
    expect(cfg.motion_execution_writer_backend == "null", "default writer backend null");
    expect(metrics.software_motion.writer_backend == 0, "default software motion metrics backend zero");

    const std::string dir = source_dir() + "/";
    const std::vector<std::string> production_files = {
        "app.hpp",
        "main.cpp",
        "software_motion_transport.hpp",
        "loopback_software_motion_transport.hpp",
        "software_direction_speed_motion_command_writer.hpp",
        "motion_command_writer.hpp",
        "motion_write_controller.hpp",
        "bl4820_motion_safety_executor.hpp",
        "config.hpp",
        "metrics.hpp",
    };
    for (const auto &file : production_files) {
        const auto text = read_file(dir + file);
        expect(text.find("test_software_motion_transport_fakes.hpp") == std::string::npos,
               (file + " must not include test fake transport").c_str());
    }

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
