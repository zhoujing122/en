#include "robot_slamd/autonomy/real_adapters/real_software_motion_port_stub.hpp"
#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
} // namespace

int main() {
    using namespace robot_slamd;

    RealSoftwareMotionPortStub port;
    expect(!port.ready(), "default not ready");

    AlgorithmMotionCommandBuilder builder;
    const auto command = builder.stop(100.0, "stub_test");
    const auto result = port.send_algorithm_command(command);
    expect(!result.ok, "send not ok");
    expect(!result.accepted, "send rejected");
    expect(result.error.find("real_software_motion_port_stub_not_implemented") !=
               std::string::npos,
           "stable reject error");
    expect(port.rejected_commands().size() == 1, "records rejected command");

    const auto feedback = port.latest_feedback(101.0);
    expect(!feedback.command_active, "not active");
    expect(feedback.command_settled, "settled");
    expect(feedback.fault, "fault feedback");
    expect(feedback.fault_reason ==
               "real_software_motion_port_stub_not_implemented",
           "feedback reason");
    expect(port.status().find("waiting_for_real_motion_adapter_implementation") !=
               std::string::npos,
           "status names implementation wait");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
