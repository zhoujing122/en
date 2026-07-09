#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_fake_motion_port.hpp"
#include "robot_slamd/motion/algorithm_motion_command_builder.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    AlgorithmMotionCommandBuilder builder;
    FullAutonomousSlamFakeMotionPort port;
    expect(port.ready(), "ready");
    expect(port.send_algorithm_command(builder.stop(1.0, "test")).accepted, "stop accepted");
    expect(port.send_algorithm_command(builder.active_scan_turn_left(1.1)).accepted, "left accepted");
    expect(port.send_algorithm_command(builder.active_scan_turn_right(1.2)).accepted, "right accepted");
    auto forward = builder.recovery_forward(1.3);
    expect(!port.send_algorithm_command(forward).accepted, "forward rejected");
    auto backward = builder.recovery_backward(1.4);
    expect(!port.send_algorithm_command(backward).accepted, "backward rejected");
    expect(port.saw_forward_or_backward(), "forward/backward recorded");
    expect(port.command_count() == 5, "command count");
    expect(port.active_scan_command_count() == 2, "scan count");
    expect(port.stop_command_count() == 1, "stop count");
    FullAutonomousSlamFakeMotionPort rejecting(true);
    expect(!rejecting.send_algorithm_command(builder.active_scan_turn_left(2.0)).accepted, "reject motion left");
    return failures ? 1 : 0;
}
