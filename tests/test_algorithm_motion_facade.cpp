#include "robot_slamd/motion/algorithm_motion_facade.hpp"
#include "robot_slamd/software_motion/loopback_software_motion_transport.hpp"

#include <iostream>
#include <memory>

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

    AlgorithmMotionFacade missing(nullptr,
                                  AlgorithmMotionCommandBuilder(),
                                  AlgorithmMotionCommandAdapter());
    auto r = missing.stop(1.0);
    expect(!r.ok && r.error == "software_motion_transport_missing", "missing transport rejected");

    auto transport = std::make_shared<LoopbackSoftwareMotionCommandTransport>(true);
    AlgorithmMotionFacade facade(transport,
                                 AlgorithmMotionCommandBuilder(),
                                 AlgorithmMotionCommandAdapter());
    r = facade.stop(2.0);
    expect(r.ok && r.accepted, "stop accepted");
    expect(transport->sent_commands.back().direction == SoftwareMotionDirection::Stop, "stop sent");

    r = facade.emergency_stop(2.1);
    expect(r.ok && transport->sent_commands.back().direction == SoftwareMotionDirection::EmergencyStop, "emergency sent");

    r = facade.direction_probe_left(3.0);
    expect(r.ok && transport->sent_commands.back().direction == SoftwareMotionDirection::TurnLeft, "direction probe left sent");

    r = facade.active_scan_turn_right(4.0);
    expect(r.ok && transport->sent_commands.back().direction == SoftwareMotionDirection::TurnRight, "active scan right sent");

    auto failing_transport = std::make_shared<LoopbackSoftwareMotionCommandTransport>(false);
    AlgorithmMotionFacade failing(failing_transport,
                                  AlgorithmMotionCommandBuilder(),
                                  AlgorithmMotionCommandAdapter());
    r = failing.stop(5.0);
    expect(!r.ok && r.error == "software_motion_rejected", "transport rejected");

    AlgorithmMotionFacade recovery_blocked(transport,
                                           AlgorithmMotionCommandBuilder(),
                                           AlgorithmMotionCommandAdapter());
    size_t before = transport->sent_commands.size();
    r = recovery_blocked.recovery_forward(6.0);
    expect(!r.ok && r.error == "recovery_commands_disabled", "recovery forward default blocked");
    expect(transport->sent_commands.size() == before, "blocked recovery not sent");

    AlgorithmToSoftwareMotionOptions opts;
    opts.allow_translation_commands = true;
    opts.allow_recovery_commands = true;
    AlgorithmMotionFacade recovery_enabled(transport,
                                           AlgorithmMotionCommandBuilder(),
                                           AlgorithmMotionCommandAdapter(opts));
    r = recovery_enabled.recovery_forward(7.0);
    expect(r.ok && transport->sent_commands.back().direction == SoftwareMotionDirection::Forward, "recovery forward sent when enabled");

    transport->reject_non_stop = true;
    r = facade.active_scan_turn_left(8.0);
    expect(!r.ok && r.error == "software_motion_rejected", "non-stop rejected");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
