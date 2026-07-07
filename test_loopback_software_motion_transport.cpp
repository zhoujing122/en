#include "software_motion_transport.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::SoftwareMotionCommand command(robot_slamd::SoftwareMotionDirection d) {
    robot_slamd::SoftwareMotionCommand c;
    c.direction = d;
    c.speed_normalized = (d == robot_slamd::SoftwareMotionDirection::Stop ||
                          d == robot_slamd::SoftwareMotionDirection::EmergencyStop) ? 0.0 : 0.03;
    c.timestamp_s = 1.0;
    c.ttl_s = 0.3;
    c.source = robot_slamd::SoftwareMotionCommandSource::ActiveScan;
    return c;
}
} // namespace

int main() {
    using namespace robot_slamd;
    LoopbackSoftwareMotionCommandTransport t(true);
    auto r = t.send_command(command(SoftwareMotionDirection::Stop));
    expect(r.ok && r.accepted, "shadow stop accepted");
    expect(t.sent_commands.size() == 1, "stop recorded");
    r = t.send_command(command(SoftwareMotionDirection::TurnLeft));
    expect(r.ok && r.accepted, "shadow turn accepted");
    expect(t.sent_commands.size() == 2, "turn recorded");
    expect(t.send_count == 2, "send count");

    LoopbackSoftwareMotionCommandTransport non_shadow(false);
    r = non_shadow.send_command(command(SoftwareMotionDirection::Stop));
    expect(r.ok && !r.accepted, "non-shadow rejected");
    expect(non_shadow.reject_count == 1, "non-shadow reject count");

    LoopbackSoftwareMotionCommandTransport stops_only(true);
    stops_only.reject_non_stop = true;
    r = stops_only.send_command(command(SoftwareMotionDirection::TurnLeft));
    expect(r.ok && !r.accepted, "non-stop rejected");
    expect(stops_only.reject_count == 1, "non-stop reject count");
    r = stops_only.send_command(command(SoftwareMotionDirection::EmergencyStop));
    expect(r.ok && r.accepted, "emergency stop accepted");

    stops_only.clear();
    expect(stops_only.sent_commands.empty(), "clear commands");
    expect(stops_only.send_count == 0 && stops_only.reject_count == 0, "clear counts");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
