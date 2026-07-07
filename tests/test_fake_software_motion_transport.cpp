#include "test_software_motion_transport_fakes.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::SoftwareMotionCommand command(double t) {
    robot_slamd::SoftwareMotionCommand c;
    c.direction = robot_slamd::SoftwareMotionDirection::TurnLeft;
    c.speed_normalized = 0.05;
    c.timestamp_s = t;
    c.ttl_s = 0.3;
    c.source = robot_slamd::SoftwareMotionCommandSource::ActiveScan;
    return c;
}
} // namespace

int main() {
    using namespace robot_slamd;

    FakeSoftwareMotionCommandTransport t;
    auto r1 = t.send_command(command(1.0));
    expect(r1.ok && r1.accepted, "send should be accepted");
    expect(t.send_count == 1, "send_count increments");
    expect(t.sent_commands.size() == 1, "sent command recorded");
    expect(r1.transport_sequence == 1, "first sequence");

    auto r2 = t.send_command(command(2.0));
    expect(r2.transport_sequence == 2, "sequence monotonic");

    t.fail_next_send = true;
    auto rf = t.send_command(command(3.0));
    expect(!rf.ok && !rf.accepted, "fail_next_send should fail");
    expect(t.send_count == 3, "failed send counted");

    t.reject_next_send = true;
    auto rr = t.send_command(command(4.0));
    expect(rr.ok && !rr.accepted, "reject_next_send should reject");
    expect(t.send_count == 4, "rejected send counted");

    t.clear();
    expect(t.send_count == 0, "clear send count");
    expect(t.sent_commands.empty(), "clear commands");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
