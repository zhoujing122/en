#include "robot_slamd/motion/algorithm_motion_command_adapter.hpp"
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

robot_slamd::AlgorithmMotionCommand base(robot_slamd::AlgorithmMotionKind kind,
                                         robot_slamd::AlgorithmMotionProfile profile) {
    robot_slamd::AlgorithmMotionCommand c;
    c.kind = kind;
    c.profile = profile;
    c.timestamp_s = 10.0;
    c.ttl_s = 0.3;
    c.duration_s = 0.2;
    c.speed_normalized = 0.02;
    c.reason = "adapter_test";
    c.sequence = 42;
    return c;
}
} // namespace

int main() {
    using namespace robot_slamd;
    AlgorithmMotionCommandAdapter adapter;

    auto stop = base(AlgorithmMotionKind::Stop, AlgorithmMotionProfile::Safety);
    stop.speed_normalized = 0.0;
    stop.duration_s = 0.0;
    auto r = adapter.to_software_command(stop);
    expect(r.ok && r.command.direction == SoftwareMotionDirection::Stop, "stop maps to stop");

    auto emergency = base(AlgorithmMotionKind::EmergencyStop, AlgorithmMotionProfile::Safety);
    emergency.speed_normalized = 0.0;
    emergency.duration_s = 0.0;
    r = adapter.to_software_command(emergency);
    expect(r.ok && r.command.direction == SoftwareMotionDirection::EmergencyStop, "emergency maps to emergency");

    r = adapter.to_software_command(base(AlgorithmMotionKind::DirectionProbeLeft, AlgorithmMotionProfile::DirectionProbe));
    expect(r.ok && r.command.direction == SoftwareMotionDirection::TurnLeft, "direction probe left maps to turn left");
    expect(r.command.source == SoftwareMotionCommandSource::ManualTest, "direction probe source manual test");

    r = adapter.to_software_command(base(AlgorithmMotionKind::DirectionProbeRight, AlgorithmMotionProfile::DirectionProbe));
    expect(r.ok && r.command.direction == SoftwareMotionDirection::TurnRight, "direction probe right maps to turn right");

    r = adapter.to_software_command(base(AlgorithmMotionKind::ActiveScanTurnLeft, AlgorithmMotionProfile::ActiveScan));
    expect(r.ok && r.command.direction == SoftwareMotionDirection::TurnLeft, "active scan left maps to turn left");
    expect(r.command.source == SoftwareMotionCommandSource::ActiveScan, "active scan source");

    r = adapter.to_software_command(base(AlgorithmMotionKind::ActiveScanTurnRight, AlgorithmMotionProfile::ActiveScan));
    expect(r.ok && r.command.direction == SoftwareMotionDirection::TurnRight, "active scan right maps to turn right");

    r = adapter.to_software_command(base(AlgorithmMotionKind::RecoveryForward, AlgorithmMotionProfile::Recovery));
    expect(!r.ok && r.error == "recovery_commands_disabled", "recovery disabled blocks recovery forward");

    r = adapter.to_software_command(base(AlgorithmMotionKind::Forward, AlgorithmMotionProfile::ManualTest));
    expect(!r.ok && r.error == "manual_test_commands_disabled", "manual disabled blocks forward first");

    AlgorithmToSoftwareMotionOptions opts;
    opts.allow_translation_commands = true;
    opts.allow_recovery_commands = true;
    AlgorithmMotionCommandAdapter recovery_adapter(opts);
    r = recovery_adapter.to_software_command(base(AlgorithmMotionKind::RecoveryForward, AlgorithmMotionProfile::Recovery));
    expect(r.ok && r.command.direction == SoftwareMotionDirection::Forward, "recovery forward maps to forward when enabled");
    expect(r.command.source == SoftwareMotionCommandSource::Recovery, "recovery source");
    expect(r.command.reason == "adapter_test", "reason passthrough");
    expect(r.command.sequence == 42, "sequence passthrough");
    expect(r.command.timestamp_s == 10.0, "timestamp passthrough");
    expect(r.command.ttl_s == 0.3, "ttl passthrough");

    AlgorithmMotionCommandAdapter bad_adapter;
    auto bad = base(AlgorithmMotionKind::TurnLeft, AlgorithmMotionProfile::Safety);
    r = bad_adapter.to_software_command(bad);
    expect(!r.ok && r.error.find("software_motion_command_invalid:") == 0, "software validation prefix");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
