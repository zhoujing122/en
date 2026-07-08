#include "robot_slamd/autonomy/live_handoff/live_handoff_readiness_checker.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::RealAdapterStubStatus ready_status(
    robot_slamd::RealAdapterStubKind kind) {
    robot_slamd::RealAdapterStubStatus status;
    status.kind = kind;
    status.state = robot_slamd::RealAdapterStubState::StubOnly;
    status.ready = true;
    return status;
}
} // namespace

int main() {
    using namespace robot_slamd;

    const auto ready_sensor = ready_status(RealAdapterStubKind::Sensor);
    const auto ready_motion = ready_status(RealAdapterStubKind::Motion);
    const auto ready_backend = ready_status(RealAdapterStubKind::SlamBackend);
    RealAdapterStubStatus not_ready_sensor;
    not_ready_sensor.kind = RealAdapterStubKind::Sensor;
    RealAdapterStubStatus not_ready_motion;
    not_ready_motion.kind = RealAdapterStubKind::Motion;
    RealAdapterStubStatus not_ready_backend;
    not_ready_backend.kind = RealAdapterStubKind::SlamBackend;

    LiveHandoffReadinessChecker disabled;
    auto report = disabled.evaluate(ready_sensor,
                                    ready_motion,
                                    ready_backend,
                                    true,
                                    true,
                                    true,
                                    true,
                                    true);
    expect(report.block_reason == LiveHandoffBlockReason::ConfigDisabled,
           "disabled config blocks");

    LiveHandoffReadinessOptions options;
    options.enabled = true;
    LiveHandoffReadinessChecker checker(options);

    report = checker.evaluate(not_ready_sensor,
                              ready_motion,
                              ready_backend,
                              true,
                              true,
                              true,
                              true,
                              true);
    expect(report.block_reason ==
               LiveHandoffBlockReason::RealSensorAdapterMissing,
           "sensor missing");

    report = checker.evaluate(ready_sensor,
                              not_ready_motion,
                              ready_backend,
                              true,
                              true,
                              true,
                              true,
                              true);
    expect(report.block_reason ==
               LiveHandoffBlockReason::RealMotionAdapterMissing,
           "motion missing");

    report = checker.evaluate(ready_sensor,
                              ready_motion,
                              not_ready_backend,
                              true,
                              true,
                              true,
                              true,
                              true);
    expect(report.block_reason ==
               LiveHandoffBlockReason::RealSlamBackendMissing,
           "backend missing");

    report = checker.evaluate(ready_sensor,
                              ready_motion,
                              ready_backend,
                              false,
                              true,
                              true,
                              true,
                              true);
    expect(report.block_reason ==
               LiveHandoffBlockReason::SoftwareTransportAcceptanceMissing,
           "transport acceptance missing");

    report = checker.evaluate(ready_sensor,
                              ready_motion,
                              ready_backend,
                              true,
                              false,
                              true,
                              true,
                              true);
    expect(report.block_reason == LiveHandoffBlockReason::E2EPreLiveMissing,
           "e2e missing");

    report = checker.evaluate(ready_sensor,
                              ready_motion,
                              ready_backend,
                              true,
                              true,
                              false,
                              true,
                              true);
    expect(report.block_reason == LiveHandoffBlockReason::DirectionProbeMissing,
           "direction missing");

    report = checker.evaluate(ready_sensor,
                              ready_motion,
                              ready_backend,
                              true,
                              true,
                              true,
                              false,
                              true);
    expect(report.block_reason == LiveHandoffBlockReason::StopEstopTtlMissing,
           "stop ttl missing");

    report = checker.evaluate(ready_sensor,
                              ready_motion,
                              ready_backend,
                              true,
                              true,
                              true,
                              true,
                              false);
    expect(report.block_reason == LiveHandoffBlockReason::HardwareSafetyMissing,
           "hardware safety missing");
    expect(!report.warnings.empty(), "forward backward warning present");

    report = checker.evaluate(ready_sensor,
                              ready_motion,
                              ready_backend,
                              true,
                              true,
                              true,
                              true,
                              true);
    expect(report.state == LiveHandoffReadinessState::StubOnlyReady,
           "all true only stub ready");
    expect(report.state != LiveHandoffReadinessState::LiveReady,
           "never live ready");
    expect(!report.ok, "stub only not ok");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
