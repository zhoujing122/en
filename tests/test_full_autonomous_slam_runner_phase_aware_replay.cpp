#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_runner.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FullAutonomousSlamScenarioBuilder builder;
    FullAutonomousSlamRunner runner;
    const auto report = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes));
    expect(report.ok, "success ok");
    expect(report.sensor_consumed_count > 0, "consumed");
    expect(report.sensor_skipped_count > 0, "skipped");
    bool skipped_motion_phase = false;
    bool saw_skipped = false;
    for (const auto &event : report.trace.events) {
        if (event.kind == FullAutonomousSlamTraceEventKind::SensorSkipped) {
            saw_skipped = true;
            if (event.state == AutonomousSlamState::SendingMotionCommand ||
                event.state == AutonomousSlamState::WaitingMotionSettle) {
                skipped_motion_phase = true;
            }
        }
    }
    expect(saw_skipped, "trace skipped");
    expect(skipped_motion_phase, "motion phase skipped");
    expect(report.shadow_motion_command_count >= report.active_scan_command_count, "shadow only count");
    return failures ? 1 : 0;
}
