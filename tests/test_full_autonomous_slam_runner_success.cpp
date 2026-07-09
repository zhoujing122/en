#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_runner.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FullAutonomousSlamScenarioBuilder builder;
    FullAutonomousSlamRunner runner;
    auto complete = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::CompletesWithoutActiveScan));
    expect(complete.ok, "complete ok");
    expect(complete.final_state == AutonomousSlamState::Completed, "complete state");
    expect(complete.final_quality.good_enough, "complete quality");
    expect(complete.backend_accepted_update_count >= 1, "complete backend count");
    expect(complete.active_scan_command_count == 0, "complete no active scan");
    auto active = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes));
    expect(active.ok, "active ok");
    expect(active.final_state == AutonomousSlamState::Completed, "active state");
    expect(active.final_quality.good_enough, "active quality");
    expect(active.scenario == FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes, "active scenario set");
    expect(active.backend_accepted_update_count >= 2, "active backend count");
    expect(active.active_scan_command_count > 0, "active scan happened");
    expect(active.summary == "full_autonomous_slam_pipeline_passed", "summary passed");
    return failures ? 1 : 0;
}
