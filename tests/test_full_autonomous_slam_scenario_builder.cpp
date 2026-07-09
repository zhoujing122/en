#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_scenario_builder.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FullAutonomousSlamScenarioBuilder builder;
    const auto complete = builder.build(FullAutonomousSlamPipelineScenarioKind::CompletesWithoutActiveScan);
    expect(!complete.name.empty(), "complete name");
    expect(!complete.replay_log.records.empty(), "complete replay");
    expect(complete.backend_options.min_valid_scan_count_for_good == 1, "complete quickly good");
    const auto active = builder.build(FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes);
    expect(!active.name.empty(), "active name");
    expect(!active.replay_log.records.empty(), "active replay");
    expect(active.backend_options.min_valid_scan_count_for_good >= 2, "active needs scans");
    const auto invalid = builder.build(FullAutonomousSlamPipelineScenarioKind::SensorReplayInvalid);
    expect(!invalid.replay_log.records.empty(), "invalid replay exists");
    const auto backend = builder.build(FullAutonomousSlamPipelineScenarioKind::BackendRejected);
    expect(!backend.backend_options.ready, "backend rejected not ready");
    const auto motion = builder.build(FullAutonomousSlamPipelineScenarioKind::MotionRejected);
    expect(motion.reject_motion, "motion rejected flag");
    const auto stuck = builder.build(FullAutonomousSlamPipelineScenarioKind::MapQualityStuck);
    expect(stuck.backend_options.min_valid_scan_count_for_good > stuck.pipeline_options.max_steps, "stuck cannot become good");
    return failures ? 1 : 0;
}
