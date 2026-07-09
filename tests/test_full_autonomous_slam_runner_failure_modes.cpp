#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_runner.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FullAutonomousSlamScenarioBuilder builder;
    FullAutonomousSlamRunner runner;
    auto sensor = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::SensorReplayInvalid));
    expect(!sensor.ok, "sensor invalid fails");
    expect(sensor.fault == FullAutonomousSlamPipelineFault::SensorSnapshotMissing, "sensor fault");
    auto backend = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::BackendRejected));
    expect(!backend.ok, "backend rejected fails");
    expect(backend.fault == FullAutonomousSlamPipelineFault::BackendNotReady, "backend fault");
    auto motion = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::MotionRejected));
    expect(!motion.ok, "motion rejected fails");
    expect(motion.fault == FullAutonomousSlamPipelineFault::MotionRejected || motion.fault == FullAutonomousSlamPipelineFault::CoordinatorFault, "motion fault");
    auto stuck = runner.run_once(builder.build(FullAutonomousSlamPipelineScenarioKind::MapQualityStuck));
    expect(!stuck.ok, "stuck fails");
    expect(stuck.fault == FullAutonomousSlamPipelineFault::MaxStepsExceeded || stuck.fault == FullAutonomousSlamPipelineFault::MapQualityStuck || stuck.fault == FullAutonomousSlamPipelineFault::CoordinatorFault, "stuck fault");
    auto maxed = builder.build(FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes);
    maxed.pipeline_options.max_steps = 1;
    auto maxed_report = runner.run_once(maxed);
    expect(!maxed_report.ok, "max steps fails");
    expect(maxed_report.fault == FullAutonomousSlamPipelineFault::MaxStepsExceeded, "max steps fault");
    return failures ? 1 : 0;
}
