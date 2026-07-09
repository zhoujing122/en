#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_pipeline_types.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    expect(to_string(FullAutonomousSlamPipelineStage::Completed) == "completed", "stage string");
    expect(to_string(FullAutonomousSlamPipelineScenarioKind::BackendRejected) == "backend_rejected", "scenario string");
    expect(to_string(FullAutonomousSlamPipelineFault::MotionRejected) == "motion_rejected", "fault string");
    expect(full_autonomous_slam_pipeline_stage_id(FullAutonomousSlamPipelineStage::Fault) > 0, "stage id");
    expect(full_autonomous_slam_pipeline_scenario_kind_id(FullAutonomousSlamPipelineScenarioKind::SensorReplayInvalid) > 0, "scenario id");
    expect(full_autonomous_slam_pipeline_fault_id(FullAutonomousSlamPipelineFault::BackendRejected) > 0, "fault id");
    FullAutonomousSlamPipelineOptions options;
    expect(!options.enabled, "default disabled");
    FullAutonomousSlamPipelineReport report;
    expect(!report.ok, "default report not ok");
    return failures ? 1 : 0;
}
