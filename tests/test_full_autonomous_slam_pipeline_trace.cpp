#include "robot_slamd/autonomy/full_pipeline/trace/full_autonomous_slam_pipeline_trace.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    expect(to_string(FullAutonomousSlamTraceEventKind::SensorSkipped) == "sensor_skipped", "string");
    FullAutonomousSlamPipelineTrace trace;
    FullAutonomousSlamTraceEvent event;
    event.kind = FullAutonomousSlamTraceEventKind::StepStarted;
    append_full_autonomous_slam_trace_event(trace, event);
    event.kind = FullAutonomousSlamTraceEventKind::SensorConsumed;
    append_full_autonomous_slam_trace_event(trace, event);
    event.kind = FullAutonomousSlamTraceEventKind::SensorSkipped;
    append_full_autonomous_slam_trace_event(trace, event);
    event.kind = FullAutonomousSlamTraceEventKind::BackendUpdated;
    append_full_autonomous_slam_trace_event(trace, event);
    event.kind = FullAutonomousSlamTraceEventKind::FakeMapBuilt;
    append_full_autonomous_slam_trace_event(trace, event);
    event.kind = FullAutonomousSlamTraceEventKind::FakeMapSaved;
    append_full_autonomous_slam_trace_event(trace, event);
    expect(trace.events.size() == 6, "event count");
    expect(trace.sensor_consumed_count == 1, "consumed count");
    expect(trace.sensor_skipped_count == 1, "skipped count");
    expect(trace.backend_updated_count == 1, "backend count");
    return failures ? 1 : 0;
}
