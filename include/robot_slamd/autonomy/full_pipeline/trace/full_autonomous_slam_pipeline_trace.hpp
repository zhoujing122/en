#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class FullAutonomousSlamTraceEventKind {
    StepStarted,
    SensorConsumed,
    SensorSkipped,
    BackendUpdated,
    BackendRejected,
    CoordinatorUpdated,
    MotionCommandSent,
    MotionRejected,
    MapQualityGood,
    MapQualityPoor,
    FakeMapBuilt,
    FakeMapSaved,
    Completed,
    Fault
};

struct FullAutonomousSlamTraceEvent {
    FullAutonomousSlamTraceEventKind kind =
        FullAutonomousSlamTraceEventKind::StepStarted;
    int step_index = 0;
    double now_s = 0.0;
    AutonomousSlamState state = AutonomousSlamState::Idle;
    bool sensor_consumed = false;
    bool backend_updated = false;
    bool motion_command_sent = false;
    bool map_quality_good = false;
    std::string message;
};

struct FullAutonomousSlamPipelineTrace {
    std::vector<FullAutonomousSlamTraceEvent> events;
    int sensor_consumed_count = 0;
    int sensor_skipped_count = 0;
    int backend_updated_count = 0;
    int motion_command_sent_count = 0;
    int fault_event_count = 0;
};

inline std::string to_string(
    FullAutonomousSlamTraceEventKind kind) {
    switch (kind) {
    case FullAutonomousSlamTraceEventKind::StepStarted:
        return "step_started";
    case FullAutonomousSlamTraceEventKind::SensorConsumed:
        return "sensor_consumed";
    case FullAutonomousSlamTraceEventKind::SensorSkipped:
        return "sensor_skipped";
    case FullAutonomousSlamTraceEventKind::BackendUpdated:
        return "backend_updated";
    case FullAutonomousSlamTraceEventKind::BackendRejected:
        return "backend_rejected";
    case FullAutonomousSlamTraceEventKind::CoordinatorUpdated:
        return "coordinator_updated";
    case FullAutonomousSlamTraceEventKind::MotionCommandSent:
        return "motion_command_sent";
    case FullAutonomousSlamTraceEventKind::MotionRejected:
        return "motion_rejected";
    case FullAutonomousSlamTraceEventKind::MapQualityGood:
        return "map_quality_good";
    case FullAutonomousSlamTraceEventKind::MapQualityPoor:
        return "map_quality_poor";
    case FullAutonomousSlamTraceEventKind::FakeMapBuilt:
        return "fake_map_built";
    case FullAutonomousSlamTraceEventKind::FakeMapSaved:
        return "fake_map_saved";
    case FullAutonomousSlamTraceEventKind::Completed:
        return "completed";
    case FullAutonomousSlamTraceEventKind::Fault:
        return "fault";
    }
    return "unknown";
}

inline int full_autonomous_slam_trace_event_kind_id(
    FullAutonomousSlamTraceEventKind kind) {
    return static_cast<int>(kind);
}

inline void append_full_autonomous_slam_trace_event(
    FullAutonomousSlamPipelineTrace &trace,
    FullAutonomousSlamTraceEvent event) {
    if (event.kind == FullAutonomousSlamTraceEventKind::SensorConsumed ||
        event.sensor_consumed) {
        trace.sensor_consumed_count++;
    }
    if (event.kind == FullAutonomousSlamTraceEventKind::SensorSkipped) {
        trace.sensor_skipped_count++;
    }
    if (event.kind == FullAutonomousSlamTraceEventKind::BackendUpdated ||
        event.kind == FullAutonomousSlamTraceEventKind::BackendRejected ||
        event.backend_updated) {
        trace.backend_updated_count++;
    }
    if (event.kind == FullAutonomousSlamTraceEventKind::MotionCommandSent ||
        event.motion_command_sent) {
        trace.motion_command_sent_count++;
    }
    if (event.kind == FullAutonomousSlamTraceEventKind::Fault ||
        event.kind == FullAutonomousSlamTraceEventKind::MotionRejected) {
        trace.fault_event_count++;
    }
    trace.events.push_back(event);
}

} // namespace robot_slamd
