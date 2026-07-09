#pragma once

#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_pipeline_types.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class FullAutonomousSlamReportWriter {
public:
    std::string write_text_report(
        const FullAutonomousSlamPipelineReport &report) const {
        std::ostringstream o;
        o << "Full Autonomous SLAM Fake Pipeline Report\n";
        o << "overall ok: " << (report.ok ? "true" : "false") << "\n";
        o << "scenario: " << to_string(report.scenario) << "\n";
        o << "final stage: " << to_string(report.stage) << "\n";
        o << "final state: " << to_string(report.final_state) << "\n";
        o << "final fault: " << to_string(report.fault) << "\n";
        o << "step count: " << report.step_count << "\n";
        o << "sensor snapshot count: " << report.sensor_snapshot_count << "\n";
        o << "sensor consumed count: " << report.sensor_consumed_count << "\n";
        o << "sensor skipped count: " << report.sensor_skipped_count << "\n";
        o << "backend accepted update count: " << report.backend_accepted_update_count << "\n";
        o << "backend rejected update count: " << report.backend_rejected_update_count << "\n";
        o << "active scan command count: " << report.active_scan_command_count << "\n";
        o << "stop command count: " << report.stop_command_count << "\n";
        o << "final coverage_ratio: " << report.final_quality.coverage_ratio << "\n";
        o << "final yaw_coverage_ratio: " << report.final_quality.yaw_coverage_ratio << "\n";
        o << "final valid_scan_count: " << report.final_quality.valid_scan_count << "\n";
        o << "fake map built: " << (report.fake_map_built ? "true" : "false") << "\n";
        o << "fake map saved: " << (report.fake_map_saved ? "true" : "false") << "\n";
        o << "fake map id: " << report.fake_map_artifact.metadata.map_id << "\n";
        o << "trace event count: " << report.trace.events.size() << "\n";
        o << "trace sensor consumed: " << report.trace.sensor_consumed_count << "\n";
        o << "trace sensor skipped: " << report.trace.sensor_skipped_count << "\n";
        o << "trace backend updated: " << report.trace.backend_updated_count << "\n";
        o << "trace motion command sent: " << report.trace.motion_command_sent_count << "\n";
        o << "trace fault events: " << report.trace.fault_event_count << "\n";
        o << "passed cases:\n";
        for (const auto &entry : report.passed) {
            o << "- " << entry << "\n";
        }
        o << "failed cases:\n";
        for (const auto &entry : report.failed) {
            o << "- " << entry << "\n";
        }
        o << "warnings:\n";
        for (const auto &entry : report.warnings) {
            o << "- " << entry << "\n";
        }
        o << "summary: " << report.summary << "\n";
        o << "This report does not prove real hardware readiness.\n";
        o << "This report does not prove production SLAM readiness.\n";
        o << "This pipeline uses fake/replay sensor data.\n";
        o << "This pipeline uses deterministic SLAM backend skeleton.\n";
        o << "This pipeline uses fake/shadow motion only.\n";
        o << "No real robot motion is executed.\n";
        o << "Fake map artifact is metadata only.\n";
        o << "No real map file is written.\n";
        o << "Fake map does not contain occupancy grid data.\n";
        return o.str();
    }
};

} // namespace robot_slamd
