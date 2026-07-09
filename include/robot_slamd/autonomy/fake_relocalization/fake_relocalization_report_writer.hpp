#pragma once

#include "robot_slamd/autonomy/fake_relocalization/fake_map_relocalization_runner.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class FakeRelocalizationReportWriter {
public:
    std::string write_text_report(
        const FakeRelocalizationResult &result) const {
        std::ostringstream o;
        o << "Fake Relocalization Report\n"
          << "overall ok: " << (result.ok ? "true" : "false") << "\n"
          << "status: " << to_string(result.status) << "\n"
          << "fault: " << to_string(result.fault) << "\n"
          << "map id: " << result.map_artifact.metadata.map_id << "\n"
          << "map coverage: " << result.map_artifact.final_quality.coverage_ratio << "\n"
          << "map yaw coverage: " << result.map_artifact.final_quality.yaw_coverage_ratio << "\n"
          << "valid range count: " << result.valid_range_count << "\n"
          << "valid range ratio: " << result.valid_range_ratio << "\n"
          << "confidence: " << result.pose.confidence << "\n"
          << "confidence band: " << to_string(result.confidence_band) << "\n"
          << "pose candidate x: " << result.pose.x_m << "\n"
          << "pose candidate y: " << result.pose.y_m << "\n"
          << "pose candidate yaw: " << result.pose.yaw_rad << "\n"
          << "passed:\n";
        for (const auto &item : result.passed) {
            o << "- " << item << "\n";
        }
        o << "failed:\n";
        for (const auto &item : result.failed) {
            o << "- " << item << "\n";
        }
        o << "warnings:\n";
        for (const auto &item : result.warnings) {
            o << "- " << item << "\n";
        }
        append_disclaimer(o, result.summary);
        return o.str();
    }

    std::string write_runner_report(
        const FakeMapRelocalizationRunnerReport &report) const {
        std::ostringstream o;
        o << "Fake Relocalization Report\n"
          << "overall ok: " << (report.ok ? "true" : "false") << "\n"
          << "pipeline summary: " << report.pipeline_report.summary << "\n"
          << "runner summary: " << report.summary << "\n"
          << "map loaded: " << (report.map_loaded ? "true" : "false") << "\n"
          << "replay snapshot count: " << report.replay_snapshot_count << "\n"
          << "pose writeback attempted: " << (report.pose_writeback_attempted ? "true" : "false") << "\n"
          << write_text_report(report.relocalization_result);
        return o.str();
    }

private:
    static void append_disclaimer(
        std::ostringstream &o,
        const std::string &summary) {
        o << "summary: " << summary << "\n"
          << "This report does not prove real relocalization readiness.\n"
          << "Pose candidate is fake and report-only.\n"
          << "No pose writeback is performed.\n"
          << "No real map file is read.\n"
          << "No real hardware is used.\n";
    }
};

} // namespace robot_slamd
