#pragma once

#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_scenario_types.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class AutonomousSlamE2EReportWriter {
public:
    std::string write_text_report(
        const AutonomousSlamE2EScenarioReport &report) const {
        std::ostringstream out;
        out << "Autonomous SLAM End-to-End Pre-live Scenario Report\n";
        out << "overall_ok: " << (report.ok ? "true" : "false") << "\n";
        out << "scenario_kind: " << to_string(report.scenario_kind) << "\n";
        out << "stage: " << to_string(report.stage) << "\n";
        out << "block_reason: " << to_string(report.block_reason) << "\n";
        out << "slam_backend_acceptance_ok: "
            << (report.slam_backend_acceptance.ok ? "true" : "false") << "\n";
        out << "prelive_ok: "
            << (report.prelive_report.ok ? "true" : "false") << "\n";
        out << "final_autonomous_slam_state: "
            << to_string(report.prelive_report.final_state) << "\n";
        out << "final_map_quality_good: "
            << (report.prelive_report.final_map_quality.good_enough ? "true" : "false")
            << "\n";
        out << "final_map_quality_coverage: "
            << report.prelive_report.final_map_quality.coverage_ratio << "\n";
        out << "motion_command_count: "
            << report.prelive_report.counters.motion_command_count << "\n";
        out << "stop_command_count: "
            << report.prelive_report.counters.stop_command_count << "\n";
        out << "active_scan_command_count: "
            << report.prelive_report.counters.active_scan_command_count << "\n";
        write_list(out, "passed_cases", report.passed);
        write_list(out, "failed_cases", report.failed);
        write_list(out, "warnings", report.warnings);
        out << "summary: " << report.summary << "\n";
        out << "This report does not prove real robot motion.\n";
        out << "This report does not prove real ToF/IMU/Wheel driver readiness.\n";
        out << "This report does not prove real SLAM backend quality.\n";
        out << "This report only validates end-to-end pre-live shadow integration.\n";
        return out.str();
    }

private:
    static void write_list(std::ostringstream &out,
                           const std::string &name,
                           const std::vector<std::string> &values) {
        out << name << ":\n";
        for (const auto &value : values) {
            out << "  - " << value << "\n";
        }
    }
};

} // namespace robot_slamd
