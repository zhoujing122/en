#pragma once

#include "robot_slamd/autonomy/prelive/prelive_autonomous_slam_types.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class PreLiveAutonomousSlamReportWriter {
public:
    std::string write_text_report(const PreLiveIntegrationReport &report) const {
        std::ostringstream out;
        out << "Pre-live Autonomous SLAM Integration Report\n";
        out << "overall_ok: " << (report.ok ? 1 : 0) << "\n";
        out << "stage: " << to_string(report.stage) << "\n";
        out << "block_reason: " << to_string(report.block_reason) << "\n";
        out << "readiness: " << to_string(report.readiness) << "\n";
        out << "final_state: " << to_string(report.final_state) << "\n";
        out << "final_fault: " << to_string(report.final_fault) << "\n";

        out << "counters:\n";
        out << "  readiness_check_count: " << report.counters.readiness_check_count << "\n";
        out << "  contract_check_count: " << report.counters.contract_check_count << "\n";
        out << "  adapter_acceptance_run_count: " << report.counters.adapter_acceptance_run_count << "\n";
        out << "  coordinator_step_count: " << report.counters.coordinator_step_count << "\n";
        out << "  motion_command_count: " << report.counters.motion_command_count << "\n";
        out << "  stop_command_count: " << report.counters.stop_command_count << "\n";
        out << "  active_scan_command_count: " << report.counters.active_scan_command_count << "\n";
        out << "  motion_reject_count: " << report.counters.motion_reject_count << "\n";
        out << "  coordinator_fault_count: " << report.counters.coordinator_fault_count << "\n";

        out << "final_map_quality:\n";
        out << "  good_enough: " << (report.final_map_quality.good_enough ? 1 : 0) << "\n";
        out << "  coverage_ratio: " << report.final_map_quality.coverage_ratio << "\n";
        out << "  yaw_coverage_ratio: " << report.final_map_quality.yaw_coverage_ratio << "\n";
        out << "  valid_scan_count: " << report.final_map_quality.valid_scan_count << "\n";
        out << "  reason: " << report.final_map_quality.reason << "\n";

        write_list(out, "passed_cases", report.passed);
        write_list(out, "failed_cases", report.failed);
        write_list(out, "warnings", report.warnings);

        out << "algorithm_command_count: " << report.algorithm_commands.size() << "\n";
        out << "software_command_count: " << report.software_commands.size() << "\n";
        out << "summary: " << report.summary << "\n";
        out << "This report does not prove real robot motion.\n";
        out << "This report does not prove real ToF/IMU/Wheel driver readiness.\n";
        out << "This report only validates pre-live shadow integration.\n";
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
