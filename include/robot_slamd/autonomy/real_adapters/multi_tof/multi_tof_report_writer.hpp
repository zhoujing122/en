#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_acceptance_runner.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class MultiTofReportWriter {
public:
    std::string write_text_report(
        const MultiTofAcceptanceReport &report) const {
        std::ostringstream out;
        out << "Multi-ToF Raw Data Contract Report\n";
        out << "overall ok: " << (report.ok ? "true" : "false") << "\n";
        out << "valid_tof_count: " << report.valid_result.valid_tof_count << "\n";
        out << "front yaw=0, left yaw=+90, right yaw=-90\n";
        out << "passed:\n";
        for (const auto &entry : report.passed) {
            out << "- " << entry << "\n";
        }
        out << "failed:\n";
        for (const auto &entry : report.failed) {
            out << "- " << entry << "\n";
        }
        out << "warnings:\n";
        for (const auto &entry : report.warnings) {
            out << "- " << entry << "\n";
        }
        out << "summary: " << report.summary << "\n";
        out << "This report does not prove real hardware readiness.\n";
        out << "This stage only validates 3-ToF raw data contract.\n";
        out << "3-ToF pairwise sync is deferred to M3-C1.\n";
        out << "Multi-ToF snapshot builder is deferred to M3-C2.\n";
        out << "No real ToF/IMU/Wheel device is read.\n";
        return out.str();
    }
};

} // namespace robot_slamd
