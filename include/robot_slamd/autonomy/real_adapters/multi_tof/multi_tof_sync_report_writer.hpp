#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_acceptance_runner.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class MultiTofSyncReportWriter {
public:
    std::string write_text_report(
        const MultiTofSyncAcceptanceReport &report,
        const MultiTofSyncOptions &options = {}) const {
        std::ostringstream out;
        out << "Multi-ToF Sync Report\n";
        out << "overall ok: " << (report.ok ? "true" : "false") << "\n";
        out << "valid_tof_count: " << report.valid_result.valid_tof_count << "\n";
        out << "degraded: " << (report.degraded_result.degraded ? "true" : "false") << "\n";
        out << "front-left dt: " << report.valid_result.front_left_dt_s << "\n";
        out << "front-right dt: " << report.valid_result.front_right_dt_s << "\n";
        out << "left-right dt: " << report.valid_result.left_right_dt_s << "\n";
        out << "multi-tof imu dt: " << report.valid_result.multi_tof_imu_dt_s << "\n";
        out << "multi-tof wheel dt: " << report.valid_result.multi_tof_wheel_dt_s << "\n";
        out << "time reference: " << to_string(options.time_reference) << "\n";
        out << "degradation mode: " << to_string(options.degradation_mode) << "\n";
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
        out << "This report validates 3-ToF + IMU + Wheel sync contract.\n";
        out << "This stage does not build Multi-ToF snapshot.\n";
        out << "This stage does not read real hardware.\n";
        out << "ToF/Wheel timestamps are request-window estimates.\n";
        return out.str();
    }
};

} // namespace robot_slamd
