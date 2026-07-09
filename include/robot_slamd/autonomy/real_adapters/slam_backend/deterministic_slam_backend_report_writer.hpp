#pragma once

#include "robot_slamd/autonomy/real_adapters/slam_backend/regression/replay_to_slam_backend_regression_runner.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class DeterministicSlamBackendReportWriter {
public:
    std::string write_text_report(
        const ReplayToSlamBackendRegressionReport &report) const {
        std::ostringstream o;
        o << "Deterministic SLAM Backend Skeleton Report\n";
        o << "overall ok: " << (report.ok ? "true" : "false") << "\n";
        o << "accepted update count: " << report.accepted_update_count << "\n";
        o << "rejected update count: " << report.rejected_update_count << "\n";
        o << "final quality good: "
          << (report.final_quality.good_enough ? "true" : "false") << "\n";
        o << "coverage_ratio: " << report.final_quality.coverage_ratio << "\n";
        o << "yaw_coverage_ratio: "
          << report.final_quality.yaw_coverage_ratio << "\n";
        o << "valid_scan_count: " << report.final_quality.valid_scan_count << "\n";
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
        o << "This report does not prove real SLAM readiness.\n";
        o << "This backend is deterministic skeleton, not production mapping.\n";
        o << "Coverage ratio is a skeleton proxy, not real map coverage.\n";
        o << "No real hardware, no real motion, no real map file is used.\n";
        return o.str();
    }
};

} // namespace robot_slamd
