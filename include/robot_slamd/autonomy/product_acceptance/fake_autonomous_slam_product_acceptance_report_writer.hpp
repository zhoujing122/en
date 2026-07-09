#pragma once

#include "robot_slamd/autonomy/product_acceptance/fake_autonomous_slam_product_acceptance_runner.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class FakeAutonomousSlamProductAcceptanceReportWriter {
public:
    std::string write_text_report(
        const FakeAutonomousSlamProductAcceptanceReport &report) const {
        std::ostringstream o;
        o << "Fake Autonomous SLAM Product Acceptance Report\n";
        o << "overall ok: " << (report.ok ? "true" : "false") << "\n";
        o << "status: " << to_string(report.status) << "\n";
        o << "block reason: " << to_string(report.block_reason) << "\n";
        o << "mapping pipeline summary: " << report.mapping_report.summary << "\n";
        o << "fake map built: " << (report.mapping_report.fake_map_built ? "true" : "false") << "\n";
        o << "fake map saved: " << (report.mapping_report.fake_map_saved ? "true" : "false") << "\n";
        o << "relocalization summary: " << report.relocalization_report.summary << "\n";
        o << "relocalization confidence: "
          << report.relocalization_report.relocalization_result.pose.confidence << "\n";
        o << "relocalization readiness summary: " << report.readiness_result.summary << "\n";
        o << "pose writeback attempted: "
          << (report.relocalization_report.pose_writeback_attempted ? "true" : "false") << "\n";
        o << "forward/backward observed: false\n";
        o << "adapter replacement manifest:\n" << report.adapter_manifest_text;
        o << "passed cases:\n";
        for (const auto &line : report.passed) {
            o << "- " << line << "\n";
        }
        o << "failed cases:\n";
        for (const auto &line : report.failed) {
            o << "- " << line << "\n";
        }
        o << "warnings:\n";
        for (const auto &line : report.warnings) {
            o << "- " << line << "\n";
        }
        o << "summary: " << report.summary << "\n";
        o << "This acceptance does not prove real hardware readiness.\n";
        o << "This acceptance does not prove production SLAM readiness.\n";
        o << "All sensor, map, motion, and relocalization components are fake/replay/shadow.\n";
        o << "No pose writeback is performed.\n";
        o << "Real hardware integration requires replacing the listed adapters.\n";
        return o.str();
    }
};

} // namespace robot_slamd
