#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_replay/regression/real_sensor_replay_regression_types.hpp"

#include <sstream>

namespace robot_slamd {

class RealSensorReplayRegressionReportWriter {
public:
    std::string write_text_report(
        const RealSensorReplayRegressionReport &report) const {
        std::ostringstream o;
        o << "Real Sensor Replay Regression Report\n";
        o << "overall ok: " << (report.ok ? "true" : "false") << "\n";
        o << "status: " << to_string(report.status) << "\n";
        o << "block reason: " << to_string(report.block_reason) << "\n";
        o << "case count: " << report.case_count << "\n";
        o << "pass count: " << report.pass_count << "\n";
        o << "fail count: " << report.fail_count << "\n";
        o << "passed:\n";
        for (const auto &entry : report.passed) {
            o << "- " << entry << "\n";
        }
        o << "failed:\n";
        for (const auto &entry : report.failed) {
            o << "- " << entry << "\n";
        }
        o << "warnings:\n";
        for (const auto &entry : report.warnings) {
            o << "- " << entry << "\n";
        }
        o << "summary: " << report.summary << "\n";
        o << "This report does not prove real hardware readiness.\n";
        o << "This report only validates offline replay regression behavior.\n";
        o << "ToF and Wheel timestamps are request-window estimates.\n";
        return o.str();
    }
};

} // namespace robot_slamd
