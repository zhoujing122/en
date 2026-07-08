#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_adapter_acceptance_runner.hpp"

#include <sstream>

namespace robot_slamd {

class RealSensorAdapterReportWriter {
public:
    std::string write_text_report(
        const RealSensorAdapterAcceptanceRunnerResult &result) const {
        std::ostringstream out;
        out << "Real Sensor Adapter Data Contract Report\n";
        out << "overall ok: " << (result.ok ? "true" : "false") << "\n";
        out << "valid packet result: "
            << (result.valid_build_result.ok ? "ok" : "failed") << "\n";
        out << "request-based ToF/Wheel timing note: ToF and Wheel "
               "timestamps are request-window estimates.\n";
        out << "passed cases:\n";
        for (const auto &item : result.passed) {
            out << "- " << item << "\n";
        }
        out << "failed cases:\n";
        for (const auto &item : result.failed) {
            out << "- " << item << "\n";
        }
        out << "warnings:\n";
        for (const auto &item : result.warnings) {
            out << "- " << item << "\n";
        }
        out << "summary: " << result.summary << "\n";
        out << "This report does not prove real ToF driver readiness.\n";
        out << "This report does not prove real IMU driver readiness.\n";
        out << "This report does not prove real Wheel driver readiness.\n";
        out << "This report only validates the real sensor adapter data "
               "contract skeleton.\n";
        out << "ToF and Wheel timestamps are request-window estimates, not "
               "hardware capture timestamps.\n";
        return out.str();
    }
};

} // namespace robot_slamd
