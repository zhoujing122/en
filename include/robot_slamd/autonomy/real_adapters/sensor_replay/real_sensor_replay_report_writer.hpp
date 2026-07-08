#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_acceptance_runner.hpp"

#include <sstream>

namespace robot_slamd {

class RealSensorReplayReportWriter {
public:
    std::string write_text_report(
        const RealSensorReplayAcceptanceRunnerResult &result) const {
        std::ostringstream o;
        o << "Real Sensor Replay Report\n";
        o << "overall ok: " << (result.ok ? "true" : "false") << "\n";
        o << "valid log result: " << to_string(result.valid_log_result.status)
          << " accepted=" << result.valid_log_result.accepted_packet_count
          << " rejected=" << result.valid_log_result.rejected_packet_count
          << "\n";
        o << "invalid latency result: "
          << to_string(result.invalid_latency_result.status)
          << " rejected="
          << result.invalid_latency_result.rejected_packet_count << "\n";
        o << "sync too large result: "
          << to_string(result.sync_too_large_result.status)
          << " rejected="
          << result.sync_too_large_result.rejected_packet_count << "\n";
        o << "passed cases:\n";
        for (const auto &entry : result.passed) {
            o << "- " << entry << "\n";
        }
        o << "failed cases:\n";
        for (const auto &entry : result.failed) {
            o << "- " << entry << "\n";
        }
        o << "warnings:\n";
        for (const auto &entry : result.warnings) {
            o << "- " << entry << "\n";
        }
        o << "summary: " << result.summary << "\n";
        o << "This report does not prove real hardware readiness.\n";
        o << "This report does not read real ToF/IMU/Wheel devices.\n";
        o << "This report only validates offline replay/log contract.\n";
        o << "ToF and Wheel timestamps are request-window estimates, not hardware capture timestamps.\n";
        return o.str();
    }
};

} // namespace robot_slamd
