#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_acceptance_runner.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class MultiTofReplayReportWriter {
public:
    std::string write_text_report(
        const MultiTofReplayAcceptanceReport &report) const {
        std::ostringstream out;
        out << "Multi-ToF Replay Report\n";
        out << "overall_ok: " << (report.ok ? "true" : "false") << "\n";
        out << "record_count: " << report.record_count << "\n";
        out << "packet_count: " << report.packet_count << "\n";
        out << "invalid_record_count: " << report.invalid_record_count << "\n";
        out << "consumed_packet_count: " << report.consumed_packet_count << "\n";
        out << "last_status: " << to_string(report.last_result.status) << "\n";
        out << "last_fault: " << to_string(report.last_result.fault) << "\n";
        out << "snapshot_has_multi_tof: "
            << (report.last_result.snapshot.has_multi_tof ? "true" : "false")
            << "\n";
        out << "valid_tof_count: "
            << report.last_result.snapshot.multi_tof.valid_tof_count << "\n";
        out << "degraded: "
            << (report.last_result.snapshot.multi_tof.degraded ? "true" : "false")
            << "\n";
        out << "synchronized_time_s: "
            << report.last_result.snapshot.multi_tof.synchronized_time_s
            << "\n";
        out << "passed_count: " << report.passed_case_count << "\n";
        out << "failed_count: " << report.failed_case_count << "\n";
        out << "summary: " << report.summary << "\n";
        out << "This stage validates in-memory 3-ToF replay/log pipeline.\n";
        out << "No real replay file is read.\n";
        out << "No real ToF/IMU/Wheel hardware is read.\n";
        out << "3-ToF fake full pipeline is deferred to M3-C4.\n";
        return out.str();
    }
};

} // namespace robot_slamd
