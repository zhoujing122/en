#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_acceptance_runner.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class MultiTofSnapshotReportWriter {
public:
    std::string write_text_report(
        const MultiTofSnapshotAcceptanceReport &report) const {
        std::ostringstream out;
        out << "Multi-ToF Snapshot Report\n";
        out << "overall_ok: " << (report.ok ? "true" : "false") << "\n";
        out << "has_multi_tof: "
            << (report.valid_result.snapshot.has_multi_tof ? "true" : "false")
            << "\n";
        out << "has_legacy_tof: "
            << (report.valid_result.snapshot.has_tof ? "true" : "false")
            << "\n";
        out << "synchronized_time_s: "
            << report.valid_result.snapshot.multi_tof.synchronized_time_s
            << "\n";
        out << "valid_tof_count: " << report.valid_result.valid_tof_count
            << "\n";
        out << "front_present: "
            << (report.valid_result.snapshot.multi_tof.has_front ? "true" : "false")
            << "\n";
        out << "left_present: "
            << (report.valid_result.snapshot.multi_tof.has_left ? "true" : "false")
            << "\n";
        out << "right_present: "
            << (report.valid_result.snapshot.multi_tof.has_right ? "true" : "false")
            << "\n";
        out << "legacy_primary: front\n";
        out << "passed_count: " << report.passed_case_count << "\n";
        out << "failed_count: " << report.failed_case_count << "\n";
        out << "summary: " << report.summary << "\n";
        out << "This report does not prove real hardware readiness.\n";
        out << "This stage builds synchronized Multi-ToF snapshots only.\n";
        out << "This stage does not read real hardware.\n";
        out << "This stage does not implement Multi-ToF replay/log.\n";
        return out.str();
    }
};

} // namespace robot_slamd
