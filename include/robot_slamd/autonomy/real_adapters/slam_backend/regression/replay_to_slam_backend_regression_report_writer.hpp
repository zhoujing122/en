#pragma once

#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_report_writer.hpp"

namespace robot_slamd {

class ReplayToSlamBackendRegressionReportWriter {
public:
    std::string write_text_report(
        const ReplayToSlamBackendRegressionReport &report) const {
        DeterministicSlamBackendReportWriter writer;
        return writer.write_text_report(report);
    }
};

} // namespace robot_slamd
