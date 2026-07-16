#pragma once

#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_multi_tof_slam_backend_binding.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class DeterministicMultiTofSlamBackendReportWriter {
public:
    std::string write_text_report(
        const DeterministicMultiTofBackendState &state,
        const RobotSlamMapQuality &quality) const {
        std::ostringstream o;
        o << "Deterministic Multi-ToF SLAM Backend Report\n"
          << "stage: " << to_string(state.stage) << "\n"
          << "fault: " << to_string(state.fault) << "\n"
          << "accepted updates: " << state.accepted_update_count << "\n"
          << "rejected updates: " << state.rejected_update_count << "\n"
          << "valid tof count: " << state.valid_tof_count << "\n"
          << "coverage: " << quality.coverage_ratio << "\n"
          << "yaw coverage: " << quality.yaw_coverage_ratio << "\n"
          << "This backend is deterministic proxy logic, not production SLAM.\n"
          << "It requires RobotSlamSensorSnapshot.has_multi_tof.\n";
        return o.str();
    }
};

} // namespace robot_slamd
