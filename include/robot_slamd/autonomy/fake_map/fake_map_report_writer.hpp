#pragma once

#include "robot_slamd/autonomy/fake_map/fake_map_artifact_types.hpp"

#include <sstream>
#include <string>

namespace robot_slamd {

class FakeMapReportWriter {
public:
    std::string write_text_report(
        const FakeMapStorageResult &result) const {
        std::ostringstream o;
        const auto &artifact = result.artifact;
        o << "Fake Map Artifact Report\n"
          << "overall ok: " << (result.ok ? "true" : "false") << "\n"
          << "status: " << to_string(artifact.status) << "\n"
          << "fault: " << to_string(artifact.fault) << "\n"
          << "map_id: " << artifact.metadata.map_id << "\n"
          << "scenario: " << artifact.metadata.scenario_name << "\n"
          << "coverage: " << artifact.metadata.coverage_ratio << "\n"
          << "yaw coverage: " << artifact.metadata.yaw_coverage_ratio << "\n"
          << "valid scan count: "
          << artifact.metadata.valid_scan_count << "\n"
          << "accepted updates: "
          << artifact.metadata.accepted_update_count << "\n"
          << "active scan commands: "
          << artifact.metadata.active_scan_command_count << "\n"
          << "serialized text:\n"
          << artifact.serialized_text
          << "passed:\n";
        for (const auto &item : result.passed) {
            o << "- " << item << "\n";
        }
        o << "failed:\n";
        for (const auto &item : result.failed) {
            o << "- " << item << "\n";
        }
        o << "warnings:\n";
        for (const auto &item : result.warnings) {
            o << "- " << item << "\n";
        }
        o << "summary: " << result.summary << "\n"
          << "Fake map artifact is metadata only.\n"
          << "No real map file is written.\n"
          << "Fake map does not contain occupancy grid data.\n";
        return o.str();
    }
};

} // namespace robot_slamd
