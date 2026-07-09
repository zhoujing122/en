#pragma once

#include "robot_slamd/autonomy/fake_map/fake_map_artifact_types.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_pipeline_types.hpp"

#include <sstream>
#include <string>
#include <vector>

namespace robot_slamd {

class FakeMapStorage {
public:
    FakeMapStorage(
        FakeMapSaveOptions save_options = {},
        FakeMapLoadOptions load_options = {})
        : save_options_(save_options),
          load_options_(load_options) {}

    FakeMapStorageResult build_from_pipeline_report(
        const FullAutonomousSlamPipelineReport &report,
        const std::string &map_id,
        double created_time_s) const {
        FakeMapStorageResult result;
        if (save_options_.require_completed_pipeline && !report.ok) {
            return reject(
                result,
                FakeMapArtifactFault::PipelineNotCompleted,
                "fake_map_pipeline_not_completed");
        }
        if (save_options_.require_quality_good &&
            !report.final_quality.good_enough) {
            return reject(
                result,
                FakeMapArtifactFault::QualityNotGood,
                "fake_map_quality_not_good");
        }
        if (map_id.empty()) {
            return reject(
                result,
                FakeMapArtifactFault::EmptyMapId,
                "fake_map_empty_map_id");
        }

        result.ok = true;
        result.artifact.status = FakeMapArtifactStatus::Built;
        result.artifact.fault = FakeMapArtifactFault::None;
        result.artifact.metadata.map_id = map_id;
        result.artifact.metadata.scenario_name = to_string(report.scenario);
        result.artifact.metadata.created_time_s = created_time_s;
        result.artifact.metadata.accepted_update_count =
            report.backend_accepted_update_count;
        result.artifact.metadata.active_scan_command_count =
            report.active_scan_command_count;
        result.artifact.metadata.coverage_ratio =
            report.final_quality.coverage_ratio;
        result.artifact.metadata.yaw_coverage_ratio =
            report.final_quality.yaw_coverage_ratio;
        result.artifact.metadata.valid_scan_count =
            report.final_quality.valid_scan_count;
        result.artifact.final_quality = report.final_quality;
        result.artifact.serialized_text =
            serialize_metadata(result.artifact.metadata);
        result.artifact.message = "fake_map_artifact_built";
        result.passed.push_back("fake_map_artifact_built");
        result.summary = "fake_map_artifact_built";
        return result;
    }

    FakeMapStorageResult save_artifact(
        const FakeMapArtifact &artifact) {
        FakeMapStorageResult result;
        if (!save_options_.enabled) {
            return reject(
                result,
                FakeMapArtifactFault::SaveDisabled,
                "fake_map_save_disabled");
        }
        if (!valid_artifact(artifact)) {
            return reject(
                result,
                FakeMapArtifactFault::InvalidArtifact,
                "fake_map_invalid_artifact");
        }
        if (!save_options_.allow_overwrite &&
            has_artifact(artifact.metadata.map_id)) {
            return reject(
                result,
                FakeMapArtifactFault::InvalidArtifact,
                "fake_map_duplicate_artifact");
        }

        FakeMapArtifact saved = artifact;
        saved.status = FakeMapArtifactStatus::Saved;
        saved.message = "fake_map_artifact_saved";
        if (save_options_.allow_overwrite) {
            for (auto &existing : artifacts_) {
                if (existing.metadata.map_id == saved.metadata.map_id) {
                    existing = saved;
                    result.ok = true;
                    result.artifact = saved;
                    result.passed.push_back("fake_map_artifact_saved");
                    result.summary = "fake_map_artifact_saved";
                    return result;
                }
            }
        }
        artifacts_.push_back(saved);
        result.ok = true;
        result.artifact = saved;
        result.passed.push_back("fake_map_artifact_saved");
        result.summary = "fake_map_artifact_saved";
        return result;
    }

    FakeMapStorageResult load_artifact(
        const std::string &map_id) const {
        FakeMapStorageResult result;
        if (!load_options_.enabled) {
            return reject(
                result,
                FakeMapArtifactFault::LoadDisabled,
                "fake_map_load_disabled");
        }
        for (const auto &artifact : artifacts_) {
            if (artifact.metadata.map_id == map_id) {
                if (load_options_.require_valid_artifact &&
                    !valid_artifact(artifact)) {
                    return reject(
                        result,
                        FakeMapArtifactFault::InvalidArtifact,
                        "fake_map_invalid_artifact");
                }
                result.ok = true;
                result.artifact = artifact;
                result.artifact.status = FakeMapArtifactStatus::Loaded;
                result.artifact.message = "fake_map_artifact_loaded";
                result.passed.push_back("fake_map_artifact_loaded");
                result.summary = "fake_map_artifact_loaded";
                return result;
            }
        }
        return reject(
            result,
            FakeMapArtifactFault::ArtifactNotFound,
            "fake_map_artifact_not_found");
    }

    bool has_artifact(
        const std::string &map_id) const {
        for (const auto &artifact : artifacts_) {
            if (artifact.metadata.map_id == map_id) {
                return true;
            }
        }
        return false;
    }

    int artifact_count() const {
        return static_cast<int>(artifacts_.size());
    }

private:
    static FakeMapStorageResult reject(
        FakeMapStorageResult result,
        FakeMapArtifactFault fault,
        const std::string &message) {
        result.ok = false;
        result.artifact.status = FakeMapArtifactStatus::Rejected;
        result.artifact.fault = fault;
        result.artifact.message = message;
        result.failed.push_back(message);
        result.summary = message;
        return result;
    }

    static bool valid_artifact(
        const FakeMapArtifact &artifact) {
        return !artifact.metadata.map_id.empty() &&
               !artifact.serialized_text.empty() &&
               artifact.fault == FakeMapArtifactFault::None &&
               artifact.status != FakeMapArtifactStatus::Empty &&
               artifact.status != FakeMapArtifactStatus::Rejected;
    }

    static std::string serialize_metadata(
        const FakeMapMetadata &metadata) {
        std::ostringstream o;
        o << "fake_map_artifact\n"
          << "map_id=" << metadata.map_id << "\n"
          << "scenario=" << metadata.scenario_name << "\n"
          << "created_time_s=" << metadata.created_time_s << "\n"
          << "coverage=" << metadata.coverage_ratio << "\n"
          << "yaw_coverage=" << metadata.yaw_coverage_ratio << "\n"
          << "valid_scan_count=" << metadata.valid_scan_count << "\n"
          << "accepted_update_count=" << metadata.accepted_update_count << "\n"
          << "active_scan_command_count="
          << metadata.active_scan_command_count << "\n"
          << "frame_id=" << metadata.frame_id << "\n"
          << "source=" << metadata.source << "\n";
        return o.str();
    }

    FakeMapSaveOptions save_options_;
    FakeMapLoadOptions load_options_;
    std::vector<FakeMapArtifact> artifacts_;
};

} // namespace robot_slamd
