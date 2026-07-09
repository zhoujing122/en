#pragma once

#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_types.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace robot_slamd {

class FakeRelocalizationBinding {
public:
    explicit FakeRelocalizationBinding(
        FakeRelocalizationOptions options = {})
        : options_(options) {}

    bool ready() const {
        return options_.enabled;
    }

    FakeRelocalizationResult relocalize(
        const FakeMapArtifact &map_artifact,
        const RobotSlamSensorSnapshot &snapshot) const {
        FakeRelocalizationResult result;
        result.map_artifact = map_artifact;
        result.input_snapshot = snapshot;

        if (!ready()) {
            return reject(
                result,
                FakeRelocalizationFault::Disabled,
                "fake_relocalization_disabled");
        }
        if (options_.allow_pose_writeback) {
            return reject(
                result,
                FakeRelocalizationFault::PoseWritebackForbidden,
                "fake_relocalization_pose_writeback_forbidden");
        }
        if (!valid_map_artifact(map_artifact)) {
            return reject(
                result,
                FakeRelocalizationFault::InvalidMapArtifact,
                "fake_relocalization_invalid_map_artifact");
        }
        result.status = FakeRelocalizationStatus::MapLoaded;
        result.passed.push_back("fake_relocalization_map_loaded");

        if (options_.require_map_quality_good &&
            (!map_artifact.final_quality.good_enough ||
             map_artifact.final_quality.coverage_ratio < options_.min_map_coverage_ratio ||
             map_artifact.final_quality.yaw_coverage_ratio < options_.min_map_yaw_coverage_ratio)) {
            return reject(
                result,
                FakeRelocalizationFault::MapQualityTooLow,
                "fake_relocalization_map_quality_too_low");
        }
        if (options_.require_tof && !snapshot.has_tof) {
            return reject(
                result,
                FakeRelocalizationFault::MissingTof,
                "fake_relocalization_missing_tof");
        }
        if (snapshot.tof.ranges_m.empty()) {
            return reject(
                result,
                FakeRelocalizationFault::EmptyTofRanges,
                "fake_relocalization_empty_tof_ranges");
        }

        result.status = FakeRelocalizationStatus::EvaluatingScan;
        result.valid_range_count = count_valid_tof_ranges(snapshot);
        result.valid_range_ratio = static_cast<double>(result.valid_range_count) /
                                   static_cast<double>(snapshot.tof.ranges_m.size());
        if (result.valid_range_count < options_.min_valid_ranges) {
            return reject(
                result,
                FakeRelocalizationFault::TooFewValidRanges,
                "fake_relocalization_too_few_valid_ranges");
        }

        const double confidence = compute_confidence(
            map_artifact,
            snapshot,
            result.valid_range_count);
        if (confidence < options_.min_confidence) {
            result.pose.confidence = confidence;
            result.confidence_band = FakeRelocalizationConfidenceBand::Low;
            return reject(
                result,
                FakeRelocalizationFault::ConfidenceTooLow,
                "fake_relocalization_confidence_too_low");
        }

        result.ok = true;
        result.status = FakeRelocalizationStatus::Accepted;
        result.fault = FakeRelocalizationFault::None;
        result.pose = make_pose_candidate(map_artifact, confidence);
        result.confidence_band = confidence >= options_.high_confidence_threshold
                                     ? FakeRelocalizationConfidenceBand::High
                                     : FakeRelocalizationConfidenceBand::Medium;
        result.passed.push_back("fake_relocalization_scan_accepted");
        result.passed.push_back("fake_relocalization_no_pose_writeback");
        result.summary = "fake_relocalization_accepted";
        return result;
    }

    std::string status() const {
        std::ostringstream o;
        o << "fake_relocalization_ready=" << (ready() ? "true" : "false")
          << ",allow_pose_writeback=" << (options_.allow_pose_writeback ? "true" : "false")
          << ",min_confidence=" << options_.min_confidence;
        return o.str();
    }

private:
    int count_valid_tof_ranges(
        const RobotSlamSensorSnapshot &snapshot) const {
        int count = 0;
        for (const auto range_m : snapshot.tof.ranges_m) {
            if (std::isfinite(range_m) && range_m > 0.0) {
                count++;
            }
        }
        return count;
    }

    double compute_confidence(
        const FakeMapArtifact &map_artifact,
        const RobotSlamSensorSnapshot &snapshot,
        int valid_range_count) const {
        const double total = static_cast<double>(snapshot.tof.ranges_m.size());
        const double valid_ratio = total > 0.0
                                       ? static_cast<double>(valid_range_count) / total
                                       : 0.0;
        const double coverage = clamp01(map_artifact.final_quality.coverage_ratio);
        const double yaw_coverage = clamp01(map_artifact.final_quality.yaw_coverage_ratio);
        return clamp01(0.50 * coverage + 0.25 * yaw_coverage + 0.25 * valid_ratio);
    }

    FakePoseCandidate make_pose_candidate(
        const FakeMapArtifact &map_artifact,
        double confidence) const {
        FakePoseCandidate pose;
        pose.valid = true;
        pose.x_m = map_artifact.metadata.coverage_ratio;
        pose.y_m = map_artifact.metadata.yaw_coverage_ratio;
        pose.yaw_rad = 0.01 * static_cast<double>(
            map_artifact.metadata.valid_scan_count +
            map_artifact.metadata.active_scan_command_count);
        pose.confidence = confidence;
        pose.frame_id = map_artifact.metadata.frame_id.empty()
                            ? "fake_map"
                            : map_artifact.metadata.frame_id;
        pose.source = "fake_relocalization_report_only";
        return pose;
    }

    static bool valid_map_artifact(
        const FakeMapArtifact &map_artifact) {
        const bool status_ok =
            map_artifact.status == FakeMapArtifactStatus::Built ||
            map_artifact.status == FakeMapArtifactStatus::Saved ||
            map_artifact.status == FakeMapArtifactStatus::Loaded;
        return status_ok &&
               !map_artifact.metadata.map_id.empty() &&
               !map_artifact.serialized_text.empty() &&
               map_artifact.fault == FakeMapArtifactFault::None;
    }

    static double clamp01(double value) {
        return std::max(0.0, std::min(1.0, value));
    }

    static FakeRelocalizationResult reject(
        FakeRelocalizationResult result,
        FakeRelocalizationFault fault,
        const std::string &summary) {
        result.ok = false;
        result.status = fault == FakeRelocalizationFault::Disabled ||
                                fault == FakeRelocalizationFault::PoseWritebackForbidden
                            ? FakeRelocalizationStatus::Fault
                            : FakeRelocalizationStatus::Rejected;
        result.fault = fault;
        result.pose.valid = false;
        result.failed.push_back(summary);
        result.summary = summary;
        return result;
    }

    FakeRelocalizationOptions options_;
};

} // namespace robot_slamd
