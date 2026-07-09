#pragma once

#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_types.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class FakeRelocalizationReadinessStatus {
    NotChecked,
    Ready,
    Blocked
};

enum class FakeRelocalizationReadinessBlockReason {
    None,
    BindingDisabled,
    PoseWritebackAllowed,
    MapArtifactMissing,
    MapQualityTooLow,
    YawCoverageTooLow,
    RelocalizationRejected,
    PoseCandidateInvalid,
    PoseWritebackAttempted
};

struct FakeRelocalizationReadinessOptions {
    bool enabled = false;
    bool require_binding_ready = true;
    bool require_no_pose_writeback = true;
    bool require_map_quality_good = true;
    double min_confidence = 0.70;
    double min_map_coverage_ratio = 0.60;
    double min_map_yaw_coverage_ratio = 0.30;
};

struct FakeRelocalizationReadinessResult {
    bool ok = false;
    FakeRelocalizationReadinessStatus status =
        FakeRelocalizationReadinessStatus::NotChecked;
    FakeRelocalizationReadinessBlockReason block_reason =
        FakeRelocalizationReadinessBlockReason::None;
    FakeRelocalizationResult relocalization_result;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(
    FakeRelocalizationReadinessStatus status) {
    switch (status) {
    case FakeRelocalizationReadinessStatus::NotChecked:
        return "not_checked";
    case FakeRelocalizationReadinessStatus::Ready:
        return "ready";
    case FakeRelocalizationReadinessStatus::Blocked:
        return "blocked";
    }
    return "unknown";
}

inline std::string to_string(
    FakeRelocalizationReadinessBlockReason reason) {
    switch (reason) {
    case FakeRelocalizationReadinessBlockReason::None:
        return "none";
    case FakeRelocalizationReadinessBlockReason::BindingDisabled:
        return "binding_disabled";
    case FakeRelocalizationReadinessBlockReason::PoseWritebackAllowed:
        return "pose_writeback_allowed";
    case FakeRelocalizationReadinessBlockReason::MapArtifactMissing:
        return "map_artifact_missing";
    case FakeRelocalizationReadinessBlockReason::MapQualityTooLow:
        return "map_quality_too_low";
    case FakeRelocalizationReadinessBlockReason::YawCoverageTooLow:
        return "yaw_coverage_too_low";
    case FakeRelocalizationReadinessBlockReason::RelocalizationRejected:
        return "relocalization_rejected";
    case FakeRelocalizationReadinessBlockReason::PoseCandidateInvalid:
        return "pose_candidate_invalid";
    case FakeRelocalizationReadinessBlockReason::PoseWritebackAttempted:
        return "pose_writeback_attempted";
    }
    return "unknown";
}

inline int fake_relocalization_readiness_status_id(
    FakeRelocalizationReadinessStatus status) {
    return static_cast<int>(status);
}

inline int fake_relocalization_readiness_block_reason_id(
    FakeRelocalizationReadinessBlockReason reason) {
    return static_cast<int>(reason);
}

class FakeRelocalizationReadinessGate {
public:
    explicit FakeRelocalizationReadinessGate(
        FakeRelocalizationReadinessOptions options = {})
        : options_(options) {}

    FakeRelocalizationReadinessResult check(
        const FakeRelocalizationResult &result,
        bool pose_writeback_attempted) const {
        FakeRelocalizationReadinessResult readiness;
        readiness.relocalization_result = result;

        if (!options_.enabled) {
            return block(
                readiness,
                FakeRelocalizationReadinessBlockReason::BindingDisabled,
                "fake_relocalization_readiness_disabled");
        }
        if (options_.require_no_pose_writeback && pose_writeback_attempted) {
            return block(
                readiness,
                FakeRelocalizationReadinessBlockReason::PoseWritebackAttempted,
                "fake_relocalization_readiness_pose_writeback_attempted");
        }
        if (!result.ok) {
            return block(
                readiness,
                FakeRelocalizationReadinessBlockReason::RelocalizationRejected,
                "fake_relocalization_readiness_rejected");
        }
        if (!result.pose.valid) {
            return block(
                readiness,
                FakeRelocalizationReadinessBlockReason::PoseCandidateInvalid,
                "fake_relocalization_readiness_pose_invalid");
        }
        if (result.pose.confidence < options_.min_confidence) {
            return block(
                readiness,
                FakeRelocalizationReadinessBlockReason::RelocalizationRejected,
                "fake_relocalization_readiness_confidence_low");
        }
        if (options_.require_map_quality_good &&
            result.map_artifact.final_quality.coverage_ratio <
                options_.min_map_coverage_ratio) {
            return block(
                readiness,
                FakeRelocalizationReadinessBlockReason::MapQualityTooLow,
                "fake_relocalization_readiness_map_coverage_low");
        }
        if (options_.require_map_quality_good &&
            result.map_artifact.final_quality.yaw_coverage_ratio <
                options_.min_map_yaw_coverage_ratio) {
            return block(
                readiness,
                FakeRelocalizationReadinessBlockReason::YawCoverageTooLow,
                "fake_relocalization_readiness_yaw_coverage_low");
        }

        readiness.ok = true;
        readiness.status = FakeRelocalizationReadinessStatus::Ready;
        readiness.block_reason = FakeRelocalizationReadinessBlockReason::None;
        readiness.passed.push_back("fake_relocalization_readiness_ready");
        readiness.passed.push_back("fake_relocalization_readiness_no_pose_writeback");
        readiness.summary = "fake_relocalization_readiness_ready";
        return readiness;
    }

private:
    static FakeRelocalizationReadinessResult block(
        FakeRelocalizationReadinessResult result,
        FakeRelocalizationReadinessBlockReason reason,
        const std::string &summary) {
        result.ok = false;
        result.status = FakeRelocalizationReadinessStatus::Blocked;
        result.block_reason = reason;
        result.failed.push_back(summary);
        result.summary = summary;
        return result;
    }

    FakeRelocalizationReadinessOptions options_;
};

} // namespace robot_slamd
