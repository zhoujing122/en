#pragma once

#include "robot_slamd/mapping/sparse_tof/sparse_grid_types.hpp"
#include "robot_slamd/runtime/active_multi_tof_observation_types.hpp"
#include "robot_slamd/runtime/sparse_slam_frames.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

enum class SparseTofLocalMatchMode {
    YawOnly,
    FullSE2
};

inline std::string to_string(SparseTofLocalMatchMode mode) {
    switch (mode) {
    case SparseTofLocalMatchMode::YawOnly:
        return "yaw_only";
    case SparseTofLocalMatchMode::FullSE2:
        return "full_se2";
    }
    return "unknown";
}

inline bool parse_sparse_tof_local_match_mode(
    const std::string &text,
    SparseTofLocalMatchMode &mode) {
    if (text == "yaw_only") {
        mode = SparseTofLocalMatchMode::YawOnly;
        return true;
    }
    if (text == "full_se2") {
        mode = SparseTofLocalMatchMode::FullSE2;
        return true;
    }
    return false;
}

struct SparseTofLocalMatchConfig {
    SparseTofLocalMatchMode mode = SparseTofLocalMatchMode::YawOnly;
    double max_abs_yaw_rad = 0.2617993877991494;
    double coarse_yaw_step_rad = 0.03490658503988659;
    double fine_yaw_step_rad = 0.008726646259971648;
    double max_abs_translation_x_m = 0.0;
    double max_abs_translation_y_m = 0.0;
    std::size_t max_candidate_count = 256;
    std::size_t max_coarse_candidates = 64;
    std::size_t max_fine_candidates = 64;
    std::size_t max_total_candidates = 128;
    std::size_t max_scored_rays = 96;
    std::size_t max_cells_per_ray = 256;
    std::size_t min_bundle_frames = 2;
    std::size_t min_matchable_rays = 3;
    std::size_t min_reference_cells = 1;
    std::size_t min_reference_occupied_cells = 1;
    std::size_t min_reference_free_cells = 0;
    double max_bundle_duration_s = 30.0;
    double no_return_free_space_range_m = 12.0;
    std::size_t min_used_rays = 3;
    std::size_t min_known_cells = 6;
    double max_unknown_ratio = 0.85;
    double max_contradiction_ratio = 0.60;
    double minimum_normalized_score = 0.02;
    double minimum_score_margin = 0.03;
    double minimum_score_range = 0.02;
    double runner_up_exclusion_yaw_rad = 0.03490658503988659;
    double multimodal_max_score_drop = 0.04;
    double free_agreement_weight = 1.0;
    double free_contradiction_weight = -3.0;
    double occupied_endpoint_agreement_weight = 4.0;
    double occupied_endpoint_contradiction_weight = -4.0;
    bool require_revision_match = true;
    bool require_immutable_snapshot = true;
};

enum class SparseTofLocalMatchInputStatus {
    Ready,
    BundleMissing,
    BundleNotFrozen,
    BundleInvalid,
    BundleInsufficient,
    ReferenceMapMissing,
    ReferenceMapInvalid,
    ReferenceMapTooSparse,
    ReferenceRevisionMismatch,
    PredictionInvalid,
    TimestampInvalid,
    ContractViolation,
    Fault
};

inline std::string to_string(SparseTofLocalMatchInputStatus status) {
    switch (status) {
    case SparseTofLocalMatchInputStatus::Ready: return "ready";
    case SparseTofLocalMatchInputStatus::BundleMissing: return "bundle_missing";
    case SparseTofLocalMatchInputStatus::BundleNotFrozen: return "bundle_not_frozen";
    case SparseTofLocalMatchInputStatus::BundleInvalid: return "bundle_invalid";
    case SparseTofLocalMatchInputStatus::BundleInsufficient: return "bundle_insufficient";
    case SparseTofLocalMatchInputStatus::ReferenceMapMissing: return "reference_map_missing";
    case SparseTofLocalMatchInputStatus::ReferenceMapInvalid: return "reference_map_invalid";
    case SparseTofLocalMatchInputStatus::ReferenceMapTooSparse: return "reference_map_too_sparse";
    case SparseTofLocalMatchInputStatus::ReferenceRevisionMismatch: return "reference_revision_mismatch";
    case SparseTofLocalMatchInputStatus::PredictionInvalid: return "prediction_invalid";
    case SparseTofLocalMatchInputStatus::TimestampInvalid: return "timestamp_invalid";
    case SparseTofLocalMatchInputStatus::ContractViolation: return "contract_violation";
    case SparseTofLocalMatchInputStatus::Fault: return "fault";
    }
    return "unknown";
}

struct SparseTofLocalMatchInput {
    std::uint64_t bundle_id = 0;
    std::shared_ptr<const FrozenMultiTofObservationBundle> frozen_bundle;
    std::shared_ptr<const SparseOccupancyGridSnapshot> reference_map;
    std::uint64_t expected_reference_map_revision = 0;
    std::uint64_t current_runtime_map_revision = 0;
    MapFromOdom2D predicted_map_from_odom = identity_map_from_odom();
    MapFromOdom2D map_from_odom_at_collection_start = identity_map_from_odom();
    double match_request_timestamp_s = 0.0;
    std::string source;
    SparseTofLocalMatchConfig config;
};

struct SparseTofLocalMatchInputValidationResult {
    bool ready = false;
    SparseTofLocalMatchInputStatus status =
        SparseTofLocalMatchInputStatus::ContractViolation;
    std::string reason;
};

enum class SparseTofLocalMatchStatus {
    NotRun,
    InputRejected,
    NotImplemented,
    Rejected,
    RejectedInvalidInput,
    RejectedInsufficientInformation,
    RejectedUnknownDominated,
    RejectedContradictory,
    RejectedNoValidCandidate,
    RejectedCandidateBudget,
    RejectedBestAtSearchEdge,
    RejectedLowScore,
    RejectedLowMargin,
    RejectedFlatCurve,
    RejectedMultimodal,
    AcceptedYawOnly,
    AcceptedFullSE2,
    Fault
};

inline std::string to_string(SparseTofLocalMatchStatus status) {
    switch (status) {
    case SparseTofLocalMatchStatus::NotRun: return "not_run";
    case SparseTofLocalMatchStatus::InputRejected: return "input_rejected";
    case SparseTofLocalMatchStatus::NotImplemented: return "not_implemented";
    case SparseTofLocalMatchStatus::Rejected: return "rejected";
    case SparseTofLocalMatchStatus::RejectedInvalidInput: return "rejected_invalid_input";
    case SparseTofLocalMatchStatus::RejectedInsufficientInformation: return "rejected_insufficient_information";
    case SparseTofLocalMatchStatus::RejectedUnknownDominated: return "rejected_unknown_dominated";
    case SparseTofLocalMatchStatus::RejectedContradictory: return "rejected_contradictory";
    case SparseTofLocalMatchStatus::RejectedNoValidCandidate: return "rejected_no_valid_candidate";
    case SparseTofLocalMatchStatus::RejectedCandidateBudget: return "rejected_candidate_budget";
    case SparseTofLocalMatchStatus::RejectedBestAtSearchEdge: return "rejected_best_at_search_edge";
    case SparseTofLocalMatchStatus::RejectedLowScore: return "rejected_low_score";
    case SparseTofLocalMatchStatus::RejectedLowMargin: return "rejected_low_margin";
    case SparseTofLocalMatchStatus::RejectedFlatCurve: return "rejected_flat_curve";
    case SparseTofLocalMatchStatus::RejectedMultimodal: return "rejected_multimodal";
    case SparseTofLocalMatchStatus::AcceptedYawOnly: return "accepted_yaw_only";
    case SparseTofLocalMatchStatus::AcceptedFullSE2: return "accepted_full_se2";
    case SparseTofLocalMatchStatus::Fault: return "fault";
    }
    return "unknown";
}

struct SparseTofLocalMatchCandidateMetrics {
    double delta_yaw_rad = 0.0;
    double candidate_yaw_rad = 0.0;
    std::size_t used_ray_count = 0;
    std::size_t hit_ray_count = 0;
    std::size_t no_return_ray_count = 0;
    std::size_t ignored_ray_count = 0;
    std::size_t traversed_cell_count = 0;
    std::size_t known_cell_count = 0;
    std::size_t unknown_cell_count = 0;
    std::size_t free_agreement_count = 0;
    std::size_t occupied_endpoint_agreement_count = 0;
    std::size_t contradiction_count = 0;
    double unknown_ratio = 1.0;
    double contradiction_ratio = 1.0;
    double raw_score = 0.0;
    double normalized_score = 0.0;
    bool valid = false;
};

struct SparseTofLocalMatchResult {
    SparseTofLocalMatchStatus status = SparseTofLocalMatchStatus::NotRun;
    std::uint64_t bundle_id = 0;
    std::uint64_t reference_map_revision = 0;
    bool matcher_executed = false;
    bool accepted = false;
    SparseTofLocalMatchMode mode = SparseTofLocalMatchMode::YawOnly;
    MapFromOdom2D predicted_map_from_odom = identity_map_from_odom();
    std::optional<MapFromOdom2D> proposed_map_from_odom;
    std::optional<RobotPose2D> correction_delta;
    std::optional<double> best_score;
    std::optional<double> second_best_score;
    std::optional<double> score_margin;
    std::size_t evaluated_candidate_count = 0;
    std::size_t coarse_candidate_count = 0;
    std::size_t fine_candidate_count = 0;
    SparseTofLocalMatchCandidateMetrics best_metrics;
    bool runner_up_available = false;
    std::optional<double> best_delta_yaw_rad;
    std::optional<double> best_yaw_rad;
    std::optional<double> runner_up_yaw_rad;
    std::optional<double> runner_up_yaw_separation_rad;
    double score_min = 0.0;
    double score_max = 0.0;
    double score_mean = 0.0;
    double score_range = 0.0;
    bool best_at_search_edge = false;
    bool flat_curve = false;
    bool multimodal = false;
    bool proposal_ready = false;
    std::vector<SparseTofLocalMatchCandidateMetrics> candidate_metrics;
    std::string reason = "local_match_not_run";
};

class PreparedSparseTofLocalMatchInput {
public:
    PreparedSparseTofLocalMatchInput() = default;

    static PreparedSparseTofLocalMatchInput ready(
        SparseTofLocalMatchInput input) {
        PreparedSparseTofLocalMatchInput out;
        out.ready_ = true;
        out.status_ = SparseTofLocalMatchInputStatus::Ready;
        out.reason_ = "local_match_input_ready";
        out.input_ = std::make_shared<const SparseTofLocalMatchInput>(
            std::move(input));
        return out;
    }

    static PreparedSparseTofLocalMatchInput rejected(
        SparseTofLocalMatchInputStatus status,
        std::string reason) {
        PreparedSparseTofLocalMatchInput out;
        out.status_ = status;
        out.reason_ = std::move(reason);
        return out;
    }

    bool is_ready() const { return ready_; }
    SparseTofLocalMatchInputStatus status() const { return status_; }
    const std::string &reason() const { return reason_; }
    const SparseTofLocalMatchInput *input() const { return input_.get(); }

private:
    bool ready_ = false;
    SparseTofLocalMatchInputStatus status_ =
        SparseTofLocalMatchInputStatus::ContractViolation;
    std::string reason_ = "local_match_input_not_prepared";
    std::shared_ptr<const SparseTofLocalMatchInput> input_;
};

} // namespace robot_slamd
