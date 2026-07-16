#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_pose_correction_state.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_keyframe_manager.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class SparseTofLocalSlamFault {
    None,
    NotInitialized,
    ExistingMapRequiresInitialPose,
    OdomPoseMissing,
    OdomPoseInvalid,
    MultiTofMissing,
    SnapshotRejected,
    KeyframeNotReady,
    BootstrapRejected,
    ReferenceMapNotReady,
    MatchRejected,
    MatchAmbiguous,
    MatchDegenerate,
    CorrectionRejected,
    MapPrepareFailed,
    MapCapacityReached,
    MapCommitFailed,
    InternalInvariantViolation
};

inline std::string to_string(SparseTofLocalSlamFault fault) {
    switch (fault) {
    case SparseTofLocalSlamFault::None: return "none";
    case SparseTofLocalSlamFault::NotInitialized: return "not_initialized";
    case SparseTofLocalSlamFault::ExistingMapRequiresInitialPose: return "existing_map_requires_initial_pose";
    case SparseTofLocalSlamFault::OdomPoseMissing: return "odom_pose_missing";
    case SparseTofLocalSlamFault::OdomPoseInvalid: return "odom_pose_invalid";
    case SparseTofLocalSlamFault::MultiTofMissing: return "multi_tof_missing";
    case SparseTofLocalSlamFault::SnapshotRejected: return "snapshot_rejected";
    case SparseTofLocalSlamFault::KeyframeNotReady: return "keyframe_not_ready";
    case SparseTofLocalSlamFault::BootstrapRejected: return "bootstrap_rejected";
    case SparseTofLocalSlamFault::ReferenceMapNotReady: return "reference_map_not_ready";
    case SparseTofLocalSlamFault::MatchRejected: return "match_rejected";
    case SparseTofLocalSlamFault::MatchAmbiguous: return "match_ambiguous";
    case SparseTofLocalSlamFault::MatchDegenerate: return "match_degenerate";
    case SparseTofLocalSlamFault::CorrectionRejected: return "correction_rejected";
    case SparseTofLocalSlamFault::MapPrepareFailed: return "map_prepare_failed";
    case SparseTofLocalSlamFault::MapCapacityReached: return "map_capacity_reached";
    case SparseTofLocalSlamFault::MapCommitFailed: return "map_commit_failed";
    case SparseTofLocalSlamFault::InternalInvariantViolation: return "internal_invariant_violation";
    }
    return "unknown";
}

struct SparseTofLocalSlamConfig {
    SparseOccupancyGridConfig grid;
    SparseTofPoseCorrectionConfig correction;
    SparseTofKeyframeConfig keyframe;
    SparseTofLocalScanMatcherConfig matcher;
};

struct SparseTofLocalSlamInput {
    double now_s = 0.0;
    RobotPose2D odom_pose;
    bool has_odom_pose = false;
    SparseTofMatchObservation observation;
    bool has_multi_tof = false;
    bool motion_active = false;
    bool motion_settled = false;
    bool mapping_publish_allowed = false;
};

struct SparseTofLocalSlamReport {
    bool ok = false;
    InitialPoseMode initial_pose_mode = InitialPoseMode::StartupZero;
    bool initial_yaw_explicitly_defined = true;
    bool initial_yaw_measured_by_imu = false;
    bool existing_map_initial_pose_required = false;
    std::size_t matcher_attempt_count = 0;
    std::size_t matcher_accept_count = 0;
    std::size_t matcher_reject_count = 0;
    std::size_t full_se2_correction_count = 0;
    std::size_t yaw_only_correction_count = 0;
    std::size_t bootstrap_keyframe_count = 0;
    std::size_t matched_keyframe_count = 0;
    std::size_t rejected_keyframe_count = 0;
    std::size_t committed_keyframe_count = 0;
    std::size_t self_match_prevention_count = 0;
    std::size_t map_transaction_rejection_count = 0;
    std::size_t total_candidate_count = 0;
    std::size_t maximum_candidates_in_one_match = 0;
    double last_best_score = 0.0;
    double last_runner_up_score = 0.0;
    double last_score_margin = 0.0;
    double last_known_cell_ratio = 0.0;
    double last_correction_dx_m = 0.0;
    double last_correction_dy_m = 0.0;
    double last_correction_dyaw_rad = 0.0;
    RobotPose2D last_odom_pose;
    RobotPose2D last_corrected_map_pose;
    bool native_multi_tof_consumed = false;
    bool legacy_projection_consumed = false;
    bool matcher_used_ground_truth = false;
    bool matcher_used_fake_world = false;
    bool command_derived_pose_used = false;
    bool estimator_pose_writeback_used = false;
    bool reference_map_frozen_during_match = false;
    bool self_matching_prevented = false;
    bool batch_map_transaction_verified = false;
    bool deterministic_output_verified = false;
    bool bounded_search_verified = false;
    bool phase_aware_publish_verified = false;
    bool real_hardware_accessed = false;
    bool real_motion_attempted = false;
    bool real_map_write_attempted = false;
    bool real_pose_writeback_attempted = false;
    SparseTofLocalSlamFault last_fault = SparseTofLocalSlamFault::None;
    std::string last_message;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

struct SparseTofLocalSlamOutput {
    bool ok = false;
    RobotPose2D odom_pose;
    RobotPose2D corrected_map_pose;
    bool correction_applied = false;
    SparseTofCorrectionMode correction_mode = SparseTofCorrectionMode::Rejected;
    bool keyframe_bootstrapped = false;
    bool keyframe_matched = false;
    bool keyframe_committed = false;
    bool map_updated = false;
    SparseTofLocalSlamFault fault = SparseTofLocalSlamFault::None;
    std::string message;
};

class SparseTofLocalSlamPipeline {
public:
    explicit SparseTofLocalSlamPipeline(SparseTofLocalSlamConfig config = {})
        : config_(config), grid_(config.grid), correction_(config.correction),
          keyframes_(config.keyframe), matcher_(config.matcher) {}

    SparseTofLocalSlamOutput reset_startup_zero() {
        SparseTofLocalSlamOutput out;
        auto reset = correction_.reset_new_map();
        initialized_ = reset.ok;
        report_.initial_pose_mode = InitialPoseMode::StartupZero;
        report_.initial_yaw_explicitly_defined = true;
        report_.initial_yaw_measured_by_imu = false;
        out.ok = reset.ok;
        out.fault = reset.ok ? SparseTofLocalSlamFault::None : SparseTofLocalSlamFault::NotInitialized;
        out.message = reset.message;
        return out;
    }

    SparseTofLocalSlamOutput reset_configured_pose(const RobotPose2D &map_pose) {
        SparseTofLocalSlamOutput out;
        auto reset = correction_.reset_configured_pose(map_pose);
        initialized_ = reset.ok;
        report_.initial_pose_mode = InitialPoseMode::ConfiguredPose;
        report_.initial_yaw_explicitly_defined = reset.ok;
        report_.initial_yaw_measured_by_imu = false;
        out.ok = reset.ok;
        out.fault = reset.ok ? SparseTofLocalSlamFault::None : SparseTofLocalSlamFault::NotInitialized;
        out.message = reset.message;
        return out;
    }

    SparseTofLocalSlamOutput reset_existing_map_without_pose() {
        SparseTofLocalSlamOutput out;
        report_.existing_map_initial_pose_required = true;
        out.ok = false;
        out.fault = SparseTofLocalSlamFault::ExistingMapRequiresInitialPose;
        out.message = "existing_map_requires_initial_pose";
        return out;
    }

    SparseTofLocalSlamOutput process(const SparseTofLocalSlamInput &input) {
        SparseTofLocalSlamOutput out;
        if (!initialized_) return finish(out, SparseTofLocalSlamFault::NotInitialized, "local_slam_not_initialized");
        if (!input.has_odom_pose) return finish(out, SparseTofLocalSlamFault::OdomPoseMissing, "local_slam_odom_missing");
        if (!se2_pose_finite(input.odom_pose)) return finish(out, SparseTofLocalSlamFault::OdomPoseInvalid, "local_slam_odom_invalid");
        out.odom_pose = input.odom_pose;
        out.corrected_map_pose = correction_.corrected_pose(input.odom_pose);
        report_.last_odom_pose = out.odom_pose;
        report_.last_corrected_map_pose = out.corrected_map_pose;

        if (input.motion_active || !input.motion_settled || !input.mapping_publish_allowed) {
            report_.phase_aware_publish_verified = true;
            out.ok = true;
            out.message = "local_slam_phase_suppressed";
            report_.ok = true;
            return out;
        }
        if (!input.has_multi_tof) {
            return finish(out, SparseTofLocalSlamFault::MultiTofMissing, "local_slam_multi_tof_missing");
        }

        SparseTofMatchObservation observation = input.observation;
        observation.odom_pose = input.odom_pose;
        if (observation.ray_count == 0) {
            return finish(out, SparseTofLocalSlamFault::SnapshotRejected, "local_slam_snapshot_empty");
        }
        const auto add = keyframes_.add_observation(observation, input.motion_settled);
        if (!add.ok) return finish(out, SparseTofLocalSlamFault::SnapshotRejected, add.message);
        report_.native_multi_tof_consumed = true;

        if (grid_.active_cell_count() == 0) {
            if (!keyframes_.ready_for_bootstrap(true, input.motion_settled)) {
                out.ok = true;
                out.message = "local_slam_keyframe_accumulating";
                report_.ok = true;
                return out;
            }
            auto boot = keyframes_.commit_bootstrap(grid_, correction_.map_from_odom());
            if (!boot.ok) return map_failure(out, boot);
            out.keyframe_bootstrapped = true;
            out.keyframe_committed = true;
            out.map_updated = true;
            out.corrected_map_pose = correction_.corrected_pose(input.odom_pose);
            report_.bootstrap_keyframe_count++;
            report_.committed_keyframe_count++;
            report_.batch_map_transaction_verified = true;
            report_.self_matching_prevented = true;
            report_.self_match_prevention_count++;
            out.ok = true;
            out.message = boot.message;
            report_.ok = true;
            return out;
        }

        if (!keyframes_.ready_for_match(false, input.motion_settled)) {
            out.ok = true;
            out.message = "local_slam_keyframe_accumulating";
            report_.ok = true;
            return out;
        }

        const auto frozen = grid_.snapshot();
        report_.reference_map_frozen_during_match = true;
        report_.matcher_attempt_count++;
        auto match = matcher_.match(frozen, keyframes_.active_bundle(), correction_.map_from_odom());
        report_.total_candidate_count += match.candidate_count;
        if (match.candidate_count > report_.maximum_candidates_in_one_match) report_.maximum_candidates_in_one_match = match.candidate_count;
        report_.last_best_score = match.best.normalized_score;
        report_.last_runner_up_score = match.runner_up.normalized_score;
        report_.last_score_margin = match.score_margin;
        report_.last_known_cell_ratio = match.best.evaluated_cell_count == 0 ? 0.0 :
            static_cast<double>(match.best.known_cell_count) / static_cast<double>(match.best.evaluated_cell_count);
        report_.bounded_search_verified = match.bounded_search_verified;
        if (!match.accepted) {
            keyframes_.mark_match_rejected();
            report_.matcher_reject_count++;
            report_.rejected_keyframe_count++;
            return finish(out, SparseTofLocalSlamFault::MatchRejected, match.message);
        }

        const RobotPose2D matched_map_pose = se2_compose(match.best.map_from_odom, input.odom_pose);
        auto prepared = correction_.prepare_correction(input.odom_pose, matched_map_pose);
        if (!prepared.ok) return finish(out, SparseTofLocalSlamFault::CorrectionRejected, prepared.message);
        auto committed = keyframes_.commit_matched(grid_, match.best.map_from_odom, match);
        if (!committed.ok) {
            correction_.abort_prepared_correction();
            return map_failure(out, committed);
        }
        auto correction_commit = correction_.commit_prepared_correction();
        if (!correction_commit.ok) return finish(out, SparseTofLocalSlamFault::InternalInvariantViolation, correction_commit.message);

        report_.matcher_accept_count++;
        report_.matched_keyframe_count++;
        report_.committed_keyframe_count++;
        report_.batch_map_transaction_verified = true;
        report_.last_correction_dx_m = match.best.correction.x_m;
        report_.last_correction_dy_m = match.best.correction.y_m;
        report_.last_correction_dyaw_rad = match.best.correction.yaw_rad;
        if (match.correction_mode == SparseTofCorrectionMode::FullSE2) report_.full_se2_correction_count++;
        if (match.correction_mode == SparseTofCorrectionMode::YawOnly) report_.yaw_only_correction_count++;
        out.correction_applied = true;
        out.correction_mode = match.correction_mode;
        out.keyframe_matched = true;
        out.keyframe_committed = true;
        out.map_updated = true;
        out.corrected_map_pose = correction_.corrected_pose(input.odom_pose);
        out.ok = true;
        out.message = committed.message;
        report_.last_corrected_map_pose = out.corrected_map_pose;
        report_.ok = true;
        return out;
    }

    const SparseOccupancyGrid &grid() const { return grid_; }
    const SparseTofPoseCorrectionState &correction_state() const { return correction_; }
    const SparseTofKeyframeManager &keyframes() const { return keyframes_; }
    const SparseTofLocalSlamReport &report() const { return report_; }

private:
    SparseTofLocalSlamConfig config_;
    SparseOccupancyGrid grid_;
    SparseTofPoseCorrectionState correction_;
    SparseTofKeyframeManager keyframes_;
    SparseTofLocalScanMatcher matcher_;
    SparseTofLocalSlamReport report_;
    bool initialized_ = false;

    SparseTofLocalSlamOutput finish(SparseTofLocalSlamOutput out,
                                    SparseTofLocalSlamFault fault,
                                    const std::string &message) {
        out.ok = fault == SparseTofLocalSlamFault::None;
        out.fault = fault;
        out.message = message;
        report_.last_fault = fault;
        report_.last_message = message;
        report_.ok = out.ok;
        return out;
    }

    SparseTofLocalSlamOutput map_failure(SparseTofLocalSlamOutput out,
                                         const SparseTofKeyframeResult &result) {
        report_.map_transaction_rejection_count++;
        SparseTofLocalSlamFault fault = SparseTofLocalSlamFault::MapCommitFailed;
        if (result.map_stats.fault == SparseOccupancyGridFault::MapCapacityReached) {
            fault = SparseTofLocalSlamFault::MapCapacityReached;
        }
        return finish(out, fault, result.message);
    }
};

} // namespace robot_slamd
