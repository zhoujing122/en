#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_scan_matcher.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_slamd {

enum class SparseTofKeyframeState {
    Accumulating,
    ReadyForBootstrap,
    ReadyForMatch,
    MatchAccepted,
    MatchRejected,
    MapCommitted,
    Discarded
};

enum class SparseTofKeyframeFault {
    None,
    InvalidConfig,
    InvalidObservation,
    DuplicateTimestamp,
    NonMonotonicTimestamp,
    BundleFull,
    BundleNotReady,
    BootstrapRejected,
    ReferenceMapNotReady,
    MapUpdateRejected
};

inline std::string to_string(SparseTofKeyframeState state) {
    switch (state) {
    case SparseTofKeyframeState::Accumulating: return "accumulating";
    case SparseTofKeyframeState::ReadyForBootstrap: return "ready_for_bootstrap";
    case SparseTofKeyframeState::ReadyForMatch: return "ready_for_match";
    case SparseTofKeyframeState::MatchAccepted: return "match_accepted";
    case SparseTofKeyframeState::MatchRejected: return "match_rejected";
    case SparseTofKeyframeState::MapCommitted: return "map_committed";
    case SparseTofKeyframeState::Discarded: return "discarded";
    }
    return "unknown";
}

inline std::string to_string(SparseTofKeyframeFault fault) {
    switch (fault) {
    case SparseTofKeyframeFault::None: return "none";
    case SparseTofKeyframeFault::InvalidConfig: return "invalid_config";
    case SparseTofKeyframeFault::InvalidObservation: return "invalid_observation";
    case SparseTofKeyframeFault::DuplicateTimestamp: return "duplicate_timestamp";
    case SparseTofKeyframeFault::NonMonotonicTimestamp: return "non_monotonic_timestamp";
    case SparseTofKeyframeFault::BundleFull: return "bundle_full";
    case SparseTofKeyframeFault::BundleNotReady: return "bundle_not_ready";
    case SparseTofKeyframeFault::BootstrapRejected: return "bootstrap_rejected";
    case SparseTofKeyframeFault::ReferenceMapNotReady: return "reference_map_not_ready";
    case SparseTofKeyframeFault::MapUpdateRejected: return "map_update_rejected";
    }
    return "unknown";
}

struct SparseTofKeyframeConfig {
    std::size_t minimum_snapshot_count = 2;
    std::size_t maximum_snapshot_count = 8;
    std::size_t minimum_valid_ray_count = 3;
    std::size_t minimum_hit_ray_count = 1;
    double minimum_translation_m = 0.05;
    double minimum_rotation_rad = 0.05;
    double minimum_duration_s = 0.0;
    double maximum_duration_s = 5.0;
    std::size_t maximum_history_count = 16;
    bool trigger_on_motion_settled = true;
    bool allow_bootstrap_when_map_empty = true;
    bool allow_odom_only_commit_after_match_rejection = false;
};

struct SparseTofKeyframeMetadata {
    std::uint64_t keyframe_id = 0;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    RobotPose2D first_odom_pose;
    RobotPose2D last_odom_pose;
    RobotPose2D corrected_last_map_pose;
    SparseTofCorrectionMode correction_mode = SparseTofCorrectionMode::Rejected;
    double correction_dx_m = 0.0;
    double correction_dy_m = 0.0;
    double correction_dyaw_rad = 0.0;
    double best_score = 0.0;
    double runner_up_score = 0.0;
    double score_margin = 0.0;
    double known_cell_ratio = 0.0;
    std::size_t snapshot_count = 0;
    std::size_t valid_ray_count = 0;
    std::size_t hit_ray_count = 0;
    std::size_t no_return_ray_count = 0;
    bool bootstrap = false;
    bool matched = false;
    bool map_committed = false;
};

struct SparseTofKeyframeResult {
    bool ok = false;
    SparseTofKeyframeFault fault = SparseTofKeyframeFault::None;
    SparseTofKeyframeState state = SparseTofKeyframeState::Accumulating;
    std::string message;
    SparseOccupancyUpdateStats map_stats;
    SparseTofKeyframeMetadata metadata;
};

class SparseTofKeyframeManager {
public:
    explicit SparseTofKeyframeManager(SparseTofKeyframeConfig config = {})
        : config_(config) {}

    bool valid_config() const {
        return config_.minimum_snapshot_count > 0 &&
               config_.maximum_snapshot_count >= config_.minimum_snapshot_count &&
               config_.minimum_valid_ray_count > 0 &&
               config_.minimum_hit_ray_count <= config_.minimum_valid_ray_count &&
               std::isfinite(config_.minimum_translation_m) && config_.minimum_translation_m >= 0.0 &&
               std::isfinite(config_.minimum_rotation_rad) && config_.minimum_rotation_rad >= 0.0 &&
               std::isfinite(config_.minimum_duration_s) && config_.minimum_duration_s >= 0.0 &&
               std::isfinite(config_.maximum_duration_s) && config_.maximum_duration_s >= config_.minimum_duration_s &&
               config_.maximum_history_count > 0;
    }

    SparseTofKeyframeResult add_observation(const SparseTofMatchObservation &observation,
                                            bool motion_settled) {
        SparseTofKeyframeResult result;
        if (!valid_config()) {
            return reject(result, SparseTofKeyframeFault::InvalidConfig, "keyframe_invalid_config");
        }
        if (!valid_observation(observation)) {
            return reject(result, SparseTofKeyframeFault::InvalidObservation, "keyframe_invalid_observation");
        }
        if (!bundle_.observations.empty()) {
            if (observation.timestamp_s == bundle_.end_time_s) {
                return reject(result, SparseTofKeyframeFault::DuplicateTimestamp, "keyframe_duplicate_timestamp");
            }
            if (observation.timestamp_s < bundle_.end_time_s) {
                return reject(result, SparseTofKeyframeFault::NonMonotonicTimestamp, "keyframe_non_monotonic_timestamp");
            }
        }
        if (bundle_.observations.size() >= config_.maximum_snapshot_count) {
            return reject(result, SparseTofKeyframeFault::BundleFull, "keyframe_bundle_full");
        }
        bundle_.add(observation);
        result.ok = true;
        result.fault = SparseTofKeyframeFault::None;
        result.state = state_for(false, motion_settled);
        result.message = "keyframe_observation_added";
        return result;
    }

    const SparseTofObservationBundle &active_bundle() const { return bundle_; }
    std::size_t active_snapshot_count() const { return bundle_.observations.size(); }
    std::size_t history_count() const { return history_.size(); }
    std::size_t committed_keyframe_count() const { return committed_keyframe_count_; }
    bool odom_only_fallback_enabled() const { return config_.allow_odom_only_commit_after_match_rejection; }

    bool ready_for_bootstrap(bool map_empty, bool motion_settled) const {
        return map_empty && config_.allow_bootstrap_when_map_empty && ready_common(motion_settled);
    }

    bool ready_for_match(bool map_empty, bool motion_settled) const {
        return !map_empty && ready_common(motion_settled);
    }

    SparseTofKeyframeResult commit_bootstrap(SparseOccupancyGrid &grid,
                                             const RobotPose2D &map_from_odom) {
        if (!ready_for_bootstrap(grid.active_cell_count() == 0, true)) {
            SparseTofKeyframeResult result;
            return reject(result, SparseTofKeyframeFault::BootstrapRejected, "keyframe_bootstrap_not_ready");
        }
        return commit_bundle(grid, map_from_odom, true, SparseTofCorrectionMode::Rejected, SparseTofCandidateScore{});
    }

    SparseTofKeyframeResult commit_matched(SparseOccupancyGrid &grid,
                                           const RobotPose2D &map_from_odom,
                                           const SparseTofMatchResult &match) {
        if (!match.accepted) {
            SparseTofKeyframeResult result;
            return reject(result, SparseTofKeyframeFault::BundleNotReady, "keyframe_match_not_accepted");
        }
        if (grid.active_cell_count() == 0) {
            SparseTofKeyframeResult result;
            return reject(result, SparseTofKeyframeFault::ReferenceMapNotReady, "keyframe_reference_map_not_ready");
        }
        return commit_bundle(grid, map_from_odom, false, match.correction_mode, match.best);
    }

    void mark_match_rejected() {
        last_state_ = SparseTofKeyframeState::MatchRejected;
    }

    void clear_active_bundle() {
        bundle_ = SparseTofObservationBundle{};
    }

private:
    SparseTofKeyframeConfig config_;
    SparseTofObservationBundle bundle_;
    std::vector<SparseTofKeyframeMetadata> history_;
    std::uint64_t next_keyframe_id_ = 1;
    std::size_t committed_keyframe_count_ = 0;
    SparseTofKeyframeState last_state_ = SparseTofKeyframeState::Accumulating;

    bool valid_observation(const SparseTofMatchObservation &observation) const {
        if (!std::isfinite(observation.timestamp_s) || !se2_pose_finite(observation.odom_pose) || observation.ray_count > 3U) {
            return false;
        }
        for (std::size_t i = 0; i < observation.ray_count; ++i) {
            const auto &ray = observation.rays[i];
            if (!std::isfinite(ray.sensor_origin_x_m) || !std::isfinite(ray.sensor_origin_y_m) || !std::isfinite(ray.ray_yaw_rad)) {
                return false;
            }
        }
        return true;
    }

    bool ready_common(bool motion_settled) const {
        if (bundle_.observations.size() < config_.minimum_snapshot_count) return false;
        if (bundle_.valid_ray_count < config_.minimum_valid_ray_count) return false;
        if (bundle_.hit_ray_count < config_.minimum_hit_ray_count) return false;
        if (config_.trigger_on_motion_settled && !motion_settled) return false;
        const double duration = bundle_.end_time_s - bundle_.start_time_s;
        if (duration < config_.minimum_duration_s) return false;
        if (duration >= config_.maximum_duration_s) return true;
        const RobotPose2D motion = se2_between(bundle_.first_odom_pose, bundle_.last_odom_pose);
        return std::sqrt(se2_squared_translation_norm(motion)) >= config_.minimum_translation_m ||
               se2_abs_yaw(motion.yaw_rad) >= config_.minimum_rotation_rad ||
               bundle_.observations.size() >= config_.maximum_snapshot_count;
    }

    SparseTofKeyframeState state_for(bool map_empty, bool motion_settled) const {
        if (ready_for_bootstrap(map_empty, motion_settled)) return SparseTofKeyframeState::ReadyForBootstrap;
        if (ready_for_match(map_empty, motion_settled)) return SparseTofKeyframeState::ReadyForMatch;
        return SparseTofKeyframeState::Accumulating;
    }

    SparseTofKeyframeResult commit_bundle(SparseOccupancyGrid &grid,
                                          const RobotPose2D &map_from_odom,
                                          bool bootstrap,
                                          SparseTofCorrectionMode mode,
                                          const SparseTofCandidateScore &score) {
        SparseTofKeyframeResult result;
        if (!se2_pose_finite(map_from_odom) || bundle_.observations.empty()) {
            return reject(result, SparseTofKeyframeFault::BundleNotReady, "keyframe_bundle_not_ready");
        }
        const auto transformed = transformed_observations(map_from_odom);
        auto stats = grid.apply_observations(transformed);
        result.map_stats = stats;
        if (!stats.accepted) {
            return reject(result, SparseTofKeyframeFault::MapUpdateRejected, "keyframe_map_update_rejected");
        }
        result.ok = true;
        result.fault = SparseTofKeyframeFault::None;
        result.state = SparseTofKeyframeState::MapCommitted;
        result.message = bootstrap ? "keyframe_bootstrap_committed" : "keyframe_match_committed";
        result.metadata = metadata_for(bootstrap, mode, score, map_from_odom);
        result.metadata.map_committed = true;
        history_.push_back(result.metadata);
        if (history_.size() > config_.maximum_history_count) {
            history_.erase(history_.begin());
        }
        committed_keyframe_count_++;
        clear_active_bundle();
        last_state_ = SparseTofKeyframeState::MapCommitted;
        return result;
    }

    std::vector<SparseTofRayObservation> transformed_observations(const RobotPose2D &map_from_odom) const {
        std::vector<SparseTofRayObservation> out;
        for (const auto &obs : bundle_.observations) {
            const RobotPose2D map_pose = se2_compose(map_from_odom, obs.odom_pose);
            for (std::size_t i = 0; i < obs.ray_count && i < 3U; ++i) {
                auto ray = obs.rays[i];
                if (ray.return_kind != ScalarTofReturnKind::Hit && ray.return_kind != ScalarTofReturnKind::NoReturn) {
                    out.push_back(ray);
                    continue;
                }
                RobotPose2D ray_pose;
                ray_pose.x_m = ray.sensor_origin_x_m;
                ray_pose.y_m = ray.sensor_origin_y_m;
                ray_pose.yaw_rad = ray.ray_yaw_rad;
                const RobotPose2D local_ray = se2_between(obs.odom_pose, ray_pose);
                const RobotPose2D mapped_ray = se2_compose(map_pose, local_ray);
                ray.sensor_origin_x_m = mapped_ray.x_m;
                ray.sensor_origin_y_m = mapped_ray.y_m;
                ray.ray_yaw_rad = mapped_ray.yaw_rad;
                out.push_back(ray);
            }
        }
        return out;
    }

    SparseTofKeyframeMetadata metadata_for(bool bootstrap,
                                           SparseTofCorrectionMode mode,
                                           const SparseTofCandidateScore &score,
                                           const RobotPose2D &map_from_odom) {
        SparseTofKeyframeMetadata meta;
        meta.keyframe_id = next_keyframe_id_++;
        meta.start_time_s = bundle_.start_time_s;
        meta.end_time_s = bundle_.end_time_s;
        meta.first_odom_pose = bundle_.first_odom_pose;
        meta.last_odom_pose = bundle_.last_odom_pose;
        meta.corrected_last_map_pose = se2_compose(map_from_odom, bundle_.last_odom_pose);
        meta.correction_mode = mode;
        meta.correction_dx_m = score.correction.x_m;
        meta.correction_dy_m = score.correction.y_m;
        meta.correction_dyaw_rad = score.correction.yaw_rad;
        meta.best_score = score.normalized_score;
        meta.runner_up_score = 0.0;
        meta.score_margin = 0.0;
        meta.known_cell_ratio = score.evaluated_cell_count == 0 ? 0.0 :
            static_cast<double>(score.known_cell_count) / static_cast<double>(score.evaluated_cell_count);
        meta.snapshot_count = bundle_.observations.size();
        meta.valid_ray_count = bundle_.valid_ray_count;
        meta.hit_ray_count = bundle_.hit_ray_count;
        meta.no_return_ray_count = bundle_.no_return_ray_count;
        meta.bootstrap = bootstrap;
        meta.matched = !bootstrap;
        return meta;
    }

    SparseTofKeyframeResult reject(SparseTofKeyframeResult result,
                                   SparseTofKeyframeFault fault,
                                   const std::string &message) const {
        result.ok = false;
        result.fault = fault;
        result.state = last_state_;
        result.message = message;
        return result;
    }
};

} // namespace robot_slamd
