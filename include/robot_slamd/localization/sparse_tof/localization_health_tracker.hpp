#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_match_types.hpp"

#include <cmath>
#include <cstddef>
#include <string>

namespace robot_slamd {

enum class LocalizationHealthState {
    Healthy,
    Degraded,
    Lost
};

inline const char *to_string(LocalizationHealthState state) {
    switch (state) {
    case LocalizationHealthState::Healthy: return "healthy";
    case LocalizationHealthState::Degraded: return "degraded";
    case LocalizationHealthState::Lost: return "lost";
    }
    return "unknown";
}

enum class LocalizationMatchEvidence {
    Accepted,
    Ambiguous,
    ConsistencyFailure,
    Fault,
    None
};

inline LocalizationMatchEvidence localization_match_evidence(
    SparseTofLocalMatchStatus status) {
    switch (status) {
    case SparseTofLocalMatchStatus::AcceptedYawOnly:
    case SparseTofLocalMatchStatus::AcceptedFullSE2:
        return LocalizationMatchEvidence::Accepted;
    case SparseTofLocalMatchStatus::RejectedInsufficientInformation:
    case SparseTofLocalMatchStatus::RejectedBestAtSearchEdge:
    case SparseTofLocalMatchStatus::RejectedLowMargin:
    case SparseTofLocalMatchStatus::RejectedFlatCurve:
    case SparseTofLocalMatchStatus::RejectedMultimodal:
        return LocalizationMatchEvidence::Ambiguous;
    case SparseTofLocalMatchStatus::RejectedUnknownDominated:
    case SparseTofLocalMatchStatus::RejectedContradictory:
    case SparseTofLocalMatchStatus::RejectedNoValidCandidate:
    case SparseTofLocalMatchStatus::RejectedLowScore:
        return LocalizationMatchEvidence::ConsistencyFailure;
    case SparseTofLocalMatchStatus::Fault:
    case SparseTofLocalMatchStatus::RejectedInvalidInput:
        return LocalizationMatchEvidence::Fault;
    case SparseTofLocalMatchStatus::NotRun:
    case SparseTofLocalMatchStatus::InputRejected:
    case SparseTofLocalMatchStatus::NotImplemented:
    case SparseTofLocalMatchStatus::Rejected:
    case SparseTofLocalMatchStatus::RejectedCandidateBudget:
        return LocalizationMatchEvidence::None;
    }
    return LocalizationMatchEvidence::None;
}

struct LocalizationHealthConfig {
    std::size_t max_consecutive_consistency_failures = 3;
    double minimum_lost_duration_s = 1.0;
    double minimum_lost_odom_distance_m = 0.20;
};

inline bool localization_health_config_valid(
    const LocalizationHealthConfig &config) {
    return config.max_consecutive_consistency_failures > 0 &&
           config.max_consecutive_consistency_failures <= 100 &&
           std::isfinite(config.minimum_lost_duration_s) &&
           config.minimum_lost_duration_s >= 0.0 &&
           std::isfinite(config.minimum_lost_odom_distance_m) &&
           config.minimum_lost_odom_distance_m >= 0.0;
}

struct LocalizationHealthSample {
    double timestamp_s = 0.0;
    RobotPose2D odom_pose;
    SparseTofLocalMatchStatus match_status =
        SparseTofLocalMatchStatus::NotRun;
    bool wheel_fresh = true;
    bool imu_fresh = true;
    bool frame_state_valid = true;
};

struct LocalizationHealthUpdate {
    bool ok = false;
    bool became_lost = false;
    LocalizationHealthState state = LocalizationHealthState::Healthy;
    LocalizationMatchEvidence evidence = LocalizationMatchEvidence::None;
    std::size_t consecutive_consistency_failures = 0;
    std::size_t ambiguity_reject_count = 0;
    double duration_since_accepted_s = 0.0;
    double odom_distance_since_accepted_m = 0.0;
    std::string reason;
};

class LocalizationHealthTracker {
public:
    explicit LocalizationHealthTracker(LocalizationHealthConfig config = {})
        : config_(config) {}

    LocalizationHealthUpdate update(const LocalizationHealthSample &sample) {
        LocalizationHealthUpdate out;
        out.state = state_;
        if (!localization_health_config_valid(config_) ||
            !std::isfinite(sample.timestamp_s) ||
            (have_timestamp_ && sample.timestamp_s < last_timestamp_s_)) {
            return immediate_lost(out, "localization_health_timestamp_invalid");
        }
        have_timestamp_ = true;
        last_timestamp_s_ = sample.timestamp_s;
        if (!sample.wheel_fresh || !sample.imu_fresh ||
            !sample.frame_state_valid ||
            !std::isfinite(sample.odom_pose.x_m) ||
            !std::isfinite(sample.odom_pose.y_m) ||
            !std::isfinite(sample.odom_pose.yaw_rad)) {
            return immediate_lost(out, "localization_health_sensor_or_pose_fault");
        }

        out.evidence = localization_match_evidence(sample.match_status);
        if (!have_acceptance_) {
            have_acceptance_ = true;
            last_accepted_timestamp_s_ = sample.timestamp_s;
            last_accepted_odom_pose_ = sample.odom_pose;
        }
        out.duration_since_accepted_s =
            sample.timestamp_s - last_accepted_timestamp_s_;
        out.odom_distance_since_accepted_m = std::hypot(
            sample.odom_pose.x_m - last_accepted_odom_pose_.x_m,
            sample.odom_pose.y_m - last_accepted_odom_pose_.y_m);

        if (out.evidence == LocalizationMatchEvidence::Accepted) {
            state_ = LocalizationHealthState::Healthy;
            consecutive_consistency_failures_ = 0;
            ambiguity_reject_count_ = 0;
            last_accepted_timestamp_s_ = sample.timestamp_s;
            last_accepted_odom_pose_ = sample.odom_pose;
            out.reason = "localization_health_match_accepted";
        } else if (out.evidence == LocalizationMatchEvidence::Ambiguous) {
            consecutive_consistency_failures_ = 0;
            ambiguity_reject_count_++;
            state_ = LocalizationHealthState::Degraded;
            out.reason = "localization_health_ambiguous";
        } else if (out.evidence ==
                   LocalizationMatchEvidence::ConsistencyFailure) {
            consecutive_consistency_failures_++;
            state_ = LocalizationHealthState::Degraded;
            const bool enough_failures =
                consecutive_consistency_failures_ >=
                    config_.max_consecutive_consistency_failures;
            const bool enough_motion_or_time =
                out.duration_since_accepted_s >=
                    config_.minimum_lost_duration_s ||
                out.odom_distance_since_accepted_m >=
                    config_.minimum_lost_odom_distance_m;
            if (enough_failures && enough_motion_or_time) {
                state_ = LocalizationHealthState::Lost;
                out.became_lost = true;
                out.reason = "localization_consistency_failures_exceeded";
            } else {
                out.reason = "localization_consistency_degraded";
            }
        } else if (out.evidence == LocalizationMatchEvidence::Fault) {
            return immediate_lost(out, "localization_match_fault");
        } else {
            out.reason = "localization_health_no_new_evidence";
        }
        out.ok = true;
        out.state = state_;
        out.consecutive_consistency_failures =
            consecutive_consistency_failures_;
        out.ambiguity_reject_count = ambiguity_reject_count_;
        return out;
    }

    void mark_recovered(double timestamp_s, const RobotPose2D &odom_pose) {
        reset();
        have_acceptance_ = true;
        have_timestamp_ = true;
        last_timestamp_s_ = timestamp_s;
        last_accepted_timestamp_s_ = timestamp_s;
        last_accepted_odom_pose_ = odom_pose;
    }

    void reset() { *this = LocalizationHealthTracker(config_); }
    LocalizationHealthState state() const { return state_; }

private:
    LocalizationHealthUpdate immediate_lost(
        LocalizationHealthUpdate out, const std::string &reason) {
        const bool was_lost = state_ == LocalizationHealthState::Lost;
        state_ = LocalizationHealthState::Lost;
        out.ok = false;
        out.became_lost = !was_lost;
        out.state = state_;
        out.evidence = LocalizationMatchEvidence::Fault;
        out.reason = reason;
        return out;
    }

    LocalizationHealthConfig config_;
    LocalizationHealthState state_ = LocalizationHealthState::Healthy;
    bool have_timestamp_ = false;
    double last_timestamp_s_ = 0.0;
    bool have_acceptance_ = false;
    double last_accepted_timestamp_s_ = 0.0;
    RobotPose2D last_accepted_odom_pose_;
    std::size_t consecutive_consistency_failures_ = 0;
    std::size_t ambiguity_reject_count_ = 0;
};

} // namespace robot_slamd
