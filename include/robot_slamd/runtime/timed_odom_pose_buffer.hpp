#pragma once

#include "robot_slamd/runtime/sparse_slam_frames.hpp"

#include <cstddef>
#include <deque>
#include <limits>
#include <string>

namespace robot_slamd {

enum class TimedPoseLookupStatus {
    Exact,
    Interpolated,
    Empty,
    BeforeOldest,
    AfterNewest,
    GapTooLarge,
    InvalidTimestamp,
    InvalidPose
};

enum class TimedPoseAppendStatus {
    Accepted,
    InvalidTimestamp,
    InvalidPose,
    NonMonotonicTimestamp,
    InvalidConfig
};

struct TimedOdomPoseBufferConfig {
    std::size_t capacity = 64;
    std::size_t maximum_capacity = 4096;
    double max_age_s = 10.0;
    double max_interpolation_gap_s = 0.25;
};

struct TimedOdomPoseSample {
    double timestamp_s = 0.0;
    OdomPose2D odom_pose;
};

struct TimedOdomPoseAppendResult {
    bool ok = false;
    TimedPoseAppendStatus status = TimedPoseAppendStatus::InvalidConfig;
    std::string reason;
};

struct TimedOdomPoseLookupResult {
    bool ok = false;
    TimedPoseLookupStatus status = TimedPoseLookupStatus::Empty;
    OdomPose2D pose;
    double lower_timestamp_s = 0.0;
    double upper_timestamp_s = 0.0;
    double interpolation_ratio = 0.0;
    std::string reason;
};

inline const char *to_string(TimedPoseLookupStatus status) {
    switch (status) {
    case TimedPoseLookupStatus::Exact:
        return "exact";
    case TimedPoseLookupStatus::Interpolated:
        return "interpolated";
    case TimedPoseLookupStatus::Empty:
        return "empty";
    case TimedPoseLookupStatus::BeforeOldest:
        return "before_oldest";
    case TimedPoseLookupStatus::AfterNewest:
        return "after_newest";
    case TimedPoseLookupStatus::GapTooLarge:
        return "gap_too_large";
    case TimedPoseLookupStatus::InvalidTimestamp:
        return "invalid_timestamp";
    case TimedPoseLookupStatus::InvalidPose:
        return "invalid_pose";
    }
    return "unknown";
}

class TimedOdomPoseBuffer {
public:
    explicit TimedOdomPoseBuffer(TimedOdomPoseBufferConfig config = {})
        : config_(config) {}

    bool config_valid() const {
        return config_.capacity > 1 &&
               config_.capacity <= config_.maximum_capacity &&
               config_.maximum_capacity > 1 &&
               std::isfinite(config_.max_age_s) && config_.max_age_s >= 0.0 &&
               std::isfinite(config_.max_interpolation_gap_s) &&
               config_.max_interpolation_gap_s >= 0.0;
    }

    TimedOdomPoseAppendResult append(double timestamp_s,
                                     const OdomPose2D &pose) {
        if (!config_valid()) {
            return {false, TimedPoseAppendStatus::InvalidConfig,
                    "timed_pose_buffer_invalid_config"};
        }
        if (!std::isfinite(timestamp_s)) {
            return {false, TimedPoseAppendStatus::InvalidTimestamp,
                    "timed_pose_timestamp_invalid"};
        }
        if (!sparse_slam_frame_pose_valid(pose)) {
            return {false, TimedPoseAppendStatus::InvalidPose,
                    "timed_pose_invalid"};
        }
        if (!samples_.empty() && timestamp_s <= samples_.back().timestamp_s) {
            return {false, TimedPoseAppendStatus::NonMonotonicTimestamp,
                    "timed_pose_timestamp_not_monotonic"};
        }

        samples_.push_back(TimedOdomPoseSample{timestamp_s, pose});
        while (samples_.size() > config_.capacity) {
            samples_.pop_front();
        }
        while (samples_.size() > 1 &&
               timestamp_s - samples_.front().timestamp_s > config_.max_age_s) {
            samples_.pop_front();
        }
        return {true, TimedPoseAppendStatus::Accepted,
                "timed_pose_accepted"};
    }

    TimedOdomPoseLookupResult lookup(double timestamp_s) const {
        if (!std::isfinite(timestamp_s)) {
            return failure(TimedPoseLookupStatus::InvalidTimestamp,
                           "lookup_timestamp_invalid");
        }
        if (samples_.empty()) {
            return failure(TimedPoseLookupStatus::Empty,
                           "timed_pose_buffer_empty");
        }
        if (timestamp_s < samples_.front().timestamp_s) {
            auto out = failure(TimedPoseLookupStatus::BeforeOldest,
                               "lookup_before_oldest");
            out.lower_timestamp_s = samples_.front().timestamp_s;
            out.upper_timestamp_s = samples_.front().timestamp_s;
            return out;
        }
        if (timestamp_s > samples_.back().timestamp_s) {
            auto out = failure(TimedPoseLookupStatus::AfterNewest,
                               "lookup_after_newest");
            out.lower_timestamp_s = samples_.back().timestamp_s;
            out.upper_timestamp_s = samples_.back().timestamp_s;
            return out;
        }

        for (std::size_t i = 0; i < samples_.size(); ++i) {
            if (timestamp_s == samples_[i].timestamp_s) {
                return success(TimedPoseLookupStatus::Exact,
                               samples_[i].odom_pose,
                               samples_[i].timestamp_s,
                               samples_[i].timestamp_s,
                               0.0);
            }
        }

        for (std::size_t i = 1; i < samples_.size(); ++i) {
            const auto &lower = samples_[i - 1];
            const auto &upper = samples_[i];
            if (lower.timestamp_s < timestamp_s &&
                timestamp_s < upper.timestamp_s) {
                const double gap = upper.timestamp_s - lower.timestamp_s;
                if (gap > config_.max_interpolation_gap_s) {
                    auto out = failure(TimedPoseLookupStatus::GapTooLarge,
                                       "lookup_gap_too_large");
                    out.lower_timestamp_s = lower.timestamp_s;
                    out.upper_timestamp_s = upper.timestamp_s;
                    return out;
                }
                const double ratio = (timestamp_s - lower.timestamp_s) / gap;
                RobotPose2D pose;
                pose.x_m = lower.odom_pose.odom_T_base.x_m +
                           ratio * (upper.odom_pose.odom_T_base.x_m -
                                    lower.odom_pose.odom_T_base.x_m);
                pose.y_m = lower.odom_pose.odom_T_base.y_m +
                           ratio * (upper.odom_pose.odom_T_base.y_m -
                                    lower.odom_pose.odom_T_base.y_m);
                pose.yaw_rad = sparse_slam_normalize_yaw(
                    lower.odom_pose.odom_T_base.yaw_rad +
                    ratio * sparse_slam_shortest_yaw_delta(
                                lower.odom_pose.odom_T_base.yaw_rad,
                                upper.odom_pose.odom_T_base.yaw_rad));
                const OdomPose2D odom_pose = make_odom_pose(pose);
                if (!sparse_slam_frame_pose_valid(odom_pose)) {
                    return failure(TimedPoseLookupStatus::InvalidPose,
                                   "interpolated_pose_invalid");
                }
                return success(TimedPoseLookupStatus::Interpolated,
                               odom_pose,
                               lower.timestamp_s,
                               upper.timestamp_s,
                               ratio);
            }
        }

        return failure(TimedPoseLookupStatus::InvalidTimestamp,
                       "lookup_unreachable");
    }

    std::size_t size() const { return samples_.size(); }
    bool empty() const { return samples_.empty(); }
    void clear() { samples_.clear(); }

private:
    static TimedOdomPoseLookupResult failure(TimedPoseLookupStatus status,
                                             const std::string &reason) {
        TimedOdomPoseLookupResult out;
        out.ok = false;
        out.status = status;
        out.reason = reason;
        return out;
    }

    static TimedOdomPoseLookupResult success(TimedPoseLookupStatus status,
                                             const OdomPose2D &pose,
                                             double lower_timestamp_s,
                                             double upper_timestamp_s,
                                             double ratio) {
        TimedOdomPoseLookupResult out;
        out.ok = true;
        out.status = status;
        out.pose = pose;
        out.lower_timestamp_s = lower_timestamp_s;
        out.upper_timestamp_s = upper_timestamp_s;
        out.interpolation_ratio = ratio;
        out.reason = to_string(status);
        return out;
    }

    TimedOdomPoseBufferConfig config_;
    std::deque<TimedOdomPoseSample> samples_;
};

} // namespace robot_slamd
