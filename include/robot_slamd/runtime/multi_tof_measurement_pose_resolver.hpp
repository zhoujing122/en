#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/runtime/multi_tof_measurement_pose_types.hpp"
#include "robot_slamd/runtime/sparse_slam_initialization.hpp"

#include <cmath>
#include <string>

namespace robot_slamd {

struct MultiTofMeasurementPoseResolverConfig {
    bool require_all_three_measurement_poses = true;
};

struct MultiTofMeasurementPoseResolverResult {
    bool ok = false;
    MultiTofMeasurementPoseSet poses;
    std::string reason;
};

class MultiTofMeasurementPoseResolver {
public:
    explicit MultiTofMeasurementPoseResolver(
        MultiTofMeasurementPoseResolverConfig config = {})
        : config_(config) {}

    MultiTofMeasurementPoseResolverResult resolve(
        const MultiTofSnapshot &snapshot,
        const TimedOdomPoseBuffer &pose_buffer,
        const MapOdomFrameState &frame_state) const {
        MultiTofMeasurementPoseResolverResult result;
        if (!frame_state.initialized()) {
            result.reason = "map_odom_frame_not_initialized";
            return result;
        }

        result.poses.front = resolve_one(snapshot.has_front,
                                         snapshot.front.effective_timestamp_s,
                                         pose_buffer,
                                         frame_state);
        result.poses.left = resolve_one(snapshot.has_left,
                                        snapshot.left.effective_timestamp_s,
                                        pose_buffer,
                                        frame_state);
        result.poses.right = resolve_one(snapshot.has_right,
                                         snapshot.right.effective_timestamp_s,
                                         pose_buffer,
                                         frame_state);
        result.poses.valid_pose_count =
            (result.poses.front.valid ? 1 : 0) +
            (result.poses.left.valid ? 1 : 0) +
            (result.poses.right.valid ? 1 : 0);
        result.poses.complete =
            result.poses.front.valid && result.poses.left.valid &&
            result.poses.right.valid;
        result.ok = config_.require_all_three_measurement_poses
                        ? result.poses.complete
                        : result.poses.valid_pose_count > 0;
        result.reason = result.ok ? "measurement_pose_set_ready"
                                  : "measurement_pose_set_incomplete";
        return result;
    }

private:
    static TimedMapBasePose2D resolve_one(bool present,
                                          double timestamp_s,
                                          const TimedOdomPoseBuffer &pose_buffer,
                                          const MapOdomFrameState &frame_state) {
        TimedMapBasePose2D out;
        out.measurement_timestamp_s = timestamp_s;
        if (!present) {
            out.reason = "tof_frame_missing";
            return out;
        }
        if (!std::isfinite(timestamp_s)) {
            out.lookup_status = TimedPoseLookupStatus::InvalidTimestamp;
            out.reason = "tof_effective_timestamp_invalid";
            return out;
        }
        const auto lookup = pose_buffer.lookup(timestamp_s);
        out.lookup_status = lookup.status;
        if (!lookup.ok) {
            out.reason = lookup.reason;
            return out;
        }
        out.base_pose_in_map = frame_state.map_pose_from_odom(lookup.pose);
        out.valid = sparse_slam_frame_pose_valid(out.base_pose_in_map);
        out.reason = out.valid ? "measurement_pose_ready"
                               : "measurement_pose_invalid";
        return out;
    }

    MultiTofMeasurementPoseResolverConfig config_;
};

} // namespace robot_slamd
