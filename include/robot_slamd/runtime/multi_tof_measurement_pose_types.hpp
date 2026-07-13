#pragma once

#include "robot_slamd/runtime/sparse_slam_frames.hpp"
#include "robot_slamd/runtime/timed_odom_pose_buffer.hpp"

#include <string>

namespace robot_slamd {

struct TimedMapBasePose2D {
    bool valid = false;
    double measurement_timestamp_s = 0.0;
    MapPose2D base_pose_in_map;
    TimedPoseLookupStatus lookup_status = TimedPoseLookupStatus::Empty;
    std::string reason;
};

struct MultiTofMeasurementPoseSet {
    TimedMapBasePose2D front;
    TimedMapBasePose2D left;
    TimedMapBasePose2D right;
    bool complete = false;
    int valid_pose_count = 0;
};

} // namespace robot_slamd
