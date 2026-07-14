#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/mapping/sparse_tof/scalar_tof_return_kind.hpp"
#include "robot_slamd/runtime/multi_tof_measurement_pose_types.hpp"
#include "robot_slamd/runtime/sparse_slam_frames.hpp"

#include <cstdint>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

enum class ActiveObservationBundleState {
    Idle,
    CollectingDuringMotion,
    WaitingForMotionSettle,
    FrozenReady,
    Aborted
};

inline std::string to_string(ActiveObservationBundleState state) {
    switch (state) {
    case ActiveObservationBundleState::Idle:
        return "idle";
    case ActiveObservationBundleState::CollectingDuringMotion:
        return "collecting_during_motion";
    case ActiveObservationBundleState::WaitingForMotionSettle:
        return "waiting_for_motion_settle";
    case ActiveObservationBundleState::FrozenReady:
        return "frozen_ready";
    case ActiveObservationBundleState::Aborted:
        return "aborted";
    }
    return "unknown";
}

enum class ActiveObservationBundleFault {
    None,
    InvalidConfig,
    InvalidState,
    MissingMultiTof,
    MissingMeasurementPose,
    InvalidTimestamp,
    NonMonotonicTimestamp,
    SnapshotGapExceeded,
    DurationExceeded,
    CapacityExceeded,
    NoMatchableRay,
    NonFinitePose,
    InsufficientSnapshots,
    InsufficientMatchableRays,
    InsufficientYawCoverage,
    BundleIdMismatch,
    FrozenImmutable,
    ReferenceMapChangedDuringActiveBundle
};

inline std::string to_string(ActiveObservationBundleFault fault) {
    switch (fault) {
    case ActiveObservationBundleFault::None:
        return "none";
    case ActiveObservationBundleFault::InvalidConfig:
        return "invalid_config";
    case ActiveObservationBundleFault::InvalidState:
        return "invalid_state";
    case ActiveObservationBundleFault::MissingMultiTof:
        return "missing_multi_tof";
    case ActiveObservationBundleFault::MissingMeasurementPose:
        return "missing_measurement_pose";
    case ActiveObservationBundleFault::InvalidTimestamp:
        return "invalid_timestamp";
    case ActiveObservationBundleFault::NonMonotonicTimestamp:
        return "non_monotonic_timestamp";
    case ActiveObservationBundleFault::SnapshotGapExceeded:
        return "snapshot_gap_exceeded";
    case ActiveObservationBundleFault::DurationExceeded:
        return "duration_exceeded";
    case ActiveObservationBundleFault::CapacityExceeded:
        return "capacity_exceeded";
    case ActiveObservationBundleFault::NoMatchableRay:
        return "no_matchable_ray";
    case ActiveObservationBundleFault::NonFinitePose:
        return "non_finite_pose";
    case ActiveObservationBundleFault::InsufficientSnapshots:
        return "insufficient_snapshots";
    case ActiveObservationBundleFault::InsufficientMatchableRays:
        return "insufficient_matchable_rays";
    case ActiveObservationBundleFault::InsufficientYawCoverage:
        return "insufficient_yaw_coverage";
    case ActiveObservationBundleFault::BundleIdMismatch:
        return "bundle_id_mismatch";
    case ActiveObservationBundleFault::FrozenImmutable:
        return "frozen_immutable";
    case ActiveObservationBundleFault::ReferenceMapChangedDuringActiveBundle:
        return "reference_map_changed_during_active_bundle";
    }
    return "unknown";
}

struct ActiveMultiTofObservationBundleConfig {
    std::size_t max_snapshots = 64;
    std::size_t max_rays = 192;
    double max_duration_s = 8.0;
    double max_snapshot_gap_s = 0.50;
    std::size_t min_snapshots = 2;
    std::size_t min_matchable_rays = 3;
    double min_yaw_span_rad = 0.0;
    bool require_all_measurement_poses = true;
};

struct ResolvedScalarTofObservationRoute {
    bool present = false;
    ScalarTofSnapshotFrame frame;
    double effective_timestamp_s = 0.0;
    ScalarTofReturnKind return_kind = ScalarTofReturnKind::Unspecified;
    TimedMapBasePose2D map_pose;
    OdomPose2D odom_pose_at_measurement;
};

struct ResolvedMultiTofObservationFrame {
    double snapshot_timestamp_s = 0.0;
    std::size_t sequence_index = 0;
    std::string source;
    ResolvedScalarTofObservationRoute front;
    ResolvedScalarTofObservationRoute left;
    ResolvedScalarTofObservationRoute right;
    bool complete = false;
    std::string reject_reason;
};

struct ActiveMultiTofObservationBundleSummary {
    ActiveObservationBundleState state = ActiveObservationBundleState::Idle;
    std::uint64_t bundle_id = 0;
    double collection_start_timestamp_s = 0.0;
    double collection_end_timestamp_s = 0.0;
    double duration_s = 0.0;
    std::uint64_t reference_map_revision = 0;
    std::size_t reference_map_cell_count = 0;
    MapFromOdom2D map_from_odom_at_start = identity_map_from_odom();
    std::size_t accepted_snapshot_count = 0;
    std::size_t rejected_snapshot_count = 0;
    std::size_t front_ray_count = 0;
    std::size_t left_ray_count = 0;
    std::size_t right_ray_count = 0;
    std::size_t hit_ray_count = 0;
    std::size_t no_return_ray_count = 0;
    std::size_t invalid_ray_count = 0;
    std::size_t unspecified_ray_count = 0;
    std::size_t matchable_ray_count = 0;
    double odom_yaw_start_rad = 0.0;
    double odom_yaw_end_rad = 0.0;
    double accumulated_abs_yaw_rad = 0.0;
    double yaw_span_rad = 0.0;
    bool complete = false;
    bool frozen = false;
    ActiveObservationBundleFault fault = ActiveObservationBundleFault::None;
    std::string reason;
};

struct ActiveMultiTofObservationAppendInput {
    RobotSlamSensorSnapshot snapshot;
    MultiTofMeasurementPoseSet measurement_poses;
    OdomPose2D front_odom_pose_at_measurement;
    OdomPose2D left_odom_pose_at_measurement;
    OdomPose2D right_odom_pose_at_measurement;
    std::size_t sequence_index = 0;
};

struct ActiveMultiTofObservationResult {
    bool ok = false;
    ActiveObservationBundleFault fault = ActiveObservationBundleFault::None;
    std::string message;
};

class FrozenMultiTofObservationBundle {
public:
    FrozenMultiTofObservationBundle() = default;

    FrozenMultiTofObservationBundle(
        ActiveMultiTofObservationBundleSummary summary,
        std::vector<ResolvedMultiTofObservationFrame> frames)
        : summary_(std::move(summary)),
          frames_(std::move(frames)) {
        summary_.frozen = true;
        summary_.complete = true;
        summary_.state = ActiveObservationBundleState::FrozenReady;
    }

    bool available() const { return summary_.frozen; }
    const ActiveMultiTofObservationBundleSummary &summary() const {
        return summary_;
    }
    const std::vector<ResolvedMultiTofObservationFrame> &frames() const {
        return frames_;
    }

private:
    ActiveMultiTofObservationBundleSummary summary_;
    std::vector<ResolvedMultiTofObservationFrame> frames_;
};

} // namespace robot_slamd
