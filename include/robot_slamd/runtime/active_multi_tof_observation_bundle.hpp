#pragma once

#include "robot_slamd/runtime/active_multi_tof_observation_types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robot_slamd {

inline bool active_bundle_config_valid(
    const ActiveMultiTofObservationBundleConfig &config) {
    constexpr std::size_t kMaxSnapshotsHardLimit = 1024;
    constexpr std::size_t kMaxRaysHardLimit = kMaxSnapshotsHardLimit * 3;
    return config.max_snapshots > 0 &&
           config.max_snapshots <= kMaxSnapshotsHardLimit &&
           config.max_rays > 0 &&
           config.max_rays <= kMaxRaysHardLimit &&
           config.max_rays >= config.max_snapshots * 3 &&
           std::isfinite(config.max_duration_s) &&
           config.max_duration_s > 0.0 &&
           std::isfinite(config.max_snapshot_gap_s) &&
           config.max_snapshot_gap_s > 0.0 &&
           config.min_snapshots <= config.max_snapshots &&
           config.min_matchable_rays <= config.max_rays &&
           std::isfinite(config.min_yaw_span_rad) &&
           config.min_yaw_span_rad >= 0.0 &&
           config.min_yaw_span_rad <= 6.28318530717958647692;
}

class ActiveMultiTofObservationBundleBuilder {
public:
    explicit ActiveMultiTofObservationBundleBuilder(
        ActiveMultiTofObservationBundleConfig config = {})
        : config_(config) {}

    ActiveMultiTofObservationResult begin(
        std::uint64_t bundle_id,
        std::uint64_t reference_map_revision,
        std::size_t reference_map_cell_count,
        const MapFromOdom2D &map_from_odom_at_start) {
        if (!active_bundle_config_valid(config_) ||
            !sparse_slam_frame_pose_valid(map_from_odom_at_start)) {
            return fail(ActiveObservationBundleFault::InvalidConfig,
                        "active_bundle_invalid_config");
        }
        frames_.clear();
        summary_ = ActiveMultiTofObservationBundleSummary{};
        summary_.state = ActiveObservationBundleState::CollectingDuringMotion;
        summary_.bundle_id = bundle_id;
        summary_.reference_map_revision = reference_map_revision;
        summary_.reference_map_cell_count = reference_map_cell_count;
        summary_.map_from_odom_at_start = map_from_odom_at_start;
        last_snapshot_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
        last_unwrapped_yaw_rad_ = 0.0;
        yaw_min_rad_ = 0.0;
        yaw_max_rad_ = 0.0;
        has_yaw_ = false;
        frozen_ = FrozenMultiTofObservationBundle{};
        return ok("active_bundle_started");
    }

    ActiveMultiTofObservationResult append(
        const ActiveMultiTofObservationAppendInput &input) {
        if (summary_.state == ActiveObservationBundleState::FrozenReady) {
            return fail_no_mutation(ActiveObservationBundleFault::FrozenImmutable,
                                    "active_bundle_frozen");
        }
        if (summary_.state == ActiveObservationBundleState::Aborted) {
            return fail_no_mutation(ActiveObservationBundleFault::InvalidState,
                                    "active_bundle_aborted");
        }
        if (summary_.state != ActiveObservationBundleState::CollectingDuringMotion &&
            summary_.state != ActiveObservationBundleState::WaitingForMotionSettle) {
            return fail_no_mutation(ActiveObservationBundleFault::InvalidState,
                                    "active_bundle_not_collecting");
        }
        const auto validation = validate_append(input);
        if (!validation.ok) {
            if (validation.fault == ActiveObservationBundleFault::CapacityExceeded ||
                validation.fault == ActiveObservationBundleFault::DurationExceeded ||
                validation.fault == ActiveObservationBundleFault::SnapshotGapExceeded ||
                validation.fault == ActiveObservationBundleFault::NonMonotonicTimestamp) {
                abort(validation.fault, validation.message);
            } else {
                summary_.rejected_snapshot_count++;
                summary_.fault = validation.fault;
                summary_.reason = validation.message;
            }
            return validation;
        }

        const double timestamp_s = input.snapshot.multi_tof.synchronized_time_s;
        ResolvedMultiTofObservationFrame frame;
        frame.snapshot_timestamp_s = timestamp_s;
        frame.sequence_index = input.sequence_index;
        frame.source = input.snapshot.multi_tof.source;
        frame.front = route(input.snapshot.multi_tof.has_front,
                            input.snapshot.multi_tof.front,
                            input.measurement_poses.front,
                            input.front_odom_pose_at_measurement);
        frame.left = route(input.snapshot.multi_tof.has_left,
                           input.snapshot.multi_tof.left,
                           input.measurement_poses.left,
                           input.left_odom_pose_at_measurement);
        frame.right = route(input.snapshot.multi_tof.has_right,
                            input.snapshot.multi_tof.right,
                            input.measurement_poses.right,
                            input.right_odom_pose_at_measurement);
        frame.complete = frame.front.present && frame.left.present &&
                         frame.right.present &&
                         frame.front.map_pose.valid &&
                         frame.left.map_pose.valid &&
                         frame.right.map_pose.valid;

        ActiveMultiTofObservationBundleSummary next = summary_;
        if (next.accepted_snapshot_count == 0) {
            next.collection_start_timestamp_s = timestamp_s;
            next.odom_yaw_start_rad =
                input.front_odom_pose_at_measurement.odom_T_base.yaw_rad;
        }
        next.collection_end_timestamp_s = timestamp_s;
        next.duration_s =
            next.collection_end_timestamp_s - next.collection_start_timestamp_s;
        next.accepted_snapshot_count++;
        count_route(frame.front, next.front_ray_count, next);
        count_route(frame.left, next.left_ray_count, next);
        count_route(frame.right, next.right_ray_count, next);
        update_yaw(input.front_odom_pose_at_measurement, next);
        next.fault = ActiveObservationBundleFault::None;
        next.reason = "active_bundle_append_accepted";
        next.complete = false;
        next.frozen = false;

        frames_.push_back(frame);
        summary_ = next;
        last_snapshot_timestamp_s_ = timestamp_s;
        return ok("active_bundle_append_accepted");
    }

    ActiveMultiTofObservationResult mark_waiting_for_motion_settle() {
        if (summary_.state != ActiveObservationBundleState::CollectingDuringMotion) {
            return fail_no_mutation(ActiveObservationBundleFault::InvalidState,
                                    "active_bundle_not_collecting");
        }
        summary_.state = ActiveObservationBundleState::WaitingForMotionSettle;
        return ok("active_bundle_waiting_for_motion_settle");
    }

    ActiveMultiTofObservationResult freeze(
        std::uint64_t current_map_revision) {
        if (summary_.state != ActiveObservationBundleState::WaitingForMotionSettle) {
            return fail_no_mutation(ActiveObservationBundleFault::InvalidState,
                                    "active_bundle_freeze_requires_waiting");
        }
        if (current_map_revision != summary_.reference_map_revision) {
            abort(ActiveObservationBundleFault::ReferenceMapChangedDuringActiveBundle,
                  "reference_map_changed_during_active_bundle");
            return fail_no_mutation(
                ActiveObservationBundleFault::ReferenceMapChangedDuringActiveBundle,
                "reference_map_changed_during_active_bundle");
        }
        if (summary_.accepted_snapshot_count < config_.min_snapshots) {
            abort(ActiveObservationBundleFault::InsufficientSnapshots,
                  "active_bundle_insufficient_snapshots");
            return fail_no_mutation(ActiveObservationBundleFault::InsufficientSnapshots,
                                    "active_bundle_insufficient_snapshots");
        }
        if (summary_.matchable_ray_count < config_.min_matchable_rays) {
            abort(ActiveObservationBundleFault::InsufficientMatchableRays,
                  "active_bundle_insufficient_matchable_rays");
            return fail_no_mutation(
                ActiveObservationBundleFault::InsufficientMatchableRays,
                "active_bundle_insufficient_matchable_rays");
        }
        if (summary_.accumulated_abs_yaw_rad < config_.min_yaw_span_rad &&
            summary_.yaw_span_rad < config_.min_yaw_span_rad) {
            abort(ActiveObservationBundleFault::InsufficientYawCoverage,
                  "active_bundle_insufficient_yaw_coverage");
            return fail_no_mutation(
                ActiveObservationBundleFault::InsufficientYawCoverage,
                "active_bundle_insufficient_yaw_coverage");
        }
        ActiveMultiTofObservationBundleSummary frozen_summary = summary_;
        frozen_summary.state = ActiveObservationBundleState::FrozenReady;
        frozen_summary.complete = true;
        frozen_summary.frozen = true;
        frozen_summary.reason = "active_bundle_frozen";
        frozen_ = FrozenMultiTofObservationBundle(frozen_summary, frames_);
        frames_.clear();
        summary_ = frozen_summary;
        return ok("active_bundle_frozen");
    }

    ActiveMultiTofObservationResult discard(std::uint64_t bundle_id) {
        if (summary_.bundle_id != bundle_id) {
            return fail_no_mutation(ActiveObservationBundleFault::BundleIdMismatch,
                                    "active_bundle_id_mismatch");
        }
        frames_.clear();
        summary_ = ActiveMultiTofObservationBundleSummary{};
        frozen_ = FrozenMultiTofObservationBundle{};
        return ok("active_bundle_discarded");
    }

    ActiveMultiTofObservationResult consume(std::uint64_t bundle_id) {
        if (summary_.state != ActiveObservationBundleState::FrozenReady) {
            return fail_no_mutation(ActiveObservationBundleFault::InvalidState,
                                    "active_bundle_consume_requires_frozen");
        }
        auto result = discard(bundle_id);
        if (result.ok) {
            result.message = "active_bundle_committed_and_consumed";
        }
        return result;
    }

    void abort(ActiveObservationBundleFault fault, const std::string &reason) {
        summary_.state = ActiveObservationBundleState::Aborted;
        summary_.fault = fault;
        summary_.reason = reason;
        frames_.clear();
    }

    const ActiveMultiTofObservationBundleSummary &summary() const {
        return summary_;
    }
    const std::vector<ResolvedMultiTofObservationFrame> &frames() const {
        return frames_;
    }
    const FrozenMultiTofObservationBundle &frozen_bundle() const {
        return frozen_;
    }

private:
    ActiveMultiTofObservationResult ok(const std::string &message) const {
        return {true, ActiveObservationBundleFault::None, message};
    }

    ActiveMultiTofObservationResult fail(ActiveObservationBundleFault fault,
                                         const std::string &message) {
        summary_.fault = fault;
        summary_.reason = message;
        return {false, fault, message};
    }

    ActiveMultiTofObservationResult fail_no_mutation(
        ActiveObservationBundleFault fault,
        const std::string &message) const {
        return {false, fault, message};
    }

    ActiveMultiTofObservationResult validate_append(
        const ActiveMultiTofObservationAppendInput &input) const {
        if (!input.snapshot.has_multi_tof) {
            return {false, ActiveObservationBundleFault::MissingMultiTof,
                    "active_bundle_missing_multi_tof"};
        }
        const double t = input.snapshot.multi_tof.synchronized_time_s;
        if (!std::isfinite(t)) {
            return {false, ActiveObservationBundleFault::InvalidTimestamp,
                    "active_bundle_timestamp_invalid"};
        }
        if (std::isfinite(last_snapshot_timestamp_s_)) {
            if (t <= last_snapshot_timestamp_s_) {
                return {false, ActiveObservationBundleFault::NonMonotonicTimestamp,
                        "active_bundle_timestamp_not_increasing"};
            }
            if (t - last_snapshot_timestamp_s_ > config_.max_snapshot_gap_s) {
                return {false, ActiveObservationBundleFault::SnapshotGapExceeded,
                        "active_bundle_snapshot_gap_exceeded"};
            }
        }
        const double start = summary_.accepted_snapshot_count == 0
                                 ? t
                                 : summary_.collection_start_timestamp_s;
        if (t - start > config_.max_duration_s) {
            return {false, ActiveObservationBundleFault::DurationExceeded,
                    "active_bundle_duration_exceeded"};
        }
        if (summary_.accepted_snapshot_count + 1 > config_.max_snapshots ||
            total_ray_count(input.snapshot.multi_tof) + total_ray_count() >
                config_.max_rays) {
            return {false, ActiveObservationBundleFault::CapacityExceeded,
                    "active_bundle_capacity_exceeded"};
        }
        if (config_.require_all_measurement_poses &&
            !input.measurement_poses.complete) {
            return {false, ActiveObservationBundleFault::MissingMeasurementPose,
                    "active_bundle_measurement_pose_incomplete"};
        }
        if (!poses_valid(input)) {
            return {false, ActiveObservationBundleFault::NonFinitePose,
                    "active_bundle_measurement_pose_invalid"};
        }
        if (matchable_ray_count(input.snapshot.multi_tof) == 0) {
            return {false, ActiveObservationBundleFault::NoMatchableRay,
                    "active_bundle_no_matchable_ray"};
        }
        return ok("active_bundle_append_valid");
    }

    bool poses_valid(const ActiveMultiTofObservationAppendInput &input) const {
        return route_pose_valid(input.snapshot.multi_tof.has_front,
                                input.measurement_poses.front,
                                input.front_odom_pose_at_measurement) &&
               route_pose_valid(input.snapshot.multi_tof.has_left,
                                input.measurement_poses.left,
                                input.left_odom_pose_at_measurement) &&
               route_pose_valid(input.snapshot.multi_tof.has_right,
                                input.measurement_poses.right,
                                input.right_odom_pose_at_measurement);
    }

    static bool route_pose_valid(bool present,
                                 const TimedMapBasePose2D &map_pose,
                                 const OdomPose2D &odom_pose) {
        if (!present) {
            return true;
        }
        return map_pose.valid &&
               sparse_slam_frame_pose_valid(map_pose.base_pose_in_map) &&
               sparse_slam_frame_pose_valid(odom_pose);
    }

    static ResolvedScalarTofObservationRoute route(
        bool present,
        const ScalarTofSnapshotFrame &frame,
        const TimedMapBasePose2D &map_pose,
        const OdomPose2D &odom_pose) {
        ResolvedScalarTofObservationRoute out;
        out.present = present;
        out.frame = frame;
        out.effective_timestamp_s = frame.effective_timestamp_s;
        out.return_kind = classify_return_kind(frame);
        out.map_pose = map_pose;
        out.odom_pose_at_measurement = odom_pose;
        return out;
    }

    static ScalarTofReturnKind classify_return_kind(
        const ScalarTofSnapshotFrame &frame) {
        if (!frame.protocol_valid || !frame.usable_for_mapping ||
            !std::isfinite(frame.effective_timestamp_s)) {
            return ScalarTofReturnKind::Invalid;
        }
        if (frame.source.find("explicit_no_return") != std::string::npos) {
            return ScalarTofReturnKind::NoReturn;
        }
        return ScalarTofReturnKind::Hit;
    }

    static bool matchable(ScalarTofReturnKind kind) {
        return kind == ScalarTofReturnKind::Hit ||
               kind == ScalarTofReturnKind::NoReturn;
    }

    static std::size_t total_ray_count(const MultiTofSnapshot &snapshot) {
        return (snapshot.has_front ? 1U : 0U) +
               (snapshot.has_left ? 1U : 0U) +
               (snapshot.has_right ? 1U : 0U);
    }

    std::size_t total_ray_count() const {
        return summary_.front_ray_count + summary_.left_ray_count +
               summary_.right_ray_count;
    }

    static std::size_t matchable_ray_count(const MultiTofSnapshot &snapshot) {
        std::size_t count = 0;
        if (snapshot.has_front && matchable(classify_return_kind(snapshot.front))) {
            count++;
        }
        if (snapshot.has_left && matchable(classify_return_kind(snapshot.left))) {
            count++;
        }
        if (snapshot.has_right && matchable(classify_return_kind(snapshot.right))) {
            count++;
        }
        return count;
    }

    static void count_route(const ResolvedScalarTofObservationRoute &route,
                            std::size_t &route_count,
                            ActiveMultiTofObservationBundleSummary &summary) {
        if (!route.present) {
            return;
        }
        route_count++;
        switch (route.return_kind) {
        case ScalarTofReturnKind::Hit:
            summary.hit_ray_count++;
            summary.matchable_ray_count++;
            break;
        case ScalarTofReturnKind::NoReturn:
            summary.no_return_ray_count++;
            summary.matchable_ray_count++;
            break;
        case ScalarTofReturnKind::Invalid:
            summary.invalid_ray_count++;
            break;
        case ScalarTofReturnKind::Unspecified:
            summary.unspecified_ray_count++;
            break;
        }
    }

    void update_yaw(const OdomPose2D &odom_pose,
                    ActiveMultiTofObservationBundleSummary &summary) {
        const double yaw = odom_pose.odom_T_base.yaw_rad;
        if (!has_yaw_) {
            has_yaw_ = true;
            last_unwrapped_yaw_rad_ = yaw;
            yaw_min_rad_ = yaw;
            yaw_max_rad_ = yaw;
            summary.odom_yaw_start_rad = yaw;
            summary.odom_yaw_end_rad = yaw;
            return;
        }
        const double delta =
            sparse_slam_shortest_yaw_delta(summary.odom_yaw_end_rad, yaw);
        const double unwrapped = last_unwrapped_yaw_rad_ + delta;
        summary.accumulated_abs_yaw_rad += std::fabs(delta);
        last_unwrapped_yaw_rad_ = unwrapped;
        yaw_min_rad_ = std::min(yaw_min_rad_, unwrapped);
        yaw_max_rad_ = std::max(yaw_max_rad_, unwrapped);
        summary.yaw_span_rad = yaw_max_rad_ - yaw_min_rad_;
        summary.odom_yaw_end_rad = yaw;
    }

    ActiveMultiTofObservationBundleConfig config_;
    ActiveMultiTofObservationBundleSummary summary_;
    std::vector<ResolvedMultiTofObservationFrame> frames_;
    FrozenMultiTofObservationBundle frozen_;
    double last_snapshot_timestamp_s_ =
        std::numeric_limits<double>::quiet_NaN();
    double last_unwrapped_yaw_rad_ = 0.0;
    double yaw_min_rad_ = 0.0;
    double yaw_max_rad_ = 0.0;
    bool has_yaw_ = false;
};

} // namespace robot_slamd
