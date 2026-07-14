#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_binding.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_keyframe_transaction.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_observation_builder.hpp"

#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace robot_slamd {

enum class SparseMultiTofBackendFault {
    None,
    BackendNotReady,
    SnapshotMissing,
    MultiTofMissing,
    NoUsableRay,
    PoseUnavailable,
    PoseInvalid,
    TimestampInvalid,
    SyncRejected,
    ReturnSemanticInvalid,
    RayTraversalFailed,
    MapCapacityReached,
    MapUpdateRejected
};

inline std::string to_string(SparseMultiTofBackendFault fault) {
    switch (fault) {
    case SparseMultiTofBackendFault::None:
        return "none";
    case SparseMultiTofBackendFault::BackendNotReady:
        return "backend_not_ready";
    case SparseMultiTofBackendFault::SnapshotMissing:
        return "snapshot_missing";
    case SparseMultiTofBackendFault::MultiTofMissing:
        return "multi_tof_missing";
    case SparseMultiTofBackendFault::NoUsableRay:
        return "no_usable_ray";
    case SparseMultiTofBackendFault::PoseUnavailable:
        return "pose_unavailable";
    case SparseMultiTofBackendFault::PoseInvalid:
        return "pose_invalid";
    case SparseMultiTofBackendFault::TimestampInvalid:
        return "timestamp_invalid";
    case SparseMultiTofBackendFault::SyncRejected:
        return "sync_rejected";
    case SparseMultiTofBackendFault::ReturnSemanticInvalid:
        return "return_semantic_invalid";
    case SparseMultiTofBackendFault::RayTraversalFailed:
        return "ray_traversal_failed";
    case SparseMultiTofBackendFault::MapCapacityReached:
        return "map_capacity_reached";
    case SparseMultiTofBackendFault::MapUpdateRejected:
        return "map_update_rejected";
    }
    return "unknown";
}

struct SparseMultiTofOccupancyBackendOptions {
    bool enabled = true;
    bool ready = true;
    int minimum_accepted_snapshots_for_good_quality = 3;
    int minimum_valid_rays_for_good_quality = 6;
    int minimum_observed_cells_for_good_quality = 8;
    int angular_bin_count = 16;
    int minimum_angular_bins_for_good_quality = 2;
    SparseOccupancyGridConfig grid;
    SparseTofObservationBuilderOptions observation_builder;
    bool use_planar_tof_extrinsics = false;
    PlanarThreeTofExtrinsics planar_tof_extrinsics;
    bool require_multi_tof_measurement_poses = false;
    bool allow_single_pose_fallback = true;
    double measurement_pose_timestamp_tolerance_s = 1e-6;
};

struct SparseMultiTofOccupancyBackendReport {
    bool ok = false;
    bool native_multi_tof_backend_consumption = false;
    bool legacy_projection_consumed = false;
    bool ground_truth_consumed = false;
    bool measurement_time_pose_consumed = false;
    bool single_pose_fallback_consumed = false;
    int snapshot_update_attempt_count = 0;
    int accepted_snapshot_count = 0;
    int rejected_snapshot_count = 0;
    int front_ray_count = 0;
    int left_ray_count = 0;
    int right_ray_count = 0;
    int hit_ray_count = 0;
    int no_return_ray_count = 0;
    int invalid_ray_count = 0;
    int free_cell_update_count = 0;
    int occupied_cell_update_count = 0;
    int duplicate_cell_suppression_count = 0;
    int occupied_over_free_resolution_count = 0;
    std::size_t active_cell_count = 0;
    std::size_t free_cell_count = 0;
    std::size_t occupied_cell_count = 0;
    std::size_t uncertain_cell_count = 0;
    double map_capacity_ratio = 0.0;
    double map_readiness_score = 0.0;
    int angular_coverage_bins = 0;
    int accepted_pose_update_count = 0;
    int rejected_pose_update_count = 0;
    bool transactionality_verified = true;
    bool deterministic_output_verified = true;
    bool capacity_bound_verified = true;
    bool ground_truth_isolation_verified = true;
    bool real_hardware_accessed = false;
    bool real_motion_attempted = false;
    bool real_map_write_attempted = false;
    bool real_pose_writeback_attempted = false;
    SparseMultiTofBackendFault last_fault = SparseMultiTofBackendFault::None;
    std::string last_message;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class SparseMultiTofOccupancyBackendBinding final : public SlamBackendBinding {
public:
    explicit SparseMultiTofOccupancyBackendBinding(
        SparseMultiTofOccupancyBackendOptions options = {})
        : options_(options),
          grid_(options.grid),
          builder_(options.observation_builder) {}

    bool ready() const override {
        return options_.enabled && options_.ready && grid_.valid();
    }

    SlamBackendUpdateResult update(
        const SlamBackendInputFrame &input) override {
        report_.snapshot_update_attempt_count++;
        if (!options_.enabled || !ready()) {
            return reject(SparseMultiTofBackendFault::BackendNotReady,
                          SlamBackendFault::BackendNotReady,
                          "sparse_multi_tof_backend_not_ready");
        }
        if (!std::isfinite(input.timestamp_s)) {
            return reject(SparseMultiTofBackendFault::TimestampInvalid,
                          SlamBackendFault::InvalidTimestamp,
                          "sparse_multi_tof_timestamp_invalid");
        }
        if (!snapshot_has_any_payload(input.snapshot)) {
            return reject(SparseMultiTofBackendFault::SnapshotMissing,
                          SlamBackendFault::InvalidTofScan,
                          "sparse_multi_tof_snapshot_missing");
        }
        if (!input.snapshot.has_multi_tof) {
            return reject(SparseMultiTofBackendFault::MultiTofMissing,
                          SlamBackendFault::MissingTof,
                          "sparse_multi_tof_missing");
        }
        if (!input.has_predicted_pose) {
            report_.rejected_pose_update_count++;
            return reject(SparseMultiTofBackendFault::PoseUnavailable,
                          SlamBackendFault::UpdateRejected,
                          "sparse_multi_tof_pose_unavailable");
        }
        if (!sparse_pose_finite(input.predicted_pose)) {
            report_.rejected_pose_update_count++;
            return reject(SparseMultiTofBackendFault::PoseInvalid,
                          SlamBackendFault::UpdateRejected,
                          "sparse_multi_tof_pose_invalid");
        }

        const bool use_measurement_poses =
            input.has_multi_tof_measurement_poses &&
            input.multi_tof_measurement_poses.complete;
        if (!use_measurement_poses &&
            options_.require_multi_tof_measurement_poses) {
            report_.rejected_pose_update_count++;
            return reject(SparseMultiTofBackendFault::PoseUnavailable,
                          SlamBackendFault::UpdateRejected,
                          "sparse_multi_tof_measurement_pose_unavailable");
        }
        if (!use_measurement_poses &&
            !options_.allow_single_pose_fallback) {
            report_.rejected_pose_update_count++;
            return reject(SparseMultiTofBackendFault::PoseUnavailable,
                          SlamBackendFault::UpdateRejected,
                          "sparse_multi_tof_single_pose_fallback_disabled");
        }
        if (use_measurement_poses &&
            !measurement_pose_set_valid(input)) {
            report_.rejected_pose_update_count++;
            return reject(SparseMultiTofBackendFault::PoseInvalid,
                          SlamBackendFault::UpdateRejected,
                          "sparse_multi_tof_measurement_pose_invalid");
        }

        std::vector<SparseTofRayObservation> observations;
        observations.reserve(3);
        build_if_present(input, "front", PlanarTofRoute::Front, input.snapshot.multi_tof.has_front,
                         input.snapshot.multi_tof.front,
                         pose_for_front(input, use_measurement_poses),
                         observations);
        build_if_present(input, "left", PlanarTofRoute::Left, input.snapshot.multi_tof.has_left,
                         input.snapshot.multi_tof.left,
                         pose_for_left(input, use_measurement_poses),
                         observations);
        build_if_present(input, "right", PlanarTofRoute::Right, input.snapshot.multi_tof.has_right,
                         input.snapshot.multi_tof.right,
                         pose_for_right(input, use_measurement_poses),
                         observations);

        const auto stats = grid_.apply_observations(observations);
        if (!stats.accepted) {
            return reject(map_fault(stats.fault),
                          SlamBackendFault::UpdateRejected,
                          stats.message);
        }

        report_.accepted_snapshot_count++;
        report_.accepted_pose_update_count++;
        report_.native_multi_tof_backend_consumption = true;
        report_.legacy_projection_consumed = false;
        report_.ground_truth_consumed = false;
        report_.measurement_time_pose_consumed = use_measurement_poses;
        report_.single_pose_fallback_consumed = !use_measurement_poses;
        report_.hit_ray_count += stats.hit_ray_count;
        report_.no_return_ray_count += stats.no_return_ray_count;
        report_.invalid_ray_count += stats.invalid_ray_count;
        report_.free_cell_update_count += stats.free_cell_update_count;
        report_.occupied_cell_update_count += stats.occupied_cell_update_count;
        report_.duplicate_cell_suppression_count +=
            stats.duplicate_cell_suppression_count;
        report_.occupied_over_free_resolution_count +=
            stats.occupied_over_free_resolution_count;
        const auto snapshot = grid_.snapshot();
        report_.active_cell_count = snapshot.cell_count();
        report_.free_cell_count = snapshot.free_cell_count();
        report_.occupied_cell_count = snapshot.occupied_cell_count();
        report_.uncertain_cell_count = snapshot.uncertain_cell_count();
        report_.map_capacity_ratio =
            options_.grid.maximum_active_cells == 0
                ? 1.0
                : static_cast<double>(report_.active_cell_count) /
                      static_cast<double>(options_.grid.maximum_active_cells);
        update_quality();
        report_.ok = true;
        report_.last_fault = SparseMultiTofBackendFault::None;
        report_.last_message = "sparse_multi_tof_update_accepted";
        report_.summary = "sparse_multi_tof_backend_ok";

        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Accepted;
        result.fault = SlamBackendFault::None;
        result.map_updated = true;
        result.keyframe_added = report_.accepted_snapshot_count == 1;
        result.integrated_scan_count = report_.accepted_snapshot_count;
        result.update_latency_s = 0.0;
        result.quality = latest_quality_;
        result.message = "sparse_multi_tof_update_accepted";
        return result;
    }

    RobotSlamMapQuality latest_quality(double now_s) const override {
        (void)now_s;
        return latest_quality_;
    }

    SlamBackendSaveResult save_map(const std::string &output_path) override {
        SlamBackendSaveResult result;
        result.output_path = output_path;
        result.ok = false;
        result.error = "sparse_multi_tof_save_not_enabled";
        return result;
    }

    std::string status() const override {
        std::ostringstream out;
        out << "SparseMultiTofOccupancyBackendBinding ready="
            << (ready() ? 1 : 0)
            << " accepted=" << report_.accepted_snapshot_count
            << " rejected=" << report_.rejected_snapshot_count
            << " cells=" << report_.active_cell_count
            << " fault=" << to_string(report_.last_fault);
        return out.str();
    }

    const SparseMultiTofOccupancyBackendReport &report() const {
        return report_;
    }

    SparseOccupancyGridSnapshot grid_snapshot() const {
        return grid_.snapshot();
    }

    SparseOccupancyGridSnapshotCaptureResult capture_grid_snapshot(
        std::uint64_t revision,
        std::size_t maximum_snapshot_cells) const {
        return grid_.capture_snapshot(revision, maximum_snapshot_cells);
    }

    SparseTofKeyframeMapPrepareResult prepare_keyframe_map_transaction(
        const FrozenMultiTofObservationBundle &bundle,
        const MapFromOdom2D &proposed_map_from_odom,
        std::uint64_t expected_reference_revision,
        std::uint64_t current_map_revision,
        std::size_t maximum_changed_cells) const {
        const SparseTofKeyframeMapTransactionBuilder transaction_builder(
            options_.observation_builder,
            options_.use_planar_tof_extrinsics,
            options_.planar_tof_extrinsics);
        return transaction_builder.prepare(
            bundle, proposed_map_from_odom, expected_reference_revision,
            current_map_revision, grid_, maximum_changed_cells);
    }

    bool commit_keyframe_map_transaction(
        SparseTofKeyframeMapTransaction &&transaction) noexcept {
        if (!transaction.ready ||
            !grid_.commit_prepared_batch(std::move(transaction.map_batch))) {
            return false;
        }
        report_.active_cell_count = transaction.stats.final_cell_count;
        report_.free_cell_count = transaction.stats.final_free_cell_count;
        report_.occupied_cell_count =
            transaction.stats.final_occupied_cell_count;
        report_.uncertain_cell_count =
            transaction.stats.final_uncertain_cell_count;
        report_.hit_ray_count +=
            static_cast<int>(transaction.stats.hit_ray_count);
        report_.no_return_ray_count +=
            static_cast<int>(transaction.stats.no_return_ray_count);
        report_.free_cell_update_count +=
            static_cast<int>(transaction.stats.free_cell_update_count);
        report_.occupied_cell_update_count +=
            static_cast<int>(transaction.stats.occupied_cell_update_count);
        report_.map_capacity_ratio =
            options_.grid.maximum_active_cells == 0
                ? 1.0
                : static_cast<double>(report_.active_cell_count) /
                      static_cast<double>(options_.grid.maximum_active_cells);
        transaction.ready = false;
        return true;
    }

private:
    bool measurement_pose_set_valid(const SlamBackendInputFrame &input) const {
        return measurement_pose_valid(input.multi_tof_measurement_poses.front,
                                      input.snapshot.multi_tof.front) &&
               measurement_pose_valid(input.multi_tof_measurement_poses.left,
                                      input.snapshot.multi_tof.left) &&
               measurement_pose_valid(input.multi_tof_measurement_poses.right,
                                      input.snapshot.multi_tof.right);
    }

    bool measurement_pose_valid(const TimedMapBasePose2D &pose,
                                const ScalarTofSnapshotFrame &frame) const {
        return pose.valid &&
               sparse_slam_frame_pose_valid(pose.base_pose_in_map) &&
               std::isfinite(pose.measurement_timestamp_s) &&
               std::fabs(pose.measurement_timestamp_s -
                         frame.effective_timestamp_s) <=
                   options_.measurement_pose_timestamp_tolerance_s;
    }

    RobotPose2D pose_for_front(const SlamBackendInputFrame &input,
                               bool use_measurement_poses) const {
        return use_measurement_poses
                   ? input.multi_tof_measurement_poses.front.base_pose_in_map.map_T_base
                   : input.predicted_pose;
    }

    RobotPose2D pose_for_left(const SlamBackendInputFrame &input,
                              bool use_measurement_poses) const {
        return use_measurement_poses
                   ? input.multi_tof_measurement_poses.left.base_pose_in_map.map_T_base
                   : input.predicted_pose;
    }

    RobotPose2D pose_for_right(const SlamBackendInputFrame &input,
                               bool use_measurement_poses) const {
        return use_measurement_poses
                   ? input.multi_tof_measurement_poses.right.base_pose_in_map.map_T_base
                   : input.predicted_pose;
    }

    void build_if_present(const SlamBackendInputFrame &input,
                          const std::string &name,
                          PlanarTofRoute sensor_route,
                          bool present,
                          const ScalarTofSnapshotFrame &frame,
                          const RobotPose2D &pose,
                          std::vector<SparseTofRayObservation> &observations) {
        if (!present) {
            return;
        }
        SparseTofObservationBuildInput build;
        build.frame = frame;
        build.estimated_pose = pose;
        build.has_planar_extrinsic = options_.use_planar_tof_extrinsics;
        build.planar_extrinsic = planar_tof_extrinsic_for(
            options_.planar_tof_extrinsics, sensor_route);
        build.now_s = input.timestamp_s;
        build.synchronized = true;
        const auto built = builder_.build(build);
        if (!built.ok) {
            report_.invalid_ray_count++;
            return;
        }
        if (name == "front") {
            report_.front_ray_count++;
        } else if (name == "left") {
            report_.left_ray_count++;
        } else {
            report_.right_ray_count++;
        }
        observations.push_back(built.observation);
        mark_angular_bin(built.observation.ray_yaw_rad);
    }

    void mark_angular_bin(double yaw_rad) {
        if (options_.angular_bin_count <= 0 || !std::isfinite(yaw_rad)) {
            return;
        }
        const double pi = 3.14159265358979323846;
        double normalized = std::fmod(yaw_rad + pi, 2.0 * pi);
        if (normalized < 0.0) {
            normalized += 2.0 * pi;
        }
        const int bin = static_cast<int>(
            std::floor(normalized / (2.0 * pi) *
                       static_cast<double>(options_.angular_bin_count)));
        if (bin >= 0 && bin < options_.angular_bin_count) {
            if (static_cast<int>(angular_bins_.size()) != options_.angular_bin_count) {
                angular_bins_.assign(static_cast<std::size_t>(
                                         options_.angular_bin_count),
                                     false);
            }
            angular_bins_[static_cast<std::size_t>(bin)] = true;
        }
    }

    void update_quality() {
        int bins = 0;
        for (bool seen : angular_bins_) {
            bins += seen ? 1 : 0;
        }
        report_.angular_coverage_bins = bins;
        const int valid_rays = report_.hit_ray_count + report_.no_return_ray_count;
        const double snapshot_score =
            ratio(report_.accepted_snapshot_count,
                  options_.minimum_accepted_snapshots_for_good_quality);
        const double ray_score =
            ratio(valid_rays, options_.minimum_valid_rays_for_good_quality);
        const double cell_score =
            ratio(static_cast<int>(report_.active_cell_count),
                  options_.minimum_observed_cells_for_good_quality);
        const double angle_score =
            ratio(bins, options_.minimum_angular_bins_for_good_quality);
        report_.map_readiness_score =
            std::min(std::min(snapshot_score, ray_score),
                     std::min(cell_score, angle_score));

        latest_quality_.coverage_ratio = report_.map_readiness_score;
        latest_quality_.yaw_coverage_ratio = angle_score;
        latest_quality_.valid_scan_count = report_.accepted_snapshot_count;
        latest_quality_.good_enough =
            report_.accepted_snapshot_count >=
                options_.minimum_accepted_snapshots_for_good_quality &&
            valid_rays >= options_.minimum_valid_rays_for_good_quality &&
            static_cast<int>(report_.active_cell_count) >=
                options_.minimum_observed_cells_for_good_quality &&
            bins >= options_.minimum_angular_bins_for_good_quality &&
            report_.map_capacity_ratio < 1.0;
        latest_quality_.reason = latest_quality_.good_enough
                                      ? "sparse_multi_tof_map_ready"
                                      : "sparse_multi_tof_map_not_ready";
    }

    double ratio(int value, int required) const {
        if (required <= 0) {
            return 1.0;
        }
        const double r = static_cast<double>(value) /
                         static_cast<double>(required);
        return r > 1.0 ? 1.0 : r;
    }

    SparseMultiTofBackendFault map_fault(
        SparseOccupancyGridFault fault) const {
        switch (fault) {
        case SparseOccupancyGridFault::None:
            return SparseMultiTofBackendFault::None;
        case SparseOccupancyGridFault::NoUsableRay:
            return SparseMultiTofBackendFault::NoUsableRay;
        case SparseOccupancyGridFault::RayTraversalFailed:
            return SparseMultiTofBackendFault::RayTraversalFailed;
        case SparseOccupancyGridFault::MapCapacityReached:
            return SparseMultiTofBackendFault::MapCapacityReached;
        case SparseOccupancyGridFault::InvalidObservation:
            return SparseMultiTofBackendFault::ReturnSemanticInvalid;
        case SparseOccupancyGridFault::InvalidConfig:
        case SparseOccupancyGridFault::InvalidPose:
        case SparseOccupancyGridFault::EvidenceOverflow:
            return SparseMultiTofBackendFault::MapUpdateRejected;
        }
        return SparseMultiTofBackendFault::MapUpdateRejected;
    }

    SlamBackendUpdateResult reject(SparseMultiTofBackendFault fault,
                                   SlamBackendFault slam_fault,
                                   const std::string &message) {
        report_.rejected_snapshot_count++;
        report_.ok = false;
        report_.last_fault = fault;
        report_.last_message = message;
        report_.summary = message;
        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Rejected;
        result.fault = slam_fault;
        result.map_updated = false;
        result.integrated_scan_count = report_.accepted_snapshot_count;
        result.quality = latest_quality_;
        result.message = message;
        return result;
    }

    SparseMultiTofOccupancyBackendOptions options_;
    SparseOccupancyGrid grid_;
    SparseTofObservationBuilder builder_;
    SparseMultiTofOccupancyBackendReport report_;
    RobotSlamMapQuality latest_quality_;
    std::vector<bool> angular_bins_;
};

inline std::string write_sparse_multi_tof_backend_report(
    const SparseMultiTofOccupancyBackendReport &report) {
    std::ostringstream out;
    out << "sparse three-ToF occupancy backend\n";
    out << "native_multi_tof_backend_consumption="
        << (report.native_multi_tof_backend_consumption ? "true" : "false")
        << "\n";
    out << "legacy_projection_consumed="
        << (report.legacy_projection_consumed ? "true" : "false")
        << "\n";
    out << "ground_truth_consumed="
        << (report.ground_truth_consumed ? "true" : "false") << "\n";
    out << "hit_ray_count=" << report.hit_ray_count << "\n";
    out << "no_return_ray_count=" << report.no_return_ray_count << "\n";
    out << "active_cell_count=" << report.active_cell_count << "\n";
    out << "map_readiness_score=" << report.map_readiness_score << "\n";
    out << "scan matching not implemented\n";
    out << "frontier and A* not implemented\n";
    out << "real hardware not accessed\n";
    return out.str();
}

} // namespace robot_slamd
