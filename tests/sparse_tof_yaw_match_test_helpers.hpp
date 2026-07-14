#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_matcher.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace yaw_match_test {

using namespace robot_slamd;

inline void expect(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

inline ResolvedScalarTofObservationRoute route(
    ScalarTofReturnKind kind, double range_m, double mount_yaw_rad,
    double timestamp_s, const OdomPose2D &odom_pose, const char *id) {
    ResolvedScalarTofObservationRoute out;
    out.present = true;
    out.frame.distance_m = range_m;
    out.frame.distance_mm = static_cast<std::uint16_t>(range_m * 1000.0);
    out.frame.confidence = 100;
    out.frame.echo_tag_u48 = 0xB10000 + out.frame.distance_mm;
    out.frame.effective_timestamp_s = timestamp_s;
    out.frame.protocol_valid = true;
    out.frame.usable_for_mapping = true;
    out.frame.mount_yaw_rad = mount_yaw_rad;
    out.frame.frame_id = id;
    out.frame.source = "yaw_match_test";
    out.effective_timestamp_s = timestamp_s;
    out.return_kind = kind;
    out.odom_pose_at_measurement = odom_pose;
    out.map_pose.valid = true;
    out.map_pose.measurement_timestamp_s = timestamp_s;
    out.map_pose.base_pose_in_map = compose(identity_map_from_odom(), odom_pose);
    return out;
}

inline std::vector<ResolvedMultiTofObservationFrame> asymmetric_frames(
    bool include_no_return = false,
    bool include_ignored = false) {
    const std::vector<RobotPose2D> poses{
        {0.00, 0.00, 0.00}, {0.08, 0.02, 0.04},
        {0.16, 0.05, 0.09}, {0.12, 0.12, 0.14},
        {0.02, 0.15, 0.08}, {-0.05, 0.08, -0.03}};
    const double front[]{2.00, 1.75, 2.25, 1.60, 2.35, 1.90};
    const double left[]{1.10, 1.45, 1.20, 1.65, 1.30, 1.55};
    const double right[]{2.75, 2.40, 2.95, 2.20, 2.65, 2.30};
    std::vector<ResolvedMultiTofObservationFrame> frames;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        const double t = 1.0 + static_cast<double>(i) * 0.1;
        const auto odom = make_odom_pose(poses[i]);
        ResolvedMultiTofObservationFrame f;
        f.snapshot_timestamp_s = t;
        f.sequence_index = i;
        f.source = "yaw_match_test";
        f.front = route(ScalarTofReturnKind::Hit, front[i], 0.0, t, odom, "front");
        f.left = route(ScalarTofReturnKind::Hit, left[i], 1.5707963267948966,
                       t, odom, "left");
        f.right = route(include_no_return && i % 2U == 0
                            ? ScalarTofReturnKind::NoReturn
                            : ScalarTofReturnKind::Hit,
                        right[i], -1.5707963267948966, t, odom, "right");
        if (include_ignored && i == 0) {
            f.front.return_kind = ScalarTofReturnKind::Invalid;
            f.left.return_kind = ScalarTofReturnKind::Unspecified;
        }
        f.complete = true;
        frames.push_back(f);
    }
    return frames;
}

inline std::vector<ResolvedMultiTofObservationFrame> symmetric_frames() {
    std::vector<ResolvedMultiTofObservationFrame> frames;
    for (std::size_t i = 0; i < 6; ++i) {
        const double t = 2.0 + static_cast<double>(i) * 0.1;
        const double yaw = i % 2U == 0 ? 0.0 : 3.14159265358979323846;
        const auto odom = make_odom_pose({0.0, 0.0, yaw});
        ResolvedMultiTofObservationFrame f;
        f.snapshot_timestamp_s = t;
        f.sequence_index = i;
        f.source = "symmetric_yaw_match_test";
        f.front = route(ScalarTofReturnKind::Hit, 1.5, 0.0, t, odom, "front");
        f.left = route(ScalarTofReturnKind::Hit, 1.5, 1.5707963267948966,
                       t, odom, "left");
        f.right = route(ScalarTofReturnKind::Hit, 1.5, -1.5707963267948966,
                        t, odom, "right");
        f.complete = true;
        frames.push_back(f);
    }
    return frames;
}

inline SparseTofRayObservation map_observation(
    const ResolvedScalarTofObservationRoute &route,
    double no_return_range_m,
    const MapFromOdom2D &true_map_from_odom) {
    const auto base = compose(true_map_from_odom,
                              route.odom_pose_at_measurement);
    SparseTofRayObservation out;
    out.return_kind = route.return_kind;
    out.sensor_origin_x_m = base.map_T_base.x_m;
    out.sensor_origin_y_m = base.map_T_base.y_m;
    out.ray_yaw_rad = sparse_slam_normalize_yaw(
        base.map_T_base.yaw_rad + route.frame.mount_yaw_rad);
    out.measured_range_m = route.frame.distance_m;
    out.free_space_range_m = route.return_kind == ScalarTofReturnKind::NoReturn
                                 ? no_return_range_m
                                 : route.frame.distance_m;
    out.confidence = route.frame.confidence;
    out.protocol_valid = true;
    out.synchronized = true;
    out.usable_for_mapping = route.return_kind == ScalarTofReturnKind::Hit ||
                             route.return_kind == ScalarTofReturnKind::NoReturn;
    return out;
}

inline SparseOccupancyGridSnapshot reference_map(
    const std::vector<ResolvedMultiTofObservationFrame> &frames,
    double no_return_range_m = 3.0,
    const MapFromOdom2D &true_map_from_odom = identity_map_from_odom()) {
    SparseOccupancyGridConfig config;
    config.resolution_m = 0.05;
    config.maximum_mapping_range_m = 12.1;
    config.maximum_ray_cells = 512;
    SparseOccupancyGrid grid(config);
    std::vector<SparseTofRayObservation> observations;
    for (const auto &f : frames) {
        observations.push_back(map_observation(f.front, no_return_range_m, true_map_from_odom));
        observations.push_back(map_observation(f.left, no_return_range_m, true_map_from_odom));
        observations.push_back(map_observation(f.right, no_return_range_m, true_map_from_odom));
    }
    for (int i = 0; i < 3; ++i) {
        expect(grid.apply_observations(observations).accepted,
               "reference map update");
    }
    const auto captured = grid.capture_snapshot(11, 100000);
    expect(captured.ok, "reference map capture");
    return captured.snapshot;
}

inline FrozenMultiTofObservationBundle frozen_bundle(
    std::vector<ResolvedMultiTofObservationFrame> frames,
    std::size_t matchable_rays) {
    ActiveMultiTofObservationBundleSummary summary;
    summary.state = ActiveObservationBundleState::FrozenReady;
    summary.bundle_id = 41;
    summary.collection_start_timestamp_s = frames.front().snapshot_timestamp_s;
    summary.collection_end_timestamp_s = frames.back().snapshot_timestamp_s;
    summary.duration_s = summary.collection_end_timestamp_s -
                         summary.collection_start_timestamp_s;
    summary.reference_map_revision = 11;
    summary.accepted_snapshot_count = frames.size();
    summary.matchable_ray_count = matchable_rays;
    summary.complete = true;
    summary.frozen = true;
    return FrozenMultiTofObservationBundle(summary, std::move(frames));
}

inline SparseTofLocalMatchInput input_from_frames(
    const std::vector<ResolvedMultiTofObservationFrame> &frames,
    double predicted_yaw_error_rad,
    double no_return_range_m = 3.0) {
    SparseTofLocalMatchInput input;
    std::size_t matchable = 0;
    for (const auto &f : frames) {
        for (const auto *r : {&f.front, &f.left, &f.right}) {
            if (r->return_kind == ScalarTofReturnKind::Hit ||
                r->return_kind == ScalarTofReturnKind::NoReturn) matchable++;
        }
    }
    input.bundle_id = 41;
    input.frozen_bundle = std::make_shared<const FrozenMultiTofObservationBundle>(
        frozen_bundle(frames, matchable));
    input.reference_map = std::make_shared<const SparseOccupancyGridSnapshot>(
        reference_map(frames, no_return_range_m,
                      make_map_from_odom({0.35, -0.20, 0.0})));
    input.expected_reference_map_revision = 11;
    input.current_runtime_map_revision = 11;
    input.predicted_map_from_odom =
        make_map_from_odom({0.35, -0.20, predicted_yaw_error_rad});
    input.map_from_odom_at_collection_start = identity_map_from_odom();
    input.match_request_timestamp_s = frames.back().snapshot_timestamp_s;
    input.source = "yaw_match_test";
    input.config.max_abs_yaw_rad = 0.20;
    input.config.coarse_yaw_step_rad = 0.04;
    input.config.fine_yaw_step_rad = 0.01;
    input.config.max_coarse_candidates = 16;
    input.config.max_fine_candidates = 16;
    input.config.max_total_candidates = 32;
    input.config.max_candidate_count = 32;
    input.config.max_scored_rays = 64;
    input.config.max_cells_per_ray = 256;
    input.config.min_bundle_frames = 2;
    input.config.min_matchable_rays = 3;
    input.config.min_reference_cells = 1;
    input.config.min_reference_occupied_cells = 1;
    input.config.min_reference_free_cells = 1;
    input.config.no_return_free_space_range_m = no_return_range_m;
    input.config.min_used_rays = 3;
    input.config.min_known_cells = 3;
    input.config.max_unknown_ratio = 0.98;
    input.config.max_contradiction_ratio = 0.95;
    input.config.minimum_normalized_score = -1.0;
    input.config.minimum_score_margin = 0.002;
    input.config.minimum_score_range = 0.001;
    input.config.runner_up_exclusion_yaw_rad = 0.05;
    input.config.multimodal_max_score_drop = 0.001;
    return input;
}

} // namespace yaw_match_test
