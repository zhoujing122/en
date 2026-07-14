#pragma once

#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_observation_builder.hpp"
#include "robot_slamd/runtime/active_multi_tof_observation_types.hpp"

#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

struct SparseTofKeyframeMapTransaction {
    bool ready = false;
    std::uint64_t bundle_id = 0;
    std::uint64_t reference_map_revision = 0;
    MapFromOdom2D proposed_map_from_odom = identity_map_from_odom();
    SparseOccupancyGridPreparedBatch map_batch;
    SparseOccupancyGridBatchStats stats;
};

struct SparseTofKeyframeMapPrepareResult {
    bool ok = false;
    SparseTofKeyframeMapTransaction transaction;
    std::string reason;
};

class SparseTofKeyframeMapTransactionBuilder {
public:
    explicit SparseTofKeyframeMapTransactionBuilder(
        SparseTofObservationBuilderOptions options = {},
        bool use_planar_tof_extrinsics = false,
        PlanarThreeTofExtrinsics planar_tof_extrinsics = {})
        : observation_builder_(options),
          use_planar_tof_extrinsics_(use_planar_tof_extrinsics),
          planar_tof_extrinsics_(planar_tof_extrinsics) {}

    SparseTofKeyframeMapPrepareResult prepare(
        const FrozenMultiTofObservationBundle &bundle,
        const MapFromOdom2D &proposed_map_from_odom,
        std::uint64_t expected_reference_revision,
        std::uint64_t current_map_revision,
        const SparseOccupancyGrid &grid,
        std::size_t maximum_changed_cells) const {
        SparseTofKeyframeMapPrepareResult result;
        if (!bundle.available() ||
            bundle.summary().bundle_id == 0 ||
            bundle.summary().reference_map_revision !=
                expected_reference_revision ||
            expected_reference_revision != current_map_revision ||
            !sparse_slam_frame_pose_valid(proposed_map_from_odom)) {
            result.reason = "keyframe_map_transaction_contract_rejected";
            return result;
        }

        std::vector<std::vector<SparseTofRayObservation>> batches;
        batches.reserve(bundle.frames().size());
        for (const auto &frame : bundle.frames()) {
            std::vector<SparseTofRayObservation> observations;
            observations.reserve(3);
            if (!append_route(frame.front, PlanarTofRoute::Front, proposed_map_from_odom,
                              frame.snapshot_timestamp_s, observations) ||
                !append_route(frame.left, PlanarTofRoute::Left, proposed_map_from_odom,
                              frame.snapshot_timestamp_s, observations) ||
                !append_route(frame.right, PlanarTofRoute::Right, proposed_map_from_odom,
                              frame.snapshot_timestamp_s, observations)) {
                result.reason = "keyframe_corrected_observation_invalid";
                return result;
            }
            if (!observations.empty()) {
                batches.push_back(std::move(observations));
            }
        }
        if (batches.empty()) {
            result.reason = "keyframe_map_transaction_no_observations";
            return result;
        }

        auto prepared =
            grid.prepare_observation_batch(batches, maximum_changed_cells);
        if (!prepared.ok) {
            result.reason = prepared.message;
            return result;
        }
        result.transaction.ready = true;
        result.transaction.bundle_id = bundle.summary().bundle_id;
        result.transaction.reference_map_revision =
            expected_reference_revision;
        result.transaction.proposed_map_from_odom = proposed_map_from_odom;
        result.transaction.stats = prepared.prepared.stats();
        result.transaction.map_batch = std::move(prepared.prepared);
        result.ok = true;
        result.reason = "keyframe_map_transaction_prepared";
        return result;
    }

private:
    bool append_route(
        const ResolvedScalarTofObservationRoute &route,
        PlanarTofRoute sensor_route,
        const MapFromOdom2D &proposed_map_from_odom,
        double now_s,
        std::vector<SparseTofRayObservation> &observations) const {
        if (!route.present ||
            route.return_kind == ScalarTofReturnKind::Invalid ||
            route.return_kind == ScalarTofReturnKind::Unspecified) {
            return true;
        }
        if (!sparse_slam_frame_pose_valid(route.odom_pose_at_measurement)) {
            return false;
        }
        const auto corrected_base =
            compose(proposed_map_from_odom, route.odom_pose_at_measurement);
        SparseTofObservationBuildInput input;
        input.frame = route.frame;
        input.estimated_pose = corrected_base.map_T_base;
        input.has_planar_extrinsic = use_planar_tof_extrinsics_;
        input.planar_extrinsic = planar_tof_extrinsic_for(
            planar_tof_extrinsics_, sensor_route);
        input.explicit_return_kind = route.return_kind;
        input.synchronized = true;
        input.now_s = now_s;
        const auto built = observation_builder_.build(input);
        if (!built.ok ||
            built.observation.return_kind != route.return_kind) {
            return false;
        }
        observations.push_back(built.observation);
        return true;
    }

    SparseTofObservationBuilder observation_builder_;
    bool use_planar_tof_extrinsics_ = false;
    PlanarThreeTofExtrinsics planar_tof_extrinsics_;
};

} // namespace robot_slamd
