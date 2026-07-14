#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_match_types.hpp"

#include <cmath>

namespace robot_slamd {

inline bool sparse_tof_local_match_config_valid(
    const SparseTofLocalMatchConfig &config) {
    constexpr std::size_t kCandidateHardLimit = 100000;
    const bool base =
        std::isfinite(config.max_abs_yaw_rad) &&
        config.max_abs_yaw_rad >= 0.0 &&
        config.max_abs_yaw_rad <= 3.14159265358979323846 &&
        std::isfinite(config.coarse_yaw_step_rad) &&
        config.coarse_yaw_step_rad > 0.0 &&
        std::isfinite(config.fine_yaw_step_rad) &&
        config.fine_yaw_step_rad > 0.0 &&
        config.fine_yaw_step_rad <= config.coarse_yaw_step_rad &&
        std::isfinite(config.max_abs_translation_x_m) &&
        config.max_abs_translation_x_m >= 0.0 &&
        std::isfinite(config.max_abs_translation_y_m) &&
        config.max_abs_translation_y_m >= 0.0 &&
        config.max_candidate_count > 0 &&
        config.max_candidate_count <= kCandidateHardLimit &&
        config.min_bundle_frames > 0 &&
        config.min_matchable_rays > 0 &&
        config.min_reference_cells > 0 &&
        std::isfinite(config.max_bundle_duration_s) &&
        config.max_bundle_duration_s > 0.0;
    if (!base) {
        return false;
    }
    if (config.mode == SparseTofLocalMatchMode::YawOnly &&
        (config.max_abs_translation_x_m != 0.0 ||
         config.max_abs_translation_y_m != 0.0)) {
        return false;
    }
    return config.min_reference_occupied_cells <= config.min_reference_cells &&
           config.min_reference_free_cells <= config.min_reference_cells;
}

class SparseTofLocalMatchInputValidator {
public:
    SparseTofLocalMatchInputValidationResult validate(
        const SparseTofLocalMatchInput &input) const {
        if (!sparse_tof_local_match_config_valid(input.config)) {
            return fail(SparseTofLocalMatchInputStatus::ContractViolation,
                        "local_match_config_invalid");
        }
        if (!input.frozen_bundle) {
            return fail(SparseTofLocalMatchInputStatus::BundleMissing,
                        "local_match_bundle_missing");
        }
        const auto &bundle = *input.frozen_bundle;
        const auto &summary = bundle.summary();
        if (!bundle.available() || !summary.frozen ||
            summary.state != ActiveObservationBundleState::FrozenReady) {
            return fail(SparseTofLocalMatchInputStatus::BundleNotFrozen,
                        "local_match_bundle_not_frozen");
        }
        if (input.bundle_id == 0 || input.bundle_id != summary.bundle_id) {
            return fail(SparseTofLocalMatchInputStatus::BundleInvalid,
                        "local_match_bundle_id_mismatch");
        }
        if (bundle.frames().empty() ||
            summary.accepted_snapshot_count != bundle.frames().size()) {
            return fail(SparseTofLocalMatchInputStatus::BundleInvalid,
                        "local_match_bundle_frame_count_invalid");
        }
        if (bundle.frames().size() < input.config.min_bundle_frames ||
            summary.matchable_ray_count < input.config.min_matchable_rays) {
            return fail(SparseTofLocalMatchInputStatus::BundleInsufficient,
                        "local_match_bundle_insufficient");
        }
        if (!std::isfinite(summary.collection_start_timestamp_s) ||
            !std::isfinite(summary.collection_end_timestamp_s) ||
            !std::isfinite(summary.duration_s) ||
            summary.collection_start_timestamp_s >
                summary.collection_end_timestamp_s ||
            summary.duration_s < 0.0 ||
            summary.duration_s > input.config.max_bundle_duration_s ||
            std::fabs(summary.duration_s -
                      (summary.collection_end_timestamp_s -
                       summary.collection_start_timestamp_s)) > 1e-9) {
            return fail(SparseTofLocalMatchInputStatus::BundleInvalid,
                        "local_match_bundle_time_invalid");
        }
        for (const auto &frame : bundle.frames()) {
            if (!frame_valid(frame)) {
                return fail(SparseTofLocalMatchInputStatus::BundleInvalid,
                            "local_match_bundle_frame_invalid");
            }
        }

        if (!input.reference_map) {
            return fail(SparseTofLocalMatchInputStatus::ReferenceMapMissing,
                        "local_match_reference_map_missing");
        }
        const auto &map = *input.reference_map;
        if (!map.valid() || !std::isfinite(map.resolution_m()) ||
            map.resolution_m() <= 0.0 ||
            map.cell_count() != map.cells().size() ||
            map.cell_count() != map.free_cell_count() +
                                map.occupied_cell_count() +
                                map.uncertain_cell_count() ||
            !strictly_ordered(map.cells())) {
            return fail(SparseTofLocalMatchInputStatus::ReferenceMapInvalid,
                        "local_match_reference_map_invalid");
        }
        if (input.config.require_revision_match &&
            (map.revision() != input.expected_reference_map_revision ||
             map.revision() != summary.reference_map_revision ||
             map.revision() != input.current_runtime_map_revision)) {
            return fail(
                SparseTofLocalMatchInputStatus::ReferenceRevisionMismatch,
                "local_match_reference_revision_mismatch");
        }
        if (map.cell_count() < input.config.min_reference_cells ||
            map.occupied_cell_count() <
                input.config.min_reference_occupied_cells ||
            map.free_cell_count() < input.config.min_reference_free_cells) {
            return fail(SparseTofLocalMatchInputStatus::ReferenceMapTooSparse,
                        "local_match_reference_map_too_sparse");
        }
        if (!sparse_slam_frame_pose_valid(input.predicted_map_from_odom) ||
            !sparse_slam_frame_pose_valid(
                input.map_from_odom_at_collection_start)) {
            return fail(SparseTofLocalMatchInputStatus::PredictionInvalid,
                        "local_match_prediction_invalid");
        }
        if (!std::isfinite(input.match_request_timestamp_s) ||
            input.match_request_timestamp_s <
                summary.collection_end_timestamp_s) {
            return fail(SparseTofLocalMatchInputStatus::TimestampInvalid,
                        "local_match_request_timestamp_invalid");
        }
        if (input.source.empty()) {
            return fail(SparseTofLocalMatchInputStatus::ContractViolation,
                        "local_match_source_missing");
        }
        return {true, SparseTofLocalMatchInputStatus::Ready,
                "local_match_input_ready"};
    }

private:
    static SparseTofLocalMatchInputValidationResult fail(
        SparseTofLocalMatchInputStatus status,
        const std::string &reason) {
        return {false, status, reason};
    }

    static bool return_kind_valid(ScalarTofReturnKind kind) {
        switch (kind) {
        case ScalarTofReturnKind::Unspecified:
        case ScalarTofReturnKind::Invalid:
        case ScalarTofReturnKind::Hit:
        case ScalarTofReturnKind::NoReturn:
            return true;
        }
        return false;
    }

    static bool route_valid(const ResolvedScalarTofObservationRoute &route) {
        return route.present &&
               std::isfinite(route.effective_timestamp_s) &&
               return_kind_valid(route.return_kind) &&
               route.map_pose.valid &&
               sparse_slam_frame_pose_valid(route.map_pose.base_pose_in_map) &&
               sparse_slam_frame_pose_valid(route.odom_pose_at_measurement) &&
               std::isfinite(route.map_pose.measurement_timestamp_s) &&
               std::fabs(route.effective_timestamp_s -
                         route.map_pose.measurement_timestamp_s) <= 1e-9 &&
               std::fabs(route.effective_timestamp_s -
                         route.frame.effective_timestamp_s) <= 1e-9;
    }

    static bool frame_valid(const ResolvedMultiTofObservationFrame &frame) {
        return frame.complete &&
               std::isfinite(frame.snapshot_timestamp_s) &&
               route_valid(frame.front) &&
               route_valid(frame.left) &&
               route_valid(frame.right);
    }

    static bool strictly_ordered(
        const std::vector<SparseOccupancyCell> &cells) {
        for (std::size_t i = 1; i < cells.size(); ++i) {
            if (!(cells[i - 1].key < cells[i].key)) {
                return false;
            }
        }
        return true;
    }
};

} // namespace robot_slamd
