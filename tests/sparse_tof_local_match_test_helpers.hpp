#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_match_input_validator.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_occupancy_grid.hpp"
#include "robot_slamd/runtime/active_multi_tof_observation_bundle.hpp"

#include <stdexcept>
#include <string>

namespace local_match_test {

inline void expect(bool condition, const std::string &message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

inline robot_slamd::ScalarTofSnapshotFrame tof(
    std::uint16_t mm, double yaw, double timestamp, const char *id) {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_mm = mm;
    out.distance_m = static_cast<double>(mm) / 1000.0;
    out.confidence = 100;
    out.echo_tag_u48 = 0x720000 + mm;
    out.effective_timestamp_s = timestamp;
    out.protocol_valid = true;
    out.usable_for_mapping = true;
    out.mount_yaw_rad = yaw;
    out.frame_id = id;
    out.source = "local_match_contract_test";
    return out;
}

inline robot_slamd::ActiveMultiTofObservationAppendInput append(double t) {
    using namespace robot_slamd;
    ActiveMultiTofObservationAppendInput in;
    in.snapshot.has_multi_tof = true;
    in.snapshot.multi_tof.has_front = true;
    in.snapshot.multi_tof.has_left = true;
    in.snapshot.multi_tof.has_right = true;
    in.snapshot.multi_tof.front = tof(1000, 0.0, t, "front");
    in.snapshot.multi_tof.left = tof(1100, 1.5707963267948966, t, "left");
    in.snapshot.multi_tof.right = tof(1100, -1.5707963267948966, t, "right");
    in.snapshot.multi_tof.synchronized_time_s = t;
    in.snapshot.multi_tof.valid_tof_count = 3;
    in.snapshot.multi_tof.source = "local_match_contract_test";
    auto pose = [t]() {
        TimedMapBasePose2D out;
        out.valid = true;
        out.measurement_timestamp_s = t;
        out.base_pose_in_map = make_map_pose({0.0, 0.0, 0.0});
        return out;
    };
    in.measurement_poses.front = pose();
    in.measurement_poses.left = pose();
    in.measurement_poses.right = pose();
    in.measurement_poses.complete = true;
    in.measurement_poses.valid_pose_count = 3;
    in.front_odom_pose_at_measurement = make_odom_pose({0.0, 0.0, 0.0});
    in.left_odom_pose_at_measurement = make_odom_pose({0.0, 0.0, 0.0});
    in.right_odom_pose_at_measurement = make_odom_pose({0.0, 0.0, 0.0});
    return in;
}

inline robot_slamd::FrozenMultiTofObservationBundle frozen_bundle() {
    using namespace robot_slamd;
    ActiveMultiTofObservationBundleConfig config;
    config.min_snapshots = 2;
    config.min_matchable_rays = 3;
    ActiveMultiTofObservationBundleBuilder builder(config);
    expect(builder.begin(7, 3, 3, identity_map_from_odom()).ok,
           "bundle begin");
    expect(builder.append(append(1.0)).ok, "bundle append first");
    expect(builder.append(append(1.1)).ok, "bundle append second");
    expect(builder.mark_waiting_for_motion_settle().ok, "bundle waiting");
    expect(builder.freeze(3).ok, "bundle freeze");
    return builder.frozen_bundle();
}

inline robot_slamd::SparseTofRayObservation map_hit() {
    robot_slamd::SparseTofRayObservation out;
    out.return_kind = robot_slamd::ScalarTofReturnKind::Hit;
    out.ray_yaw_rad = 0.0;
    out.measured_range_m = 1.0;
    out.free_space_range_m = 1.0;
    out.confidence = 100;
    out.protocol_valid = true;
    out.synchronized = true;
    out.usable_for_mapping = true;
    return out;
}

inline robot_slamd::SparseOccupancyGridSnapshot reference_map() {
    robot_slamd::SparseOccupancyGridConfig config;
    config.resolution_m = 0.5;
    robot_slamd::SparseOccupancyGrid grid(config);
    expect(grid.apply_observations({map_hit()}).accepted, "map first update");
    expect(grid.apply_observations({map_hit()}).accepted, "map second update");
    const auto capture = grid.capture_snapshot(3, 64);
    expect(capture.ok, "map capture");
    return capture.snapshot;
}

inline robot_slamd::SparseTofLocalMatchInput valid_input() {
    using namespace robot_slamd;
    SparseTofLocalMatchInput input;
    input.bundle_id = 7;
    input.frozen_bundle =
        std::make_shared<const FrozenMultiTofObservationBundle>(frozen_bundle());
    input.reference_map =
        std::make_shared<const SparseOccupancyGridSnapshot>(reference_map());
    input.expected_reference_map_revision = 3;
    input.current_runtime_map_revision = 3;
    input.predicted_map_from_odom = identity_map_from_odom();
    input.map_from_odom_at_collection_start = identity_map_from_odom();
    input.match_request_timestamp_s = 1.1;
    input.source = "local_match_contract_test";
    input.config.min_reference_cells = 2;
    input.config.min_reference_occupied_cells = 1;
    input.config.min_reference_free_cells = 1;
    return input;
}

} // namespace local_match_test
