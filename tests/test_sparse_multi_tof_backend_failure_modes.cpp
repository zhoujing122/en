#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"

#include <iostream>
#include <limits>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::ScalarTofSnapshotFrame frame() {
    robot_slamd::ScalarTofSnapshotFrame f;
    f.distance_mm = 1000;
    f.distance_m = 1.0;
    f.confidence = 100;
    f.effective_timestamp_s = 10.0;
    f.protocol_valid = true;
    f.usable_for_mapping = true;
    f.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
    f.frame_id = "front";
    f.source = "unit_test";
    return f;
}

robot_slamd::RobotSlamSensorSnapshot snapshot() {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_multi_tof = true;
    s.multi_tof.has_front = true;
    s.multi_tof.valid_tof_count = 1;
    s.multi_tof.front = frame();
    s.multi_tof.synchronized_time_s = 10.0;
    return s;
}

robot_slamd::SlamBackendInputFrame input(
    const robot_slamd::RobotSlamSensorSnapshot &s) {
    robot_slamd::SlamBackendInputFrame in;
    in.timestamp_s = 10.0;
    in.snapshot = s;
    in.has_predicted_pose = true;
    in.predicted_pose = robot_slamd::RobotPose2D{};
    return in;
}
}

int main() {
    using namespace robot_slamd;
    SparseMultiTofOccupancyBackendOptions options;
    options.grid.resolution_m = 0.5;

    SparseMultiTofOccupancyBackendBinding backend(options);
    SlamBackendInputFrame empty;
    empty.timestamp_s = 10.0;
    empty.has_predicted_pose = true;
    auto empty_result = backend.update(empty);
    expect(empty_result.status == SlamBackendUpdateStatus::Rejected,
           "empty snapshot rejected");
    expect(backend.report().last_fault ==
               SparseMultiTofBackendFault::SnapshotMissing,
           "empty snapshot fault");

    SparseMultiTofOccupancyBackendBinding no_multi(options);
    auto legacy_only = snapshot();
    legacy_only.has_multi_tof = false;
    legacy_only.has_tof = true;
    auto no_multi_result = no_multi.update(input(legacy_only));
    expect(no_multi_result.status == SlamBackendUpdateStatus::Rejected,
           "legacy-only rejected");
    expect(no_multi.report().last_fault ==
               SparseMultiTofBackendFault::MultiTofMissing,
           "multi-ToF missing fault");

    SparseMultiTofOccupancyBackendBinding missing_pose(options);
    auto no_pose_input = input(snapshot());
    no_pose_input.has_predicted_pose = false;
    auto no_pose = missing_pose.update(no_pose_input);
    expect(no_pose.status == SlamBackendUpdateStatus::Rejected,
           "missing pose rejected");
    expect(missing_pose.report().last_fault ==
               SparseMultiTofBackendFault::PoseUnavailable,
           "pose unavailable fault");

    SparseMultiTofOccupancyBackendBinding bad_pose(options);
    auto bad_pose_input = input(snapshot());
    bad_pose_input.predicted_pose.x_m =
        std::numeric_limits<double>::quiet_NaN();
    auto bad = bad_pose.update(bad_pose_input);
    expect(bad.status == SlamBackendUpdateStatus::Rejected,
           "invalid pose rejected");
    expect(bad_pose.report().last_fault ==
               SparseMultiTofBackendFault::PoseInvalid,
           "pose invalid fault");

    auto low = snapshot();
    low.multi_tof.front.confidence = 0;
    low.multi_tof.front.usable_for_mapping = false;
    SparseMultiTofOccupancyBackendBinding low_backend(options);
    auto low_result = low_backend.update(input(low));
    expect(low_result.status == SlamBackendUpdateStatus::Rejected,
           "confidence zero rejected");
    expect(low_backend.grid_snapshot().cells.empty(),
           "invalid ray no map update");

    SparseMultiTofOccupancyBackendOptions tiny = options;
    tiny.grid.maximum_active_cells = 1;
    SparseMultiTofOccupancyBackendBinding cap_backend(tiny);
    auto cap = cap_backend.update(input(snapshot()));
    expect(cap.status == SlamBackendUpdateStatus::Rejected,
           "capacity failure rejected");
    expect(cap_backend.report().last_fault ==
               SparseMultiTofBackendFault::MapCapacityReached,
           "capacity fault surfaced");
    expect(cap_backend.grid_snapshot().cells.empty(),
           "capacity failure transactional");

    expect(!cap_backend.report().real_hardware_accessed &&
               !cap_backend.report().real_motion_attempted &&
               !cap_backend.report().real_map_write_attempted &&
               !cap_backend.report().real_pose_writeback_attempted,
           "safety flags remain false");
    return failures ? 1 : 0;
}
