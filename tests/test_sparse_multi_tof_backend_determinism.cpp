#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"

#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::ScalarTofSnapshotFrame frame(std::uint16_t mm, double yaw) {
    robot_slamd::ScalarTofSnapshotFrame f;
    f.distance_mm = mm;
    f.distance_m = static_cast<double>(mm) / 1000.0;
    f.confidence = 100;
    f.effective_timestamp_s = 10.0;
    f.protocol_valid = true;
    f.usable_for_mapping = true;
    f.mount_yaw_rad = yaw;
    f.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
    f.source = "determinism";
    return f;
}

robot_slamd::SlamBackendInputFrame input(double yaw, double timestamp_s = 10.0) {
    robot_slamd::SlamBackendInputFrame in;
    in.timestamp_s = timestamp_s;
    in.has_predicted_pose = true;
    in.predicted_pose.yaw_rad = yaw;
    in.snapshot.has_multi_tof = true;
    in.snapshot.multi_tof.has_front = true;
    in.snapshot.multi_tof.has_left = true;
    in.snapshot.multi_tof.has_right = true;
    in.snapshot.multi_tof.valid_tof_count = 3;
    in.snapshot.multi_tof.front = frame(1000, 0.0);
    in.snapshot.multi_tof.left =
        frame(1000, 3.14159265358979323846 / 2.0);
    in.snapshot.multi_tof.right =
        frame(1000, -3.14159265358979323846 / 2.0);
    in.snapshot.multi_tof.front.effective_timestamp_s = timestamp_s;
    in.snapshot.multi_tof.left.effective_timestamp_s = timestamp_s;
    in.snapshot.multi_tof.right.effective_timestamp_s = timestamp_s;
    return in;
}
}

int main() {
    using namespace robot_slamd;
    SparseMultiTofOccupancyBackendOptions options;
    options.grid.resolution_m = 0.5;
    SparseMultiTofOccupancyBackendBinding a(options);
    SparseMultiTofOccupancyBackendBinding b(options);
    for (int i = 0; i < 3; ++i) {
        const double ts = 10.0 + static_cast<double>(i);
        auto ia = input(static_cast<double>(i) * 0.25, ts);
        auto ib = input(static_cast<double>(i) * 0.25, ts);
        expect(a.update(ia).status == SlamBackendUpdateStatus::Accepted,
               "backend A accepts");
        expect(b.update(ib).status == SlamBackendUpdateStatus::Accepted,
               "backend B accepts");
    }
    const auto sa = a.grid_snapshot();
    const auto sb = b.grid_snapshot();
    expect(sa.cells.size() == sb.cells.size(), "cell count deterministic");
    bool same = sa.cells.size() == sb.cells.size();
    for (std::size_t i = 0; i < sa.cells.size() && i < sb.cells.size(); ++i) {
        same = same && sa.cells[i].key == sb.cells[i].key &&
               sa.cells[i].evidence == sb.cells[i].evidence;
    }
    expect(same, "cell order and evidence deterministic");
    expect(a.report().hit_ray_count == b.report().hit_ray_count,
           "report counters deterministic");
    expect(!a.report().ground_truth_consumed, "ground truth not consumed");
    return failures ? 1 : 0;
}
