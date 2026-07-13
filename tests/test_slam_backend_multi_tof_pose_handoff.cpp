#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"

#include <iostream>

namespace {
void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

class CaptureBackend final : public robot_slamd::SlamBackendBinding {
public:
    robot_slamd::SlamBackendInputFrame last;
    int count = 0;
    bool ready() const override { return true; }
    robot_slamd::SlamBackendUpdateResult update(
        const robot_slamd::SlamBackendInputFrame &input) override {
        last = input;
        count++;
        robot_slamd::SlamBackendUpdateResult out;
        out.status = robot_slamd::SlamBackendUpdateStatus::Accepted;
        out.map_updated = true;
        out.integrated_scan_count = count;
        out.quality.coverage_ratio = 1.0;
        out.quality.yaw_coverage_ratio = 1.0;
        out.quality.valid_scan_count = count;
        out.quality.good_enough = true;
        out.quality.reason = "capture_quality";
        return out;
    }
    robot_slamd::RobotSlamMapQuality latest_quality(double) const override {
        return {};
    }
    robot_slamd::SlamBackendSaveResult save_map(const std::string &) override {
        return {};
    }
    std::string status() const override { return "capture"; }
};

robot_slamd::TimedMapBasePose2D timed(double t, double x) {
    robot_slamd::TimedMapBasePose2D out;
    out.valid = true;
    out.measurement_timestamp_s = t;
    robot_slamd::RobotPose2D pose;
    pose.x_m = x;
    out.base_pose_in_map = robot_slamd::make_map_pose(pose);
    out.lookup_status = robot_slamd::TimedPoseLookupStatus::Exact;
    return out;
}
} // namespace

int main() {
    auto backend = std::make_shared<CaptureBackend>();
    robot_slamd::SlamBackendContractOptions options;
    options.require_tof = false;
    options.require_imu_or_wheel = false;
    robot_slamd::SlamBackendMapPortAdapter adapter(
        backend, robot_slamd::SlamBackendContractChecker(options));
    robot_slamd::RobotSlamMapUpdateInput input;
    input.timestamp_s = 10.0;
    input.has_predicted_map_pose = true;
    input.has_multi_tof_measurement_poses = true;
    input.multi_tof_measurement_poses.front = timed(9.9, 1.0);
    input.multi_tof_measurement_poses.left = timed(10.0, 2.0);
    input.multi_tof_measurement_poses.right = timed(10.1, 3.0);
    input.multi_tof_measurement_poses.complete = true;
    input.multi_tof_measurement_poses.valid_pose_count = 3;
    input.source = "handoff_test";
    expect(adapter.integrate_map_update(input), "adapter update accepted");
    expect(backend->count == 1, "backend called once");
    expect(backend->last.has_multi_tof_measurement_poses,
           "measurement poses flag copied");
    expect(backend->last.multi_tof_measurement_poses.left
               .base_pose_in_map.map_T_base.x_m == 2.0,
           "left pose copied");
    expect(backend->last.source == "handoff_test", "source copied");
    robot_slamd::RobotSlamMapUpdateInput old_entry;
    old_entry.timestamp_s = 11.0;
    old_entry.mapping_commit_allowed = true;
    adapter.integrate_sensor_snapshot(old_entry.snapshot, 11.0);
    expect(!backend->last.has_multi_tof_measurement_poses,
           "old entry does not fabricate measurement poses");
    return 0;
}
