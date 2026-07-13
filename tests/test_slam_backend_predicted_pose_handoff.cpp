#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::RobotSlamMapQuality good_quality() {
    robot_slamd::RobotSlamMapQuality quality;
    quality.good_enough = true;
    quality.coverage_ratio = 0.8;
    quality.yaw_coverage_ratio = 0.8;
    quality.valid_scan_count = 1;
    quality.reason = "map_quality_good";
    return quality;
}

robot_slamd::SlamBackendUpdateResult accepted_result() {
    robot_slamd::SlamBackendUpdateResult result;
    result.status = robot_slamd::SlamBackendUpdateStatus::Accepted;
    result.fault = robot_slamd::SlamBackendFault::None;
    result.map_updated = true;
    result.integrated_scan_count = 1;
    result.update_latency_s = 0.01;
    result.quality = good_quality();
    result.message = "accepted";
    return result;
}

robot_slamd::ScalarTofSnapshotFrame scalar_frame(std::uint16_t mm,
                                                 double yaw,
                                                 const char *id) {
    robot_slamd::ScalarTofSnapshotFrame frame;
    frame.distance_mm = mm;
    frame.distance_m = static_cast<double>(mm) / 1000.0;
    frame.confidence = 100;
    frame.echo_tag_u48 = 0x1200 + mm;
    frame.effective_timestamp_s = 10.0;
    frame.protocol_valid = true;
    frame.usable_for_mapping = true;
    frame.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
    frame.mount_yaw_rad = yaw;
    frame.frame_id = id;
    frame.source = "handoff_test";
    return frame;
}

robot_slamd::RobotSlamSensorSnapshot native_snapshot(double now_s) {
    robot_slamd::RobotSlamSensorSnapshot snapshot;
    snapshot.has_tof = false;
    snapshot.has_multi_tof = true;
    snapshot.multi_tof.has_front = true;
    snapshot.multi_tof.has_left = true;
    snapshot.multi_tof.has_right = true;
    snapshot.multi_tof.valid_tof_count = 3;
    snapshot.multi_tof.front = scalar_frame(1000, 0.0, "front");
    snapshot.multi_tof.left = scalar_frame(900, 3.14159265358979323846 / 2.0, "left");
    snapshot.multi_tof.right = scalar_frame(1100, -3.14159265358979323846 / 2.0, "right");
    snapshot.multi_tof.front.effective_timestamp_s = now_s;
    snapshot.multi_tof.left.effective_timestamp_s = now_s;
    snapshot.multi_tof.right.effective_timestamp_s = now_s;
    snapshot.multi_tof.synchronized_time_s = now_s;
    snapshot.multi_tof.source = "native_multi_tof_handoff_test";
    snapshot.has_imu = true;
    snapshot.imu.timestamp_s = now_s;
    snapshot.imu.yaw_rate_rad_s = 0.0;
    snapshot.has_wheel = true;
    snapshot.wheel.timestamp_s = now_s;
    snapshot.wheel.valid = true;
    return snapshot;
}

class RecordingBackend final : public robot_slamd::SlamBackendBinding {
public:
    bool ready() const override { return ready_; }

    robot_slamd::SlamBackendUpdateResult update(
        const robot_slamd::SlamBackendInputFrame &input) override {
        update_count++;
        last_input = input;
        return accepted_result();
    }

    robot_slamd::RobotSlamMapQuality latest_quality(double) const override {
        return good_quality();
    }

    robot_slamd::SlamBackendSaveResult save_map(const std::string &path) override {
        robot_slamd::SlamBackendSaveResult result;
        result.ok = true;
        result.output_path = path;
        return result;
    }

    std::string status() const override { return "recording_backend_ready"; }

    bool ready_ = true;
    int update_count = 0;
    robot_slamd::SlamBackendInputFrame last_input;
};

robot_slamd::SlamBackendContractChecker native_checker() {
    robot_slamd::SlamBackendContractOptions options;
    options.require_tof = false;
    options.require_imu_or_wheel = true;
    options.allow_predicted_pose_missing = true;
    return robot_slamd::SlamBackendContractChecker(options);
}

robot_slamd::RobotSlamMapUpdateInput map_update(double now_s) {
    robot_slamd::RobotSlamMapUpdateInput input;
    input.timestamp_s = now_s;
    input.snapshot = native_snapshot(now_s);
    input.predicted_map_pose.x_m = 1.25;
    input.predicted_map_pose.y_m = -0.5;
    input.predicted_map_pose.yaw_rad = 0.75;
    input.has_predicted_map_pose = true;
    input.mapping_commit_allowed = true;
    input.source = "explicit_pose_handoff";
    return input;
}

bool sparse_accepts(const robot_slamd::RobotSlamMapUpdateInput &input) {
    robot_slamd::SparseMultiTofOccupancyBackendOptions options;
    options.grid.resolution_m = 0.5;
    options.minimum_accepted_snapshots_for_good_quality = 1;
    options.minimum_valid_rays_for_good_quality = 1;
    options.minimum_observed_cells_for_good_quality = 1;
    options.minimum_angular_bins_for_good_quality = 1;
    auto backend = std::make_shared<robot_slamd::SparseMultiTofOccupancyBackendBinding>(options);
    robot_slamd::SlamBackendMapPortAdapter adapter(backend, native_checker());
    return adapter.integrate_map_update(input);
}

} // namespace

int main() {
    using namespace robot_slamd;

    RobotSlamMapUpdateInput default_input;
    expect(!default_input.has_predicted_map_pose, "default map update has no pose");
    expect(default_input.mapping_commit_allowed, "default map update allows commit");

    auto backend = std::make_shared<RecordingBackend>();
    SlamBackendMapPortAdapter adapter(backend, native_checker());
    const auto input = map_update(10.0);
    expect(adapter.integrate_map_update(input), "explicit map update reaches backend");
    expect(backend->update_count == 1, "backend called once");
    expect(backend->last_input.timestamp_s == 10.0, "timestamp copied");
    expect(backend->last_input.snapshot.has_multi_tof, "snapshot copied");
    expect(backend->last_input.has_predicted_pose, "has pose copied");
    expect(backend->last_input.predicted_pose.x_m == 1.25, "pose x copied");
    expect(backend->last_input.predicted_pose.y_m == -0.5, "pose y copied");
    expect(backend->last_input.predicted_pose.yaw_rad == 0.75, "pose yaw copied");
    expect(backend->last_input.source == "explicit_pose_handoff", "source copied");
    expect(!backend->last_input.snapshot.has_tof, "legacy projection remains absent");

    auto no_commit = input;
    no_commit.mapping_commit_allowed = false;
    expect(!adapter.integrate_map_update(no_commit), "commit gate rejects update");
    expect(backend->update_count == 1, "commit gate does not call backend");

    expect(adapter.integrate_sensor_snapshot(native_snapshot(10.0), 10.0),
           "old snapshot entry can still call legacy-compatible backend");
    expect(!backend->last_input.has_predicted_pose,
           "old snapshot entry does not synthesize sparse pose");

    auto missing_pose = input;
    missing_pose.has_predicted_map_pose = false;
    expect(!sparse_accepts(missing_pose), "sparse backend rejects missing pose");

    auto nan_x = input;
    nan_x.predicted_map_pose.x_m = std::nan("");
    expect(!sparse_accepts(nan_x), "sparse backend rejects NaN x");
    auto nan_y = input;
    nan_y.predicted_map_pose.y_m = std::nan("");
    expect(!sparse_accepts(nan_y), "sparse backend rejects NaN y");
    auto nan_yaw = input;
    nan_yaw.predicted_map_pose.yaw_rad = std::nan("");
    expect(!sparse_accepts(nan_yaw), "sparse backend rejects NaN yaw");
    auto inf_x = input;
    inf_x.predicted_map_pose.x_m = std::numeric_limits<double>::infinity();
    expect(!sparse_accepts(inf_x), "sparse backend rejects infinity");

    expect(sparse_accepts(input), "finite explicit pose lets sparse backend accept native snapshot");

    auto repeat_backend = std::make_shared<RecordingBackend>();
    SlamBackendMapPortAdapter repeat_adapter(repeat_backend, native_checker());
    expect(repeat_adapter.integrate_map_update(input), "repeat explicit update accepted");
    expect(repeat_backend->last_input.predicted_pose.x_m == input.predicted_map_pose.x_m,
           "same input pose handoff deterministic");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
