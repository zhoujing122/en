#include "robot_slamd/autonomy/map_backend/replay_slam_backend_binding.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool contains(const std::vector<std::string> &values, const std::string &needle) {
    for (const auto &value : values) {
        if (value == needle) {
            return true;
        }
    }
    return false;
}

robot_slamd::RobotSlamSensorSnapshot valid_snapshot(double now_s) {
    robot_slamd::RobotSlamSensorSnapshot snapshot;
    snapshot.has_tof = true;
    snapshot.tof.timestamp_s = now_s;
    snapshot.tof.ranges_m = {0.5, 0.6, 0.7};
    snapshot.tof.angle_increment_rad = 0.1;
    snapshot.tof.max_range_m = 3.0;
    snapshot.tof.frame_id = "front_tof";
    snapshot.has_imu = true;
    snapshot.imu.timestamp_s = now_s;
    snapshot.imu.az_mps2 = 9.8;
    snapshot.has_wheel = true;
    snapshot.wheel.timestamp_s = now_s;
    snapshot.wheel.valid = true;
    return snapshot;
}

robot_slamd::RobotSlamMapQuality valid_quality(bool good = true) {
    robot_slamd::RobotSlamMapQuality quality;
    quality.good_enough = good;
    quality.coverage_ratio = good ? 0.8 : 0.2;
    quality.yaw_coverage_ratio = good ? 0.8 : 0.2;
    quality.valid_scan_count = good ? 5 : 1;
    quality.reason = good ? "map_quality_good" : "map_quality_poor";
    return quality;
}

robot_slamd::SlamBackendUpdateResult accepted_update() {
    robot_slamd::SlamBackendUpdateResult result;
    result.status = robot_slamd::SlamBackendUpdateStatus::Accepted;
    result.fault = robot_slamd::SlamBackendFault::None;
    result.map_updated = true;
    result.keyframe_added = true;
    result.integrated_scan_count = 1;
    result.update_latency_s = 0.05;
    result.quality = valid_quality();
    result.message = "slam_update_ok";
    return result;
}

} // namespace

int main() {
    using namespace robot_slamd;

    SlamBackendMapPortAdapter missing(nullptr);
    expect(!missing.ready(), "missing backend not ready");

    auto backend = std::make_shared<ReplaySlamBackendBinding>(
        std::vector<SlamBackendUpdateResult>{accepted_update()},
        std::vector<RobotSlamMapQuality>{valid_quality()});
    SlamBackendMapPortAdapter adapter(backend);
    expect(adapter.ready(), "backend ready");
    expect(adapter.integrate_sensor_snapshot(valid_snapshot(100.0), 100.0), "valid snapshot integrates");
    expect(adapter.history().size() == 1, "history records update");

    auto invalid = valid_snapshot(100.0);
    invalid.tof.ranges_m.clear();
    expect(!adapter.integrate_sensor_snapshot(invalid, 100.0), "invalid snapshot rejected");
    expect(adapter.history().size() == 2, "history records invalid input");

    SlamBackendUpdateResult rejected = accepted_update();
    rejected.status = SlamBackendUpdateStatus::Rejected;
    rejected.message = "backend_rejected";
    auto reject_backend = std::make_shared<ReplaySlamBackendBinding>(
        std::vector<SlamBackendUpdateResult>{rejected},
        std::vector<RobotSlamMapQuality>{valid_quality(false)});
    SlamBackendMapPortAdapter reject_adapter(reject_backend);
    expect(!reject_adapter.integrate_sensor_snapshot(valid_snapshot(100.0), 100.0), "rejected update returns false");

    auto quality = adapter.latest_quality(100.0);
    expect(quality.good_enough, "latest quality returns backend quality");
    expect(adapter.save_map("/tmp/slam_backend_adapter_map"), "save map delegates");
    expect(backend->last_saved_path() == "/tmp/slam_backend_adapter_map", "save path recorded");
    expect(backend->update_call_count() == 1, "backend update called once");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
