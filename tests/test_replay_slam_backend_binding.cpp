#include "robot_slamd/autonomy/map_backend/replay_slam_backend_binding.hpp"

#include <iostream>
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

    auto first = accepted_update();
    first.integrated_scan_count = 1;
    auto second = accepted_update();
    second.integrated_scan_count = 2;
    auto q1 = valid_quality(false);
    auto q2 = valid_quality(true);
    ReplaySlamBackendBinding backend({first, second}, {q1, q2});

    expect(backend.ready(), "ready default true");
    backend.set_ready(false);
    expect(!backend.ready(), "set ready false works");
    backend.set_ready(true);

    SlamBackendInputFrame input;
    input.timestamp_s = 100.0;
    input.snapshot = valid_snapshot(100.0);
    auto update = backend.update(input);
    expect(update.integrated_scan_count == 1, "first update returned");
    update = backend.update(input);
    expect(update.integrated_scan_count == 2, "second update returned");
    update = backend.update(input);
    expect(update.integrated_scan_count == 2, "last update held");

    auto quality = backend.latest_quality(100.0);
    expect(!quality.good_enough, "first quality returned");
    quality = backend.latest_quality(100.0);
    expect(quality.good_enough, "second quality returned");
    quality = backend.latest_quality(100.0);
    expect(quality.good_enough, "last quality held");

    auto save = backend.save_map("/tmp/replay_slam_backend_map");
    expect(save.ok, "save records ok");
    expect(backend.last_saved_path() == "/tmp/replay_slam_backend_map", "save path recorded");
    expect(backend.update_call_count() == 3, "update call count correct");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
