#include "robot_slamd/autonomy/map_backend/replay_slam_backend_binding.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_acceptance_runner.hpp"

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

    auto backend = std::make_shared<ReplaySlamBackendBinding>(
        std::vector<SlamBackendUpdateResult>{accepted_update()},
        std::vector<RobotSlamMapQuality>{valid_quality()});
    SlamBackendAcceptanceRunner runner(backend);
    auto result = runner.run_once(valid_snapshot(100.0));
    expect(result.ok, "valid backend acceptance passes");

    SlamBackendAcceptanceRunner missing(nullptr);
    result = missing.run_once(valid_snapshot(100.0));
    expect(!result.ok && contains(result.failed, "slam_backend_missing"), "missing backend fails");

    backend->set_ready(false);
    result = runner.run_once(valid_snapshot(100.0));
    expect(!result.ok && contains(result.failed, "slam_backend_not_ready"), "not ready backend fails");
    backend->set_ready(true);

    auto invalid = valid_snapshot(100.0);
    invalid.has_tof = false;
    result = runner.run_once(invalid);
    expect(!result.ok && contains(result.failed, "slam_backend_input_contract_failed"), "invalid input fails");

    SlamBackendUpdateResult bad_update = accepted_update();
    bad_update.map_updated = false;
    auto bad_backend = std::make_shared<ReplaySlamBackendBinding>(
        std::vector<SlamBackendUpdateResult>{bad_update},
        std::vector<RobotSlamMapQuality>{valid_quality()});
    SlamBackendAcceptanceRunner bad_update_runner(bad_backend);
    result = bad_update_runner.run_once(valid_snapshot(100.0));
    expect(!result.ok && contains(result.failed, "slam_backend_update_contract_failed"), "invalid update fails");

    auto invalid_quality = valid_quality();
    invalid_quality.coverage_ratio = 2.0;
    auto bad_quality_backend = std::make_shared<ReplaySlamBackendBinding>(
        std::vector<SlamBackendUpdateResult>{accepted_update()},
        std::vector<RobotSlamMapQuality>{invalid_quality});
    SlamBackendAcceptanceRunner bad_quality_runner(bad_quality_backend);
    result = bad_quality_runner.run_once(valid_snapshot(100.0));
    expect(!result.ok && contains(result.failed, "slam_backend_quality_contract_failed"), "invalid quality fails");

    SlamBackendAcceptanceRunnerOptions options;
    options.require_ready = false;
    options.require_save_map = true;
    auto save_fail_backend = std::make_shared<ReplaySlamBackendBinding>(
        std::vector<SlamBackendUpdateResult>{accepted_update()},
        std::vector<RobotSlamMapQuality>{valid_quality()},
        false);
    SlamBackendAcceptanceRunner save_fail_runner(save_fail_backend, {}, options);
    result = save_fail_runner.run_once(valid_snapshot(100.0));
    expect(!result.ok && contains(result.failed, "slam_backend_save_failed"), "save failure recorded");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
