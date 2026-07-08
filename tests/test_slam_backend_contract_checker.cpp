#include "robot_slamd/autonomy/map_backend/slam_backend_contract_checker.hpp"

#include <iostream>
#include <limits>
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

robot_slamd::SlamBackendInputFrame input_frame() {
    robot_slamd::SlamBackendInputFrame input;
    input.timestamp_s = 100.0;
    input.snapshot = valid_snapshot(100.0);
    input.source = "test";
    return input;
}
} // namespace

int main() {
    using namespace robot_slamd;
    SlamBackendContractChecker checker;

    auto input = input_frame();
    auto result = checker.check_input_frame(input, 100.0);
    expect(result.status == SlamBackendUpdateStatus::Accepted, "valid input accepted");

    input = input_frame();
    input.timestamp_s = std::numeric_limits<double>::quiet_NaN();
    result = checker.check_input_frame(input, 100.0);
    expect(result.message == "slam_input_non_finite_timestamp", "nan timestamp rejected");

    input = input_frame();
    input.timestamp_s = 99.0;
    result = checker.check_input_frame(input, 100.0);
    expect(result.message == "slam_input_stale", "stale input rejected");

    input = input_frame();
    input.snapshot.has_tof = false;
    result = checker.check_input_frame(input, 100.0);
    expect(result.message == "slam_input_missing_tof", "missing tof rejected");

    input = input_frame();
    input.snapshot.has_imu = false;
    input.snapshot.has_wheel = false;
    result = checker.check_input_frame(input, 100.0);
    expect(result.message == "slam_input_missing_imu_or_wheel", "missing motion reference rejected");

    input = input_frame();
    input.snapshot.tof.ranges_m.clear();
    result = checker.check_input_frame(input, 100.0);
    expect(result.message == "slam_input_empty_tof_ranges", "empty tof rejected");

    input = input_frame();
    input.snapshot.tof.angle_increment_rad = 0.0;
    result = checker.check_input_frame(input, 100.0);
    expect(result.message == "slam_input_invalid_tof_angle_increment", "angle increment rejected");

    input = input_frame();
    input.snapshot.wheel.valid = false;
    result = checker.check_input_frame(input, 100.0);
    expect(result.message == "slam_input_invalid_wheel_flag", "wheel flag rejected");

    auto update = accepted_update();
    update.map_updated = false;
    result = checker.check_update_result(update);
    expect(result.message == "slam_update_accepted_without_map_update", "accepted without map update rejected");

    update = accepted_update();
    update.update_latency_s = 2.0;
    result = checker.check_update_result(update);
    expect(result.message == "slam_update_latency_too_high", "latency too high rejected");

    RobotSlamMapQuality quality = valid_quality();
    quality.coverage_ratio = 2.0;
    result = checker.check_quality(quality);
    expect(result.message == "slam_quality_coverage_out_of_range", "quality coverage rejected");

    quality = valid_quality(false);
    quality.reason.clear();
    result = checker.check_quality(quality);
    expect(result.message == "slam_quality_poor_reason_empty", "poor quality reason required");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
