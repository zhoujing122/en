#include "robot_slamd/autonomy/contracts/real_adapter_contract_checker.hpp"

#include <cmath>
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

robot_slamd::TofScanFrame valid_tof(double now_s = 100.0) {
    robot_slamd::TofScanFrame frame;
    frame.timestamp_s = now_s;
    frame.ranges_m = {0.5, 0.6, 0.7, 0.8};
    frame.angle_increment_rad = 0.1;
    frame.max_range_m = 3.0;
    frame.frame_id = "front_tof";
    return frame;
}

robot_slamd::ImuFrame valid_imu(double now_s = 100.0) {
    robot_slamd::ImuFrame frame;
    frame.timestamp_s = now_s;
    frame.yaw_rate_rad_s = 0.1;
    frame.ax_mps2 = 0.0;
    frame.ay_mps2 = 0.0;
    frame.az_mps2 = 9.8;
    return frame;
}

robot_slamd::WheelOdomFrame valid_wheel(double now_s = 100.0) {
    robot_slamd::WheelOdomFrame frame;
    frame.timestamp_s = now_s;
    frame.linear_mps = 0.05;
    frame.angular_rad_s = 0.1;
    frame.valid = true;
    return frame;
}

std::string first_error(const robot_slamd::RealAdapterContractReport &report) {
    return report.violations.empty() ? "" : report.violations.front().code;
}
} // namespace

int main() {
    using namespace robot_slamd;
    RealAdapterContractChecker checker;

    expect(checker.check_tof_frame(valid_tof(), 100.0).ok, "valid tof passes");
    auto tof = valid_tof();
    tof.timestamp_s = std::numeric_limits<double>::quiet_NaN();
    expect(first_error(checker.check_tof_frame(tof, 100.0)) == "tof_non_finite_timestamp", "tof nan timestamp");
    tof = valid_tof(99.0);
    expect(first_error(checker.check_tof_frame(tof, 100.0)) == "tof_frame_stale", "tof stale");
    tof = valid_tof();
    tof.ranges_m = {0.5};
    expect(first_error(checker.check_tof_frame(tof, 100.0)) == "tof_range_count_too_low", "tof count low");
    tof = valid_tof();
    tof.frame_id.clear();
    expect(first_error(checker.check_tof_frame(tof, 100.0)) == "tof_frame_id_required", "tof frame id");
    tof = valid_tof();
    tof.ranges_m = {0.5, std::nan(""), std::nan(""), std::nan("")};
    expect(first_error(checker.check_tof_frame(tof, 100.0)) == "tof_nan_ratio_too_high", "tof nan ratio");

    expect(checker.check_imu_frame(valid_imu(), 100.0).ok, "valid imu passes");
    auto imu = valid_imu();
    imu.yaw_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
    expect(first_error(checker.check_imu_frame(imu, 100.0)) == "imu_non_finite_yaw_rate", "imu yaw nan");
    imu = valid_imu();
    imu.az_mps2 = 100.0;
    expect(first_error(checker.check_imu_frame(imu, 100.0)) == "imu_acc_out_of_range", "imu acc range");

    expect(checker.check_wheel_frame(valid_wheel(), 100.0).ok, "valid wheel passes");
    auto wheel = valid_wheel();
    wheel.valid = false;
    expect(first_error(checker.check_wheel_frame(wheel, 100.0)) == "wheel_invalid_flag", "wheel invalid flag");
    wheel = valid_wheel();
    wheel.linear_mps = 3.0;
    expect(first_error(checker.check_wheel_frame(wheel, 100.0)) == "wheel_linear_out_of_range", "wheel linear range");

    RobotSlamSensorSnapshot snapshot;
    expect(first_error(checker.check_sensor_snapshot(snapshot, 100.0)) == "snapshot_missing_tof", "snapshot missing tof");
    snapshot.has_tof = true;
    snapshot.tof = valid_tof();
    expect(first_error(checker.check_sensor_snapshot(snapshot, 100.0)) == "snapshot_missing_imu_or_wheel", "snapshot missing imu wheel");
    snapshot.has_imu = true;
    snapshot.imu = valid_imu();
    expect(checker.check_sensor_snapshot(snapshot, 100.0).ok, "snapshot tof imu passes");
    snapshot.has_imu = false;
    snapshot.has_wheel = true;
    snapshot.wheel = valid_wheel();
    expect(checker.check_sensor_snapshot(snapshot, 100.0).ok, "snapshot tof wheel passes");

    RobotSlamMapQuality quality;
    quality.good_enough = true;
    quality.coverage_ratio = 1.2;
    quality.yaw_coverage_ratio = 0.5;
    expect(first_error(checker.check_map_quality(quality)) == "map_coverage_out_of_range", "map coverage range");
    quality.coverage_ratio = 0.1;
    quality.good_enough = false;
    quality.reason.clear();
    auto poor = checker.check_map_quality(quality);
    expect(poor.ok, "poor map empty reason warning still ok");
    expect(!poor.warnings.empty() && poor.warnings.front() == "map_poor_quality_reason_empty", "poor map warning");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
