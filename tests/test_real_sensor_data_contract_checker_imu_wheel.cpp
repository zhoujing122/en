#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_checker.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_sample_data.hpp"

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

void expect_message(const robot_slamd::RealSensorDataContractResult &result,
                    const std::string &message) {
    expect(!result.ok, message.c_str());
    expect(result.message == message, message.c_str());
}
} // namespace

int main() {
    using namespace robot_slamd;

    RealSensorDataContractChecker checker;
    auto packet = RealSensorSampleData::valid_packet(100.0);
    auto imu = packet.imu;
    auto wheel = packet.wheel;
    expect(checker.check_imu_sample(imu, 100.0).ok, "valid imu pass");

    imu.timestamp_s = std::numeric_limits<double>::quiet_NaN();
    expect_message(checker.check_imu_sample(imu, 100.0),
                   "real_sensor_imu_non_finite_timestamp");

    imu = packet.imu;
    imu.timestamp_s = 100.06;
    expect_message(checker.check_imu_sample(imu, 100.0),
                   "real_sensor_imu_timestamp_in_future");

    imu = packet.imu;
    imu.timestamp_s = 99.0;
    expect_message(checker.check_imu_sample(imu, 100.0),
                   "real_sensor_imu_stale");

    imu = packet.imu;
    imu.yaw_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
    expect_message(checker.check_imu_sample(imu, 100.0),
                   "real_sensor_imu_non_finite_yaw_rate");

    imu = packet.imu;
    imu.yaw_rate_rad_s = 20.0;
    expect_message(checker.check_imu_sample(imu, 100.0),
                   "real_sensor_imu_yaw_rate_out_of_range");

    imu = packet.imu;
    imu.accel_x_mps2 = std::numeric_limits<double>::quiet_NaN();
    expect_message(checker.check_imu_sample(imu, 100.0),
                   "real_sensor_imu_non_finite_acceleration");

    imu = packet.imu;
    imu.accel_x_mps2 = 100.0;
    expect_message(checker.check_imu_sample(imu, 100.0),
                   "real_sensor_imu_acceleration_out_of_range");

    expect(checker.check_wheel_sample(wheel, 100.0).ok, "valid wheel pass");

    wheel = packet.wheel;
    wheel.timing = make_request_timing(99.70, 100.0);
    expect_message(checker.check_wheel_sample(wheel, 100.0),
                   "real_sensor_wheel_request_latency_too_high");

    wheel = packet.wheel;
    wheel.valid = false;
    expect_message(checker.check_wheel_sample(wheel, 100.0),
                   "real_sensor_wheel_invalid_flag");

    wheel = packet.wheel;
    wheel.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
    expect_message(checker.check_wheel_sample(wheel, 100.0),
                   "real_sensor_wheel_non_finite_linear_velocity");

    wheel = packet.wheel;
    wheel.angular_velocity_rad_s = 20.0;
    expect_message(checker.check_wheel_sample(wheel, 100.0),
                   "real_sensor_wheel_angular_velocity_out_of_range");

    expect_message(checker.check_packet(
                       RealSensorSampleData::missing_tof_packet(100.0),
                       100.0),
                   "real_sensor_packet_missing_tof");
    expect_message(checker.check_packet(
                       RealSensorSampleData::missing_imu_and_wheel_packet(
                           100.0),
                       100.0),
                   "real_sensor_packet_missing_imu_and_wheel");

    auto future_packet = RealSensorSampleData::valid_packet(100.0);
    future_packet.packet_timestamp_s = 100.06;
    expect_message(checker.check_packet(future_packet, 100.0),
                   "real_sensor_packet_timestamp_in_future");

    auto tof_mismatch = RealSensorSampleData::valid_packet(100.0);
    tof_mismatch.packet_timestamp_s = 100.25;
    tof_mismatch.imu.timestamp_s = 100.25;
    tof_mismatch.wheel.timing = make_request_timing(100.24, 100.26);
    expect_message(checker.check_packet(tof_mismatch, 100.25),
                   "real_sensor_packet_tof_time_mismatch");

    auto imu_mismatch = RealSensorSampleData::valid_packet(100.0);
    imu_mismatch.packet_timestamp_s = 100.25;
    imu_mismatch.tof.timing = make_request_timing(100.24, 100.26);
    imu_mismatch.wheel.timing = make_request_timing(100.24, 100.26);
    expect_message(checker.check_packet(imu_mismatch, 100.25),
                   "real_sensor_packet_imu_time_mismatch");

    auto wheel_mismatch = RealSensorSampleData::valid_packet(100.0);
    wheel_mismatch.packet_timestamp_s = 100.25;
    wheel_mismatch.tof.timing = make_request_timing(100.24, 100.26);
    wheel_mismatch.imu.timestamp_s = 100.25;
    expect_message(checker.check_packet(wheel_mismatch, 100.25),
                   "real_sensor_packet_wheel_time_mismatch");

    auto sync = RealSensorSampleData::sync_too_large_packet(100.0);
    sync.packet_timestamp_s = 100.075;
    sync.imu.timestamp_s = 100.15;
    auto sync_result = checker.check_packet(sync, 100.15);
    expect(!sync_result.ok, "sync fail");
    expect(sync_result.message == "real_sensor_tof_imu_sync_too_large" ||
               sync_result.message == "real_sensor_tof_wheel_sync_too_large",
           "sync uses effective time");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
