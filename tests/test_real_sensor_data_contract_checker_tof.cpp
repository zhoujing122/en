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
    auto tof = RealSensorSampleData::valid_packet(100.0).tof;
    expect(checker.check_tof_frame(tof, 100.0).ok, "valid tof pass");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.timing.request_start_s = std::numeric_limits<double>::quiet_NaN();
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_request_start_non_finite");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.timing.response_received_s = std::numeric_limits<double>::quiet_NaN();
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_response_received_non_finite");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.timing.response_received_s = tof.timing.request_start_s - 0.01;
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_response_before_request");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.timing.estimated_sample_time_s =
        std::numeric_limits<double>::quiet_NaN();
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_estimated_sample_time_non_finite");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.timing.request_latency_s = 0.25;
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_request_latency_too_high");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.timing.estimated_sample_time_s = 99.0;
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_estimated_sample_time_stale");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.frame_id.clear();
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_empty_frame_id");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.ranges_m.clear();
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_empty_ranges");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.angle_increment_rad = 0.0;
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_invalid_angle_increment");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.range_min_m = 0.0;
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_invalid_range_min");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.range_max_m = tof.range_min_m;
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_invalid_range_max");

    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.ranges_m = {1.0,
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};
    expect_message(checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_nan_ratio_too_high");

    RealSensorDataContractOptions options;
    options.allow_nan_ranges = false;
    RealSensorDataContractChecker no_nan_checker(options);
    tof = RealSensorSampleData::valid_packet(100.0).tof;
    tof.ranges_m[0] = std::numeric_limits<double>::quiet_NaN();
    expect_message(no_nan_checker.check_tof_frame(tof, 100.0),
                   "real_sensor_tof_nan_not_allowed");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
