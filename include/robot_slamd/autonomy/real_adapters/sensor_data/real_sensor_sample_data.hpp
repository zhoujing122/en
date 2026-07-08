#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_types.hpp"

#include <limits>

namespace robot_slamd {

class RealSensorSampleData {
public:
    static RealSensorRawPacket valid_packet(double timestamp_s) {
        RealSensorRawPacket packet;
        packet.has_tof = true;
        packet.has_imu = true;
        packet.has_wheel = true;
        packet.packet_timestamp_s = timestamp_s;
        packet.packet_source = "m3b1_sample";

        packet.tof.timing = make_request_timing(timestamp_s - 0.01,
                                                timestamp_s + 0.01);
        packet.tof.frame_id = "tof_frame";
        packet.tof.ranges_m = {1.0, 1.2, 1.4, 1.6};
        packet.tof.angle_min_rad = -0.2;
        packet.tof.angle_max_rad = 0.2;
        packet.tof.angle_increment_rad = 0.1;
        packet.tof.range_min_m = 0.05;
        packet.tof.range_max_m = 4.0;
        packet.tof.sequence = 1;
        packet.tof.source = "sample_tof";

        packet.imu.timestamp_s = timestamp_s;
        packet.imu.frame_id = "imu_frame";
        packet.imu.yaw_rate_rad_s = 0.1;
        packet.imu.accel_x_mps2 = 0.0;
        packet.imu.accel_y_mps2 = 0.0;
        packet.imu.accel_z_mps2 = 9.8;
        packet.imu.sequence = 1;
        packet.imu.source = "sample_imu";

        packet.wheel.timing = make_request_timing(timestamp_s - 0.015,
                                                  timestamp_s + 0.015);
        packet.wheel.frame_id = "wheel_frame";
        packet.wheel.valid = true;
        packet.wheel.linear_velocity_mps = 0.05;
        packet.wheel.angular_velocity_rad_s = 0.1;
        packet.wheel.sequence = 1;
        packet.wheel.source = "sample_wheel";
        return packet;
    }

    static RealSensorRawPacket missing_tof_packet(double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.has_tof = false;
        return packet;
    }

    static RealSensorRawPacket missing_imu_and_wheel_packet(
        double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.has_imu = false;
        packet.has_wheel = false;
        return packet;
    }

    static RealSensorRawPacket empty_tof_ranges_packet(double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.tof.ranges_m.clear();
        return packet;
    }

    static RealSensorRawPacket stale_packet(double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.packet_timestamp_s = timestamp_s - 10.0;
        packet.tof.timing = make_request_timing(timestamp_s - 10.01,
                                                timestamp_s - 9.99);
        packet.imu.timestamp_s = timestamp_s - 10.0;
        packet.wheel.timing = make_request_timing(timestamp_s - 10.01,
                                                  timestamp_s - 9.99);
        return packet;
    }

    static RealSensorRawPacket sync_too_large_packet(double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.imu.timestamp_s = timestamp_s + 0.25;
        return packet;
    }

    static RealSensorRawPacket request_latency_too_high_packet(
        double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.tof.timing = make_request_timing(timestamp_s - 0.30,
                                                timestamp_s);
        return packet;
    }

    static RealSensorRawPacket invalid_request_timing_packet(
        double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.tof.timing = make_request_timing(timestamp_s,
                                                timestamp_s - 0.01);
        return packet;
    }

    static RealSensorRawPacket invalid_imu_packet(double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.imu.yaw_rate_rad_s =
            std::numeric_limits<double>::quiet_NaN();
        return packet;
    }

    static RealSensorRawPacket invalid_wheel_packet(double timestamp_s) {
        auto packet = valid_packet(timestamp_s);
        packet.wheel.valid = false;
        return packet;
    }
};

} // namespace robot_slamd
