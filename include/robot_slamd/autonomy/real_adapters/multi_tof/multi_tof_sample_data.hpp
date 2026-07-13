#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_types.hpp"

namespace robot_slamd {

class MultiTofSampleData {
public:
    static MultiTofRawPacket valid_three_tof_packet() {
        MultiTofRawPacket packet;
        packet.has_front = true;
        packet.has_left = true;
        packet.has_right = true;
        packet.front = make_multi_tof_frame(
            MultiTofMountId::Front,
            "tof_front_frame",
            0.0);
        packet.left = make_multi_tof_frame(
            MultiTofMountId::Left,
            "tof_left_frame",
            1.5707963267948966);
        packet.right = make_multi_tof_frame(
            MultiTofMountId::Right,
            "tof_right_frame",
            -1.5707963267948966);
        packet.left.sequence = 2;
        packet.right.sequence = 3;
        packet.has_imu = true;
        packet.imu.timestamp_s = 100.0;
        packet.imu.yaw_rate_rad_s = 0.01;
        packet.has_wheel = true;
        packet.wheel.timing = make_request_timing(99.98, 100.02);
        packet.wheel.frame_id = "wheel_frame";
        packet.wheel.valid = true;
        packet.wheel.linear_velocity_mps = 0.05;
        packet.wheel.angular_velocity_rad_s = 0.01;
        packet.packet_timestamp_s = 100.0;
        packet.packet_source = "deterministic_three_tof_sample";
        return packet;
    }

    static MultiTofRawPacket missing_left_packet() {
        auto packet = valid_three_tof_packet();
        packet.has_left = false;
        return packet;
    }

    static MultiTofRawPacket duplicate_frame_id_packet() {
        auto packet = valid_three_tof_packet();
        packet.right.frame_id = packet.left.frame_id;
        return packet;
    }

    static MultiTofRawPacket duplicate_mount_id_packet() {
        auto packet = valid_three_tof_packet();
        packet.right.mount_id = MultiTofMountId::Left;
        return packet;
    }

    static MultiTofRawPacket invalid_mount_yaw_packet() {
        auto packet = valid_three_tof_packet();
        packet.left.mount_yaw_rad = 0.0;
        return packet;
    }

    static MultiTofRawPacket invalid_distance_packet() {
        auto packet = valid_three_tof_packet();
        packet.front.distance_mm = 19;
        return packet;
    }

    static MultiTofRawPacket low_confidence_packet() {
        auto packet = valid_three_tof_packet();
        packet.front.confidence = 69;
        return packet;
    }

    static MultiTofRawPacket latency_too_high_packet() {
        auto packet = valid_three_tof_packet();
        packet.front.timing = make_request_timing(99.0, 100.0);
        packet.front.timing.estimated_sample_time_s = 99.5;
        packet.packet_timestamp_s = 100.0;
        return packet;
    }
};

} // namespace robot_slamd
