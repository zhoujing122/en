#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sample_data.hpp"

namespace robot_slamd {

class MultiTofSyncSampleData {
public:
    static MultiTofRawPacket valid_synced_packet() {
        auto packet = MultiTofSampleData::valid_three_tof_packet();
        set_tof_time(packet.front, 100.000);
        set_tof_time(packet.left, 100.010);
        set_tof_time(packet.right, 99.995);
        packet.imu.timestamp_s = 100.000;
        packet.wheel.timing = request_timing_for_estimate(100.000);
        packet.packet_timestamp_s = 100.000;
        return packet;
    }

    static MultiTofRawPacket front_left_sync_too_large_packet() {
        auto packet = valid_synced_packet();
        set_tof_time(packet.left, 100.080);
        return packet;
    }

    static MultiTofRawPacket front_right_sync_too_large_packet() {
        auto packet = valid_synced_packet();
        set_tof_time(packet.right, 100.080);
        set_tof_time(packet.left, 100.010);
        return packet;
    }

    static MultiTofRawPacket left_right_sync_too_large_packet() {
        auto packet = valid_synced_packet();
        set_tof_time(packet.front, 100.000);
        set_tof_time(packet.left, 99.960);
        set_tof_time(packet.right, 100.040);
        return packet;
    }

    static MultiTofRawPacket imu_sync_too_large_packet() {
        auto packet = valid_synced_packet();
        packet.imu.timestamp_s = 100.250;
        return packet;
    }

    static MultiTofRawPacket wheel_sync_too_large_packet() {
        auto packet = valid_synced_packet();
        packet.wheel.timing = request_timing_for_estimate(100.250);
        return packet;
    }

    static MultiTofRawPacket missing_left_degradable_packet() {
        auto packet = valid_synced_packet();
        packet.has_left = false;
        return packet;
    }

    static MultiTofRawPacket missing_imu_packet() {
        auto packet = valid_synced_packet();
        packet.has_imu = false;
        return packet;
    }

    static MultiTofRawPacket missing_wheel_packet() {
        auto packet = valid_synced_packet();
        packet.has_wheel = false;
        return packet;
    }

    static RealSensorRequestTiming request_timing_for_estimate(
        double estimated_sample_time_s) {
        return make_request_timing(estimated_sample_time_s - 0.020,
                                   estimated_sample_time_s + 0.020);
    }

    static void set_tof_time(
        MultiTofRawFrame &frame,
        double estimated_sample_time_s) {
        frame.timing = request_timing_for_estimate(estimated_sample_time_s);
    }
};

} // namespace robot_slamd
