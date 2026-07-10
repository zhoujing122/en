#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_sample_data.hpp"

namespace robot_slamd {

class MultiTofSnapshotSampleData {
public:
    static MultiTofRawPacket valid_synced_packet() {
        return MultiTofSyncSampleData::valid_synced_packet();
    }

    static MultiTofRawPacket invalid_sync_packet() {
        return MultiTofSyncSampleData::front_left_sync_too_large_packet();
    }

    static MultiTofRawPacket missing_left_degraded_packet() {
        return MultiTofSyncSampleData::missing_left_degradable_packet();
    }

    static MultiTofRawPacket missing_front_packet() {
        auto packet = MultiTofSyncSampleData::valid_synced_packet();
        packet.has_front = false;
        return packet;
    }

    static MultiTofRawPacket median_prefers_left_packet() {
        auto packet = MultiTofSyncSampleData::valid_synced_packet();
        MultiTofSyncSampleData::set_tof_time(packet.front, 99.980);
        MultiTofSyncSampleData::set_tof_time(packet.left, 100.000);
        MultiTofSyncSampleData::set_tof_time(packet.right, 100.020);
        packet.imu.timestamp_s = 100.000;
        packet.wheel.timing =
            MultiTofSyncSampleData::request_timing_for_estimate(100.000);
        return packet;
    }
};

} // namespace robot_slamd
