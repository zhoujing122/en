#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_log_codec.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_sample_data.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

class MultiTofReplaySampleLog {
public:
    static std::vector<std::string> valid_log_lines() {
        return {"# multi tof replay sample", line_for(MultiTofSyncSampleData::valid_synced_packet())};
    }

    static std::vector<std::string> valid_two_packet_log_lines() {
        auto second = MultiTofSyncSampleData::valid_synced_packet();
        MultiTofSyncSampleData::set_tof_time(second.front, 100.100);
        MultiTofSyncSampleData::set_tof_time(second.left, 100.110);
        MultiTofSyncSampleData::set_tof_time(second.right, 100.095);
        second.imu.timestamp_s = 100.100;
        second.wheel.timing = MultiTofSyncSampleData::request_timing_for_estimate(100.100);
        second.packet_timestamp_s = 100.100;
        return {line_for(MultiTofSyncSampleData::valid_synced_packet()), line_for(second)};
    }

    static std::vector<std::string> invalid_numeric_log_lines() {
        auto line = line_for(MultiTofSyncSampleData::valid_synced_packet());
        replace_token(line, "front_est=100", "front_est=bad");
        return {line};
    }

    static std::vector<std::string> missing_field_log_lines() {
        auto line = line_for(MultiTofSyncSampleData::valid_synced_packet());
        replace_token(line, " left_ranges=1,1.2,1.4,1.6", "");
        return {line};
    }

    static std::vector<std::string> empty_ranges_log_lines() {
        auto line = line_for(MultiTofSyncSampleData::valid_synced_packet());
        replace_token(line, "right_ranges=1,1.2,1.4,1.6", "right_ranges=");
        return {line};
    }

    static std::vector<std::string> invalid_bool_log_lines() {
        auto line = line_for(MultiTofSyncSampleData::valid_synced_packet());
        replace_token(line, "front_present=1", "front_present=maybe");
        return {line};
    }

    static std::vector<std::string> pairwise_sync_failure_log_lines() {
        return {line_for(MultiTofSyncSampleData::front_left_sync_too_large_packet())};
    }

    static std::vector<std::string> imu_sync_failure_log_lines() {
        return {line_for(MultiTofSyncSampleData::imu_sync_too_large_packet())};
    }

    static std::vector<std::string> wheel_sync_failure_log_lines() {
        return {line_for(MultiTofSyncSampleData::wheel_sync_too_large_packet())};
    }

    static std::vector<std::string> degraded_missing_left_log_lines() {
        return {line_for(MultiTofSyncSampleData::missing_left_degradable_packet())};
    }

private:
    static std::string line_for(const MultiTofRawPacket &packet) {
        return MultiTofReplayLogCodec().encode_record(packet);
    }

    static void replace_token(
        std::string &line,
        const std::string &from,
        const std::string &to) {
        const auto pos = line.find(from);
        if (pos != std::string::npos) {
            line.replace(pos, from.size(), to);
        }
    }
};

} // namespace robot_slamd
