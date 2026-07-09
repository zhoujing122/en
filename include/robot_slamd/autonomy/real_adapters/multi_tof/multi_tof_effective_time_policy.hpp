#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_types.hpp"

namespace robot_slamd {

class MultiTofEffectiveTimePolicy {
public:
    explicit MultiTofEffectiveTimePolicy(
        MultiTofSyncOptions options = {})
        : options_(options) {}

    MultiTofEffectiveTimes compute(
        const MultiTofRawPacket &packet) const {
        MultiTofEffectiveTimes times;
        std::vector<double> present_tof_times;
        times.has_front = packet.has_front;
        times.has_left = packet.has_left;
        times.has_right = packet.has_right;
        times.has_imu = packet.has_imu;
        times.has_wheel = packet.has_wheel;
        if (packet.has_front) {
            times.front_time_s = packet.front.timing.estimated_sample_time_s;
            present_tof_times.push_back(times.front_time_s);
        }
        if (packet.has_left) {
            times.left_time_s = packet.left.timing.estimated_sample_time_s;
            present_tof_times.push_back(times.left_time_s);
        }
        if (packet.has_right) {
            times.right_time_s = packet.right.timing.estimated_sample_time_s;
            present_tof_times.push_back(times.right_time_s);
        }
        if (packet.has_imu) {
            times.imu_time_s = packet.imu.timestamp_s;
        }
        if (packet.has_wheel) {
            times.wheel_time_s = packet.wheel.timing.estimated_sample_time_s;
        }
        times.present_tof_count = static_cast<int>(present_tof_times.size());
        times.multi_tof_time_s = compute_multi_tof_time(present_tof_times);
        return times;
    }

private:
    MultiTofSyncOptions options_;

    double compute_multi_tof_time(
        const std::vector<double> &present_tof_times) const {
        if (present_tof_times.empty()) {
            return 0.0;
        }
        switch (options_.time_reference) {
        case MultiTofTimeReference::Front:
            return present_tof_times.front();
        case MultiTofTimeReference::MedianPresentTof:
            return median_time(present_tof_times);
        case MultiTofTimeReference::MeanPresentTof:
            return mean_time(present_tof_times);
        }
        return 0.0;
    }
};

} // namespace robot_slamd
