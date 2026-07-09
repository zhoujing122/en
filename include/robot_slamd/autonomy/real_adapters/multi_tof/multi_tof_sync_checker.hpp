#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_checker.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_degradation_policy.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_effective_time_policy.hpp"

#include <cmath>
#include <string>

namespace robot_slamd {

class MultiTofSyncChecker {
public:
    MultiTofSyncChecker(
        MultiTofContractOptions contract_options = {},
        MultiTofSyncOptions sync_options = {})
        : contract_options_(contract_options),
          sync_options_(sync_options),
          contract_checker_(contract_options),
          time_policy_(sync_options),
          degradation_policy_(sync_options) {}

    MultiTofSyncResult check_packet(
        const MultiTofRawPacket &packet,
        double now_s) const {
        if (!sync_options_.enabled) {
            return reject(MultiTofSyncFault::RawContractFailed,
                          "multi_tof_sync_disabled");
        }

        const auto contract_result = contract_checker_.check_packet(packet, now_s);
        if (sync_options_.require_raw_contract_pass && !contract_result.ok) {
            auto result = reject(MultiTofSyncFault::RawContractFailed,
                                 "multi_tof_sync_raw_contract_failed");
            result.failed.insert(result.failed.end(),
                                 contract_result.failed.begin(),
                                 contract_result.failed.end());
            return result;
        }

        const auto times = time_policy_.compute(packet);
        auto degradation = degradation_policy_.check_degradation(packet, times);
        if (!degradation.ok) {
            return degradation;
        }
        if (sync_options_.time_reference == MultiTofTimeReference::Front &&
            !times.has_front) {
            return reject(MultiTofSyncFault::InvalidEffectiveTime,
                          "multi_tof_sync_front_time_reference_missing_front");
        }
        MultiTofSyncResult result = degradation;
        result.times = times;
        result.valid_tof_count = times.present_tof_count;

        if (!time_is_valid(times.multi_tof_time_s, now_s)) {
            return reject(MultiTofSyncFault::InvalidEffectiveTime,
                          "multi_tof_sync_invalid_multi_tof_time");
        }
        if (times.has_front && !time_is_valid(times.front_time_s, now_s)) {
            return reject(MultiTofSyncFault::InvalidEffectiveTime,
                          "multi_tof_sync_invalid_front_time");
        }
        if (times.has_left && !time_is_valid(times.left_time_s, now_s)) {
            return reject(MultiTofSyncFault::InvalidEffectiveTime,
                          "multi_tof_sync_invalid_left_time");
        }
        if (times.has_right && !time_is_valid(times.right_time_s, now_s)) {
            return reject(MultiTofSyncFault::InvalidEffectiveTime,
                          "multi_tof_sync_invalid_right_time");
        }

        if (times.has_front && times.has_left) {
            result.front_left_dt_s = abs_dt(times.front_time_s, times.left_time_s);
            if (result.front_left_dt_s > sync_options_.max_pairwise_tof_sync_dt_s) {
                result.fault = MultiTofSyncFault::FrontLeftSyncTooLarge;
                result.status = MultiTofSyncStatus::Rejected;
                result.ok = false;
                result.failed.push_back("multi_tof_front_left_sync_too_large");
                result.message = "multi_tof_front_left_sync_too_large";
                return result;
            }
        }
        if (times.has_front && times.has_right) {
            result.front_right_dt_s = abs_dt(times.front_time_s, times.right_time_s);
            if (result.front_right_dt_s > sync_options_.max_pairwise_tof_sync_dt_s) {
                result.fault = MultiTofSyncFault::FrontRightSyncTooLarge;
                result.status = MultiTofSyncStatus::Rejected;
                result.ok = false;
                result.failed.push_back("multi_tof_front_right_sync_too_large");
                result.message = "multi_tof_front_right_sync_too_large";
                return result;
            }
        }
        if (times.has_left && times.has_right) {
            result.left_right_dt_s = abs_dt(times.left_time_s, times.right_time_s);
            if (result.left_right_dt_s > sync_options_.max_pairwise_tof_sync_dt_s) {
                result.fault = MultiTofSyncFault::LeftRightSyncTooLarge;
                result.status = MultiTofSyncStatus::Rejected;
                result.ok = false;
                result.failed.push_back("multi_tof_left_right_sync_too_large");
                result.message = "multi_tof_left_right_sync_too_large";
                return result;
            }
        }

        if (sync_options_.require_imu && !times.has_imu) {
            return reject(MultiTofSyncFault::MissingImu,
                          "multi_tof_sync_missing_imu");
        }
        if (sync_options_.require_wheel && !times.has_wheel) {
            return reject(MultiTofSyncFault::MissingWheel,
                          "multi_tof_sync_missing_wheel");
        }
        if (times.has_imu) {
            if (!time_is_valid(times.imu_time_s, now_s)) {
                return reject(MultiTofSyncFault::InvalidEffectiveTime,
                              "multi_tof_sync_invalid_imu_time");
            }
            result.multi_tof_imu_dt_s = abs_dt(times.multi_tof_time_s, times.imu_time_s);
            if (result.multi_tof_imu_dt_s > sync_options_.max_multi_tof_imu_sync_dt_s) {
                result.fault = MultiTofSyncFault::MultiTofImuSyncTooLarge;
                result.status = MultiTofSyncStatus::Rejected;
                result.ok = false;
                result.failed.push_back("multi_tof_imu_sync_too_large");
                result.message = "multi_tof_imu_sync_too_large";
                return result;
            }
        }
        if (times.has_wheel) {
            if (!time_is_valid(times.wheel_time_s, now_s)) {
                return reject(MultiTofSyncFault::InvalidEffectiveTime,
                              "multi_tof_sync_invalid_wheel_time");
            }
            result.multi_tof_wheel_dt_s = abs_dt(times.multi_tof_time_s, times.wheel_time_s);
            if (result.multi_tof_wheel_dt_s > sync_options_.max_multi_tof_wheel_sync_dt_s) {
                result.fault = MultiTofSyncFault::MultiTofWheelSyncTooLarge;
                result.status = MultiTofSyncStatus::Rejected;
                result.ok = false;
                result.failed.push_back("multi_tof_wheel_sync_too_large");
                result.message = "multi_tof_wheel_sync_too_large";
                return result;
            }
        }

        result.ok = true;
        result.status = result.degraded ? MultiTofSyncStatus::AcceptedDegraded
                                        : MultiTofSyncStatus::Accepted;
        result.fault = MultiTofSyncFault::None;
        result.passed.push_back("multi_tof_pairwise_sync_ok");
        result.passed.push_back("multi_tof_imu_wheel_sync_ok");
        result.message = result.degraded ? "multi_tof_sync_ok_degraded"
                                         : "multi_tof_sync_ok";
        return result;
    }

private:
    MultiTofContractOptions contract_options_;
    MultiTofSyncOptions sync_options_;
    MultiTofContractChecker contract_checker_;
    MultiTofEffectiveTimePolicy time_policy_;
    MultiTofDegradationPolicy degradation_policy_;

    bool time_is_valid(double time_s, double now_s) const {
        return std::isfinite(time_s) &&
               time_s - now_s <= sync_options_.max_effective_time_future_skew_s;
    }

    MultiTofSyncResult reject(
        MultiTofSyncFault fault,
        const std::string &message) const {
        MultiTofSyncResult result;
        result.ok = false;
        result.status = MultiTofSyncStatus::Rejected;
        result.fault = fault;
        result.failed.push_back(message);
        result.message = message;
        return result;
    }
};

} // namespace robot_slamd
