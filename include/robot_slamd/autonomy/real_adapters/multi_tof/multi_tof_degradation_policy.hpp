#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_types.hpp"

namespace robot_slamd {

class MultiTofDegradationPolicy {
public:
    explicit MultiTofDegradationPolicy(
        MultiTofSyncOptions options = {})
        : options_(options) {}

    MultiTofSyncResult check_degradation(
        const MultiTofRawPacket &packet,
        const MultiTofEffectiveTimes &times) const {
        MultiTofSyncResult result;
        result.times = times;
        result.valid_tof_count = times.present_tof_count;
        if (times.present_tof_count < options_.min_required_tof_count) {
            return reject(MultiTofSyncFault::TooFewTofFrames,
                          "multi_tof_sync_too_few_tof_frames");
        }
        switch (options_.degradation_mode) {
        case MultiTofDegradationMode::RequireAll:
            if (!packet.has_front || !packet.has_left || !packet.has_right ||
                times.present_tof_count != 3) {
                return reject(MultiTofSyncFault::DegradationNotAllowed,
                              "multi_tof_sync_degradation_not_allowed");
            }
            break;
        case MultiTofDegradationMode::AllowMissingOne:
            if (times.present_tof_count < 2) {
                return reject(MultiTofSyncFault::TooFewTofFrames,
                              "multi_tof_sync_too_few_tof_frames");
            }
            result.degraded = times.present_tof_count < 3;
            break;
        case MultiTofDegradationMode::AllowAnyTwo:
            if (times.present_tof_count < 2) {
                return reject(MultiTofSyncFault::TooFewTofFrames,
                              "multi_tof_sync_too_few_tof_frames");
            }
            result.degraded = times.present_tof_count < 3;
            break;
        case MultiTofDegradationMode::AllowFrontOnly:
            if (!packet.has_front) {
                return reject(MultiTofSyncFault::MissingRequiredTof,
                              "multi_tof_sync_missing_front");
            }
            if (times.present_tof_count < 1) {
                return reject(MultiTofSyncFault::TooFewTofFrames,
                              "multi_tof_sync_too_few_tof_frames");
            }
            result.degraded = times.present_tof_count < 3;
            break;
        }
        result.ok = true;
        result.status = result.degraded ? MultiTofSyncStatus::AcceptedDegraded
                                        : MultiTofSyncStatus::Accepted;
        result.fault = MultiTofSyncFault::None;
        result.message = result.degraded ? "multi_tof_degradation_allowed"
                                         : "multi_tof_degradation_not_needed";
        result.passed.push_back("multi_tof_degradation_policy_ok");
        if (result.degraded) {
            result.warnings.push_back("multi_tof_sync_degraded_missing_tof");
        }
        return result;
    }

private:
    MultiTofSyncOptions options_;

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
