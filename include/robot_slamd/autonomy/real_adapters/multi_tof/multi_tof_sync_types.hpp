#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_types.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace robot_slamd {

enum class MultiTofSyncStatus {
    Accepted,
    AcceptedDegraded,
    Rejected,
    Fault
};

enum class MultiTofSyncFault {
    None,
    RawContractFailed,
    TooFewTofFrames,
    MissingRequiredTof,
    FrontLeftSyncTooLarge,
    FrontRightSyncTooLarge,
    LeftRightSyncTooLarge,
    MultiTofImuSyncTooLarge,
    MultiTofWheelSyncTooLarge,
    MissingImu,
    MissingWheel,
    InvalidEffectiveTime,
    DegradationNotAllowed,
    DuplicateEffectiveTimeSource
};

enum class MultiTofTimeReference {
    Front,
    MedianPresentTof,
    MeanPresentTof
};

enum class MultiTofDegradationMode {
    RequireAll,
    AllowMissingOne,
    AllowAnyTwo,
    AllowFrontOnly
};

struct MultiTofEffectiveTimes {
    bool has_front = false;
    bool has_left = false;
    bool has_right = false;
    bool has_imu = false;
    bool has_wheel = false;
    double front_time_s = 0.0;
    double left_time_s = 0.0;
    double right_time_s = 0.0;
    double imu_time_s = 0.0;
    double wheel_time_s = 0.0;
    double multi_tof_time_s = 0.0;
    int present_tof_count = 0;
};

struct MultiTofSyncOptions {
    bool enabled = false;
    bool require_raw_contract_pass = true;
    bool require_imu = true;
    bool require_wheel = true;
    MultiTofTimeReference time_reference =
        MultiTofTimeReference::MedianPresentTof;
    MultiTofDegradationMode degradation_mode =
        MultiTofDegradationMode::RequireAll;
    int min_required_tof_count = 3;
    double max_pairwise_tof_sync_dt_s = 0.050;
    double max_multi_tof_imu_sync_dt_s = 0.100;
    double max_multi_tof_wheel_sync_dt_s = 0.100;
    double max_effective_time_future_skew_s = 0.050;
};

struct MultiTofSyncResult {
    bool ok = false;
    MultiTofSyncStatus status = MultiTofSyncStatus::Rejected;
    MultiTofSyncFault fault = MultiTofSyncFault::None;
    MultiTofEffectiveTimes times;
    double front_left_dt_s = 0.0;
    double front_right_dt_s = 0.0;
    double left_right_dt_s = 0.0;
    double multi_tof_imu_dt_s = 0.0;
    double multi_tof_wheel_dt_s = 0.0;
    int valid_tof_count = 0;
    bool degraded = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string message;
};

inline std::string to_string(MultiTofSyncStatus status) {
    switch (status) {
    case MultiTofSyncStatus::Accepted:
        return "accepted";
    case MultiTofSyncStatus::AcceptedDegraded:
        return "accepted_degraded";
    case MultiTofSyncStatus::Rejected:
        return "rejected";
    case MultiTofSyncStatus::Fault:
        return "fault";
    }
    return "unknown";
}

inline std::string to_string(MultiTofSyncFault fault) {
    switch (fault) {
    case MultiTofSyncFault::None:
        return "none";
    case MultiTofSyncFault::RawContractFailed:
        return "raw_contract_failed";
    case MultiTofSyncFault::TooFewTofFrames:
        return "too_few_tof_frames";
    case MultiTofSyncFault::MissingRequiredTof:
        return "missing_required_tof";
    case MultiTofSyncFault::FrontLeftSyncTooLarge:
        return "front_left_sync_too_large";
    case MultiTofSyncFault::FrontRightSyncTooLarge:
        return "front_right_sync_too_large";
    case MultiTofSyncFault::LeftRightSyncTooLarge:
        return "left_right_sync_too_large";
    case MultiTofSyncFault::MultiTofImuSyncTooLarge:
        return "multi_tof_imu_sync_too_large";
    case MultiTofSyncFault::MultiTofWheelSyncTooLarge:
        return "multi_tof_wheel_sync_too_large";
    case MultiTofSyncFault::MissingImu:
        return "missing_imu";
    case MultiTofSyncFault::MissingWheel:
        return "missing_wheel";
    case MultiTofSyncFault::InvalidEffectiveTime:
        return "invalid_effective_time";
    case MultiTofSyncFault::DegradationNotAllowed:
        return "degradation_not_allowed";
    case MultiTofSyncFault::DuplicateEffectiveTimeSource:
        return "duplicate_effective_time_source";
    }
    return "unknown";
}

inline std::string to_string(MultiTofTimeReference reference) {
    switch (reference) {
    case MultiTofTimeReference::Front:
        return "front";
    case MultiTofTimeReference::MedianPresentTof:
        return "median_present_tof";
    case MultiTofTimeReference::MeanPresentTof:
        return "mean_present_tof";
    }
    return "unknown";
}

inline std::string to_string(MultiTofDegradationMode mode) {
    switch (mode) {
    case MultiTofDegradationMode::RequireAll:
        return "require_all";
    case MultiTofDegradationMode::AllowMissingOne:
        return "allow_missing_one";
    case MultiTofDegradationMode::AllowAnyTwo:
        return "allow_any_two";
    case MultiTofDegradationMode::AllowFrontOnly:
        return "allow_front_only";
    }
    return "unknown";
}

inline int multi_tof_sync_status_id(MultiTofSyncStatus status) {
    return static_cast<int>(status);
}

inline int multi_tof_sync_fault_id(MultiTofSyncFault fault) {
    return static_cast<int>(fault);
}

inline int multi_tof_time_reference_id(MultiTofTimeReference reference) {
    return static_cast<int>(reference);
}

inline int multi_tof_degradation_mode_id(MultiTofDegradationMode mode) {
    return static_cast<int>(mode);
}

inline double abs_dt(double lhs_s, double rhs_s) {
    return std::fabs(lhs_s - rhs_s);
}

inline double median_time(std::vector<double> times_s) {
    if (times_s.empty()) {
        return 0.0;
    }
    std::sort(times_s.begin(), times_s.end());
    const std::size_t mid = times_s.size() / 2;
    if (times_s.size() % 2 == 1) {
        return times_s[mid];
    }
    return 0.5 * (times_s[mid - 1] + times_s[mid]);
}

inline double mean_time(const std::vector<double> &times_s) {
    if (times_s.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (const double time_s : times_s) {
        sum += time_s;
    }
    return sum / static_cast<double>(times_s.size());
}

} // namespace robot_slamd
