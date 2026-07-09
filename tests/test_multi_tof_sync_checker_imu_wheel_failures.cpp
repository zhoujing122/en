#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_checker.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_sample_data.hpp"

#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    MultiTofSyncOptions options;
    options.enabled = true;
    MultiTofSyncChecker checker({}, options);
    auto result = checker.check_packet(MultiTofSyncSampleData::imu_sync_too_large_packet(), 100.300);
    expect(!result.ok && result.fault == MultiTofSyncFault::MultiTofImuSyncTooLarge, "imu sync fault");
    result = checker.check_packet(MultiTofSyncSampleData::wheel_sync_too_large_packet(), 100.300);
    expect(!result.ok && result.fault == MultiTofSyncFault::MultiTofWheelSyncTooLarge, "wheel sync fault");
    result = checker.check_packet(MultiTofSyncSampleData::missing_imu_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSyncFault::MissingImu, "missing imu fault");
    result = checker.check_packet(MultiTofSyncSampleData::missing_wheel_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSyncFault::MissingWheel, "missing wheel fault");
    options.require_imu = false;
    checker = MultiTofSyncChecker({}, options);
    expect(checker.check_packet(MultiTofSyncSampleData::missing_imu_packet(), 100.200).ok, "missing imu allowed");
    options.require_imu = true;
    options.require_wheel = false;
    checker = MultiTofSyncChecker({}, options);
    expect(checker.check_packet(MultiTofSyncSampleData::missing_wheel_packet(), 100.200).ok, "missing wheel allowed");
    return failures ? 1 : 0;
}
