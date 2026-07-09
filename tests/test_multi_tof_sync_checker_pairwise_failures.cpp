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
    auto result = checker.check_packet(MultiTofSyncSampleData::front_left_sync_too_large_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSyncFault::FrontLeftSyncTooLarge, "front-left fault");
    expect(result.front_left_dt_s > options.max_pairwise_tof_sync_dt_s, "front-left dt recorded");
    result = checker.check_packet(MultiTofSyncSampleData::front_right_sync_too_large_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSyncFault::FrontRightSyncTooLarge, "front-right fault");
    expect(result.front_right_dt_s > options.max_pairwise_tof_sync_dt_s, "front-right dt recorded");
    result = checker.check_packet(MultiTofSyncSampleData::left_right_sync_too_large_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSyncFault::LeftRightSyncTooLarge, "left-right fault");
    expect(result.left_right_dt_s > options.max_pairwise_tof_sync_dt_s, "left-right dt recorded");
    return failures ? 1 : 0;
}
