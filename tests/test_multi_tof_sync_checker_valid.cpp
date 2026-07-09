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
    const auto result = MultiTofSyncChecker({}, options).check_packet(MultiTofSyncSampleData::valid_synced_packet(), 100.200);
    expect(result.ok, "valid pass");
    expect(result.status == MultiTofSyncStatus::Accepted, "status accepted");
    expect(result.valid_tof_count == 3, "valid count 3");
    expect(!result.degraded, "not degraded");
    expect(result.front_left_dt_s < options.max_pairwise_tof_sync_dt_s, "front-left below threshold");
    expect(result.front_right_dt_s < options.max_pairwise_tof_sync_dt_s, "front-right below threshold");
    expect(result.left_right_dt_s < options.max_pairwise_tof_sync_dt_s, "left-right below threshold");
    expect(result.message == "multi_tof_sync_ok", "message ok");
    return failures ? 1 : 0;
}
