#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_degradation_policy.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_effective_time_policy.hpp"
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
robot_slamd::MultiTofSyncResult check(robot_slamd::MultiTofRawPacket packet, robot_slamd::MultiTofSyncOptions options) {
    robot_slamd::MultiTofEffectiveTimePolicy time_policy(options);
    return robot_slamd::MultiTofDegradationPolicy(options).check_degradation(packet, time_policy.compute(packet));
}
}

int main() {
    using namespace robot_slamd;
    MultiTofSyncOptions options;
    options.degradation_mode = MultiTofDegradationMode::RequireAll;
    expect(check(MultiTofSyncSampleData::valid_synced_packet(), options).ok, "require all present pass");
    expect(!check(MultiTofSyncSampleData::missing_left_degradable_packet(), options).ok, "require all missing left fail");
    options.degradation_mode = MultiTofDegradationMode::AllowMissingOne;
    options.min_required_tof_count = 2;
    auto result = check(MultiTofSyncSampleData::missing_left_degradable_packet(), options);
    expect(result.ok && result.degraded, "allow missing one degraded pass");
    options.degradation_mode = MultiTofDegradationMode::AllowAnyTwo;
    auto packet = MultiTofSyncSampleData::valid_synced_packet();
    packet.has_front = false;
    result = check(packet, options);
    expect(result.ok && result.degraded, "allow any two without front pass");
    options.degradation_mode = MultiTofDegradationMode::AllowFrontOnly;
    options.min_required_tof_count = 1;
    packet = MultiTofSyncSampleData::valid_synced_packet();
    packet.has_left = false;
    packet.has_right = false;
    result = check(packet, options);
    expect(result.ok && result.degraded, "front only pass degraded");
    options.min_required_tof_count = 2;
    expect(!check(packet, options).ok, "min required still enforced");
    return failures ? 1 : 0;
}
