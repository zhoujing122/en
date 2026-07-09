#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_effective_time_policy.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_sample_data.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
void expect_near(double lhs, double rhs, const char *message) {
    expect(std::fabs(lhs - rhs) < 1e-9, message);
}
}

int main() {
    using namespace robot_slamd;
    auto packet = MultiTofSyncSampleData::valid_synced_packet();
    MultiTofSyncOptions options;
    options.time_reference = MultiTofTimeReference::MedianPresentTof;
    auto times = MultiTofEffectiveTimePolicy(options).compute(packet);
    expect_near(times.front_time_s, packet.front.timing.estimated_sample_time_s, "front estimated time");
    expect_near(times.left_time_s, packet.left.timing.estimated_sample_time_s, "left estimated time");
    expect_near(times.right_time_s, packet.right.timing.estimated_sample_time_s, "right estimated time");
    expect_near(times.wheel_time_s, packet.wheel.timing.estimated_sample_time_s, "wheel estimated time");
    expect_near(times.imu_time_s, packet.imu.timestamp_s, "imu timestamp");
    expect_near(times.multi_tof_time_s, 100.000, "median present tof");
    options.time_reference = MultiTofTimeReference::MeanPresentTof;
    times = MultiTofEffectiveTimePolicy(options).compute(packet);
    expect_near(times.multi_tof_time_s, (100.000 + 100.010 + 99.995) / 3.0, "mean present tof");
    options.time_reference = MultiTofTimeReference::Front;
    times = MultiTofEffectiveTimePolicy(options).compute(packet);
    expect_near(times.multi_tof_time_s, 100.000, "front reference");
    packet.has_left = false;
    options.time_reference = MultiTofTimeReference::MedianPresentTof;
    times = MultiTofEffectiveTimePolicy(options).compute(packet);
    expect(times.present_tof_count == 2, "missing one count");
    expect_near(times.multi_tof_time_s, (100.000 + 99.995) / 2.0, "missing one median");
    return failures ? 1 : 0;
}
