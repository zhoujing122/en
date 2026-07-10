#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_sample_data.hpp"

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
}

int main() {
    using namespace robot_slamd;
    MultiTofSyncOptions sync;
    sync.enabled = true;
    MultiTofSnapshotBuildOptions build;
    build.enabled = true;
    const auto result = MultiTofSnapshotBuilder({}, sync, build).build(
        MultiTofSnapshotSampleData::valid_synced_packet(), 100.200);
    expect(result.ok, "valid build pass");
    expect(result.snapshot.has_multi_tof, "has multi tof");
    expect(result.snapshot.multi_tof.has_front, "front present");
    expect(result.snapshot.multi_tof.has_left, "left present");
    expect(result.snapshot.multi_tof.has_right, "right present");
    expect(std::fabs(result.snapshot.multi_tof.synchronized_time_s -
                     result.sync_result.times.multi_tof_time_s) < 1e-9,
           "synchronized time from sync");
    expect(std::fabs(result.snapshot.imu.timestamp_s - 100.000) < 1e-9,
           "imu timestamp preserved");
    expect(std::fabs(result.snapshot.wheel.timestamp_s - 100.000) < 1e-9,
           "wheel timestamp estimated sample time");
    expect(result.valid_tof_count == 3, "valid count 3");
    expect(!result.degraded, "not degraded");
    return failures ? 1 : 0;
}
