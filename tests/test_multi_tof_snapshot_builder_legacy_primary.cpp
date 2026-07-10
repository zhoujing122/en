#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_sample_data.hpp"

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
    auto result = MultiTofSnapshotBuilder({}, sync, build).build(
        MultiTofSnapshotSampleData::valid_synced_packet(), 100.200);
    expect(result.ok && result.snapshot.tof.frame_id == "tof_front_frame",
           "default front primary");

    build.legacy_primary_mode = MultiTofLegacyPrimaryMode::MedianTimeClosest;
    result = MultiTofSnapshotBuilder({}, sync, build).build(
        MultiTofSnapshotSampleData::median_prefers_left_packet(), 100.200);
    expect(result.ok && result.snapshot.tof.frame_id == "tof_left_frame",
           "median closest selects left");

    MultiTofContractOptions contract;
    contract.require_front = false;
    contract.min_required_tof_count = 2;
    sync.degradation_mode = MultiTofDegradationMode::AllowAnyTwo;
    sync.min_required_tof_count = 2;
    build.min_required_tof_count = 2;
    build.legacy_primary_mode = MultiTofLegacyPrimaryMode::FirstPresent;
    result = MultiTofSnapshotBuilder(contract, sync, build).build(
        MultiTofSnapshotSampleData::missing_front_packet(), 100.200);
    expect(result.ok && result.snapshot.tof.frame_id == "tof_left_frame",
           "first present selects left when front missing");

    build.populate_legacy_tof = false;
    result = MultiTofSnapshotBuilder({}, MultiTofSyncOptions{true}, build).build(
        MultiTofSnapshotSampleData::valid_synced_packet(), 100.200);
    expect(result.ok && result.snapshot.has_multi_tof && !result.snapshot.has_tof,
           "legacy tof can be disabled");
    return failures ? 1 : 0;
}
