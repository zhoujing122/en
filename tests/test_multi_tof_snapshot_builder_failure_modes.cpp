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
    auto result = MultiTofSnapshotBuilder({}, sync, build).build(
        MultiTofSnapshotSampleData::valid_synced_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSnapshotBuildFault::BuilderDisabled,
           "builder disabled fail");

    build.enabled = true;
    result = MultiTofSnapshotBuilder({}, sync, build).build(
        MultiTofSnapshotSampleData::invalid_sync_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSnapshotBuildFault::SyncFailed,
           "sync fail build fail");

    MultiTofContractOptions contract;
    contract.require_left = false;
    contract.min_required_tof_count = 2;
    sync.degradation_mode = MultiTofDegradationMode::AllowMissingOne;
    sync.min_required_tof_count = 2;
    build.min_required_tof_count = 3;
    result = MultiTofSnapshotBuilder(contract, sync, build).build(
        MultiTofSnapshotSampleData::missing_left_degraded_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSnapshotBuildFault::TooFewTofFrames,
           "too few tof fail");

    contract.require_front = false;
    contract.require_left = true;
    contract.min_required_tof_count = 2;
    sync.degradation_mode = MultiTofDegradationMode::AllowAnyTwo;
    sync.min_required_tof_count = 2;
    build.min_required_tof_count = 2;
    build.legacy_primary_mode = MultiTofLegacyPrimaryMode::Front;
    result = MultiTofSnapshotBuilder(contract, sync, build).build(
        MultiTofSnapshotSampleData::missing_front_packet(), 100.200);
    expect(!result.ok && result.fault == MultiTofSnapshotBuildFault::LegacyPrimaryMissing,
           "missing front legacy primary fail");
    return failures ? 1 : 0;
}
