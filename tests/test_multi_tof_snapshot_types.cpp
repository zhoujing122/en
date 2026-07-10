#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_types.hpp"

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
    MultiTofSnapshotBuildOptions options;
    expect(!options.enabled, "default disabled");
    expect(options.legacy_primary_mode == MultiTofLegacyPrimaryMode::Front,
           "default front legacy primary");
    expect(to_string(MultiTofSnapshotBuildStatus::Built) == "built",
           "status string");
    expect(to_string(MultiTofSnapshotBuildFault::SyncFailed) == "sync_failed",
           "fault string");
    expect(to_string(MultiTofLegacyPrimaryMode::FirstPresent) == "first_present",
           "mode string");
    expect(multi_tof_snapshot_build_status_id(
               MultiTofSnapshotBuildStatus::BuiltDegraded) >= 0,
           "status id stable");
    RobotSlamSensorSnapshot snapshot;
    expect(!snapshot.has_multi_tof, "snapshot default no multi tof");
    return failures ? 1 : 0;
}
