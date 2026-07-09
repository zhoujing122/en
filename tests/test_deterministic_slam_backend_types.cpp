#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_types.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
} // namespace

int main() {
    using namespace robot_slamd;
    expect(to_string(DeterministicSlamBackendStage::Idle) == "idle", "stage string");
    expect(to_string(DeterministicSlamBackendFault::MissingTof) == "missing_tof", "fault string");
    expect(to_string(DeterministicSlamBackendUpdateKind::KeyframeAdded) == "keyframe_added", "kind string");
    expect(deterministic_slam_backend_stage_id(DeterministicSlamBackendStage::Ready) == 4, "stage id");
    expect(deterministic_slam_backend_fault_id(DeterministicSlamBackendFault::BackendNotReady) == 2, "fault id");
    expect(deterministic_slam_backend_update_kind_id(DeterministicSlamBackendUpdateKind::Rejected) == 3, "kind id");
    DeterministicSlamBackendOptions options;
    expect(!options.enabled, "default disabled");
    expect(!options.ready, "default not ready");
    DeterministicSlamBackendState state;
    expect(state.update_call_count == 0, "default update count zero");
    expect(state.valid_scan_count == 0, "default scan count zero");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
