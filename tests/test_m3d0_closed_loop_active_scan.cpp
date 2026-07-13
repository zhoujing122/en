#include "robot_slamd/simulation/full_pipeline/m3d0_runner.hpp"

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
    const auto scenario = M3D0ScenarioBuilder::closed_loop_active_scan();
    const auto report = M3D0Runner{}.run(scenario);
    expect(report.ok, "closed loop report ok");
    expect(report.closed_loop_verified, "closed loop verified");
    expect(report.pose_changed, "coordinator command changed plant pose");
    expect(report.wheel_changed, "plant changed wheel");
    expect(report.imu_changed, "plant changed imu");
    expect(report.sensor_publish_count >= 2, "settle then publish new snapshot");
    expect(report.non_instant_motion_verified, "motion not instant");
    expect(report.motion_settle_verified, "motion settled");
    expect(!report.forward_seen, "no forward in active scan");
    expect(!report.backward_seen, "no backward in active scan");
    expect(!report.real_hardware_accessed, "no real hardware");
    expect(!report.real_motion_attempted, "no real motion");
    expect(report.ground_truth_isolation_verified, "ground truth isolation flag");
    return failures ? 1 : 0;
}
