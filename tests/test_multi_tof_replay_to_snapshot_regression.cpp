#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_to_snapshot_regression_runner.hpp"

#include <iostream>
#include <string>

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
    const auto report = MultiTofReplayToSnapshotRegressionRunner().run_once();
    expect(report.ok, "regression ok");
    expect(report.valid_result.snapshot.has_multi_tof, "has multi tof");
    expect(report.valid_result.snapshot.multi_tof.has_front, "front preserved");
    expect(report.valid_result.snapshot.multi_tof.has_left, "left preserved");
    expect(report.valid_result.snapshot.multi_tof.has_right, "right preserved");
    expect(report.valid_result.snapshot.multi_tof.synchronized_time_s > 99.9,
           "synchronized time stable");
    expect(report.valid_result.snapshot.imu.timestamp_s == 100.0,
           "imu timestamp preserved");
    expect(report.valid_result.snapshot.wheel.timestamp_s == 100.0,
           "wheel estimated timestamp");
    expect(report.valid_result.snapshot.has_tof, "legacy tof true");
    expect(!report.rejected_result.ok, "pairwise rejected");
    expect(report.degraded_result.ok && report.degraded_result.snapshot.multi_tof.degraded,
           "degraded missing-left passes");
    return failures ? 1 : 0;
}
