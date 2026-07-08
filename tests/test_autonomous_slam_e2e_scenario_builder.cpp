#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_scenario_builder.hpp"

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

    AutonomousSlamE2EScenarioBuilder builder;
    const double t = 100.0;

    auto minimal = builder.build(AutonomousSlamE2EScenarioKind::MinimalMapAlreadyGood, t);
    expect(!minimal.sensor_snapshots.empty(), "minimal has sensor");
    expect(!minimal.backend_update_results.empty(), "minimal has update");
    expect(!minimal.backend_qualities.empty(), "minimal has quality");
    expect(minimal.backend_qualities.front().good_enough, "minimal quality good");

    auto active = builder.build(AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood, t);
    expect(!active.backend_qualities.front().good_enough, "active starts poor");
    expect(active.backend_qualities.back().good_enough, "active ends good");

    auto sensor_bad = builder.build(AutonomousSlamE2EScenarioKind::SensorContractFailure, t);
    expect(sensor_bad.sensor_snapshots.front().tof.ranges_m.empty(), "bad sensor has empty tof");

    auto rejected = builder.build(AutonomousSlamE2EScenarioKind::SlamBackendUpdateRejected, t);
    expect(rejected.backend_update_results.front().status ==
               SlamBackendUpdateStatus::Rejected,
           "rejected update generated");

    auto invalid_quality = builder.build(AutonomousSlamE2EScenarioKind::SlamBackendQualityInvalid, t);
    expect(invalid_quality.backend_qualities.front().coverage_ratio > 1.0,
           "invalid quality generated");

    auto motion_rejected = builder.build(AutonomousSlamE2EScenarioKind::MotionRejected, t);
    expect(motion_rejected.reject_motion, "motion reject flag set");

    auto stuck = builder.build(AutonomousSlamE2EScenarioKind::MapQualityStuck, t);
    bool all_poor = true;
    for (const auto &quality : stuck.backend_qualities) {
        all_poor = all_poor && !quality.good_enough;
    }
    expect(all_poor, "stuck quality stays poor");

    auto active_again = builder.build(AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood, t);
    expect(active.sensor_snapshots.front().tof.timestamp_s ==
               active_again.sensor_snapshots.front().tof.timestamp_s,
           "scenario deterministic");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
