#include "robot_slamd/simulation/full_pipeline/m3d0_runner.hpp"

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
    const auto scenario = M3D0ScenarioBuilder::closed_loop_active_scan();
    const auto a = M3D0Runner{}.run(scenario);
    const auto b = M3D0Runner{}.run(scenario);
    expect(a.ok && b.ok, "both deterministic runs ok");
    expect(a.physics_step_count == b.physics_step_count, "step count equal");
    expect(std::fabs(a.final_ground_truth_pose.x_m - b.final_ground_truth_pose.x_m) < 1e-12,
           "x repeatable");
    expect(std::fabs(a.final_ground_truth_pose.y_m - b.final_ground_truth_pose.y_m) < 1e-12,
           "y repeatable");
    expect(std::fabs(a.final_ground_truth_pose.yaw_rad - b.final_ground_truth_pose.yaw_rad) < 1e-12,
           "yaw repeatable");
    expect(a.sensor_publish_count == b.sensor_publish_count, "sensor publish repeatable");
    expect(a.motion_command_count == b.motion_command_count, "motion count repeatable");
    expect(a.summary == b.summary, "summary repeatable");

    const auto static_report = M3D0Runner{}.run(M3D0ScenarioBuilder::no_command_static());
    expect(static_report.ok, "static scenario ok");
    expect(!static_report.pose_changed, "no command pose unchanged");
    expect(!static_report.wheel_changed, "no command wheel unchanged");
    return failures ? 1 : 0;
}
