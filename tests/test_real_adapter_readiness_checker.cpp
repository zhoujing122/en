#include "robot_slamd/autonomy/autonomous_slam_fake_ports.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_readiness_checker.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

std::string first_error(const robot_slamd::RealAdapterReadinessReport &report) {
    return report.violations.empty() ? "" : report.violations.front().code;
}
} // namespace

int main() {
    using namespace robot_slamd;

    FakeRobotSlamSensorPort sensor;
    FakeRobotSlamMotionPort motion;
    FakeRobotSlamMapPort map;
    FixedStepTimePort time;
    RealAdapterReadinessChecker checker;

    auto ready = checker.check_ports(&sensor, &motion, &map, &time);
    expect(ready.ok, "all ports ready");
    expect(ready.readiness == RealAdapterReadiness::ShadowReady, "shadow ready only");
    expect(!ready.checklist.empty() && ready.checklist.front() == "software_transport_shadow_acceptance_passed", "checklist includes shadow");

    expect(first_error(checker.check_ports(nullptr, &motion, &map, &time)) == "sensor_port_missing", "sensor missing");
    sensor.ready_flag = false;
    expect(first_error(checker.check_ports(&sensor, &motion, &map, &time)) == "sensor_port_not_ready", "sensor not ready");
    sensor.ready_flag = true;
    expect(first_error(checker.check_ports(&sensor, nullptr, &map, &time)) == "motion_port_missing", "motion missing");
    expect(first_error(checker.check_ports(&sensor, &motion, nullptr, &time)) == "map_port_missing", "map missing");

    expect(checker.check_ports(&sensor, &motion, &map, nullptr).ok, "time optional by default");
    RealAdapterReadinessOptions options;
    options.require_time_port_ready = true;
    RealAdapterReadinessChecker time_checker(options);
    expect(first_error(time_checker.check_ports(&sensor, &motion, &map, nullptr)) == "time_port_missing", "time missing when required");
    expect(ready.readiness != RealAdapterReadiness::LivePreflightReady, "M3-A1 does not claim live ready");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
