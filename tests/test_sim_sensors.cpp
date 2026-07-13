#include "robot_slamd/simulation/ports/sim_sensor_port.hpp"

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
    auto clock = std::make_shared<SimClock>(100.0);
    auto world = std::make_shared<FakeWorld2D>();
    world->add_axis_aligned_room(-2.0, -2.0, 2.0, 2.0);
    auto plant = std::make_shared<SimRobotPlant>();
    RobotPose2D pose;
    pose.x_m = -0.5;
    pose.y_m = 0.25;
    pose.yaw_rad = 0.0;
    plant->reset(pose, clock->now_s());

    SimSensorPortConfig config;
    config.sync_options.enabled = true;
    config.build_options.enabled = true;
    SimSensorPort port(clock, world, plant, config);
    expect(port.ready(), "sim sensor ready");
    auto snapshot = port.latest_snapshot(clock->now_s());
    expect(snapshot.has_multi_tof, "snapshot has multi tof");
    expect(snapshot.has_tof, "legacy front projection present");
    expect(snapshot.has_imu, "imu present");
    expect(snapshot.has_wheel, "wheel present");
    expect(snapshot.multi_tof.front.distance_mm > 0, "front distance present");
    expect(snapshot.multi_tof.front.confidence == 100, "front confidence");
    expect(snapshot.multi_tof.front.echo_tag_u48 != 0, "echo tag present");
    expect(std::fabs(snapshot.multi_tof.front.full_fov_rad - sim_degrees_to_radians(1.6)) < 1e-12,
           "full fov 1.6 deg");
    expect(snapshot.tof.ranges_m.size() == 1, "legacy single range");
    expect(std::fabs(snapshot.multi_tof.front.effective_timestamp_s - 99.998) < 1e-9,
           "tof midpoint preserved");
    expect(std::fabs(snapshot.wheel.timestamp_s - 99.999) < 1e-9,
           "wheel pair midpoint preserved");

    auto stale = port.latest_snapshot(clock->now_s());
    expect(!snapshot_has_any_payload(stale), "same time does not create fresh snapshot");
    expect(port.rejected_count() == 1, "stale rejected count");
    expect(clock->advance(0.02), "advance clock");
    plant->set_target_velocity(0.0, 0.8);
    plant->step(0.02, clock->now_s());
    auto moved = port.latest_snapshot(clock->now_s());
    expect(moved.has_multi_tof, "legal frame recovers after stale");
    expect(moved.imu.yaw_rate_rad_s > 0.0, "imu yaw rate sign");
    expect(port.success_count() == 2, "success count");

    SimThreeScalarTofConfig tof_config;
    tof_config.injected_pairwise_skew_s = 1.0;
    SimSensorPort skewed(clock, world, plant, config, SimWheelEncoder{}, SimImu{},
                         SimThreeScalarTof{tof_config});
    expect(clock->advance(0.02), "advance skew clock");
    auto rejected = skewed.latest_snapshot(clock->now_s());
    expect(!snapshot_has_any_payload(rejected), "sync failure returns empty snapshot");
    expect(skewed.rejected_count() == 1, "sync failure rejected");
    return failures ? 1 : 0;
}
