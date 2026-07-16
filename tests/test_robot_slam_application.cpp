#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/autonomy/adapters/replay_robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/real_adapters/real_tof_imu_wheel_sensor_port_stub.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::ScalarTofSnapshotFrame frame(
    std::uint16_t distance_mm,
    double yaw_rad,
    const char *frame_id,
    double timestamp_s) {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_mm = distance_mm;
    out.distance_m = static_cast<double>(distance_mm) / 1000.0;
    out.confidence = 100;
    out.echo_tag_u48 = 0x450000 + distance_mm;
    out.effective_timestamp_s = timestamp_s;
    out.protocol_valid = true;
    out.usable_for_mapping = true;
    out.full_fov_rad = 1.6 * robot_slamd::kPi / 180.0;
    out.mount_yaw_rad = yaw_rad;
    out.frame_id = frame_id;
    out.source = "application_replay_fixture";
    return out;
}

robot_slamd::RobotSlamSensorSnapshot snapshot(double timestamp_s) {
    robot_slamd::RobotSlamSensorSnapshot out;
    out.has_wheel = true;
    out.wheel.timestamp_s = timestamp_s;
    out.wheel.valid = true;
    out.has_imu = true;
    out.imu.timestamp_s = timestamp_s;
    out.imu.az_mps2 = 9.8;
    out.has_multi_tof = true;
    out.multi_tof.has_front = true;
    out.multi_tof.has_left = true;
    out.multi_tof.has_right = true;
    out.multi_tof.valid_tof_count = 3;
    out.multi_tof.front = frame(900, 0.0, "front", timestamp_s);
    out.multi_tof.left =
        frame(1100, robot_slamd::kPi / 2.0, "left", timestamp_s);
    out.multi_tof.right =
        frame(1100, -robot_slamd::kPi / 2.0, "right", timestamp_s);
    out.multi_tof.synchronized_time_s = timestamp_s;
    out.multi_tof.source = "application_replay_fixture";
    return out;
}

std::shared_ptr<robot_slamd::ReplayRobotSlamSensorPort> replay() {
    return std::make_shared<robot_slamd::ReplayRobotSlamSensorPort>(
        std::vector<robot_slamd::RobotSlamSensorSnapshot>{
            snapshot(0.1), snapshot(0.2)});
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config mapping_config;
    mapping_config.runtime_sensor_source = "replay";
    mapping_config.runtime_operation = "mapping";
    mapping_config.runtime_replay_path = "in_memory_fixture";
    RobotSlamApplication mapping(
        mapping_config, SensorSource::Replay, OperationMode::Mapping,
        replay(), nullptr);
    expect(mapping.initialize().ok, "mapping Application initializes");
    expect(mapping.step(0.1).ok, "mapping startup snapshot reaches Core");
    expect(mapping.step(0.2).ok, "mapping replay snapshot reaches Core");
    expect(mapping.report().canonical_snapshot_count == 2,
           "Application counts canonical snapshots");
    expect(mapping.report().core_step_count == 2,
           "Application has one Core step path");
    expect(mapping.core().report().current_map_revision == 1,
           "mapping operation enables canonical map write");

    Config localization_config = mapping_config;
    localization_config.runtime_operation = "localization";
    RobotSlamApplication localization(
        localization_config, SensorSource::Replay,
        OperationMode::Localization, replay(), nullptr);
    expect(localization.initialize().ok,
           "localization Application initializes");
    expect(localization.step(0.1).ok,
           "localization startup snapshot reaches Core");
    expect(localization.step(0.2).ok,
           "localization snapshot is accepted");
    expect(localization.core().report().current_map_revision == 0,
           "localization operation blocks map revision writes");
    expect(localization.core().report().sparse_map_cell_count == 0,
           "localization operation blocks occupancy writes");

    Config real_config;
    real_config.runtime_sensor_source = "real";
    auto real_sensor =
        std::make_shared<RealTofImuWheelSensorPortStub>();
    RobotSlamApplication real(
        real_config, SensorSource::Real, OperationMode::Mapping,
        real_sensor, nullptr);
    const auto real_init = real.initialize();
    expect(!real_init.ok, "unavailable real source fails closed");
    expect(real.report().real_source_rejected,
           "real rejection is explicit");
    expect(real_init.reason ==
               "real_sensor_source_unavailable_fail_closed",
           "real rejection reason is stable");
    expect(real.core().report().sensor_snapshot_count == 0,
           "real rejection performs no Core sensor step");

    return failures ? 1 : 0;
}
