#include "robot_slamd/app/app.hpp"
#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/app/robot_slam_application_adapters.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sample_log.hpp"

#include <fstream>
#include <iostream>
#include <vector>

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

    const std::string path = "/tmp/robot_slam_application_replay.log";
    auto first = MultiTofSyncSampleData::valid_synced_packet();
    auto second = MultiTofSyncSampleData::valid_synced_packet();
    auto third = MultiTofSyncSampleData::valid_synced_packet();
    first.imu.yaw_rate_rad_s = 0.0;
    first.wheel.linear_velocity_mps = 0.0;
    first.wheel.angular_velocity_rad_s = 0.0;
    MultiTofSyncSampleData::set_tof_time(second.front, 100.200);
    MultiTofSyncSampleData::set_tof_time(second.left, 100.210);
    MultiTofSyncSampleData::set_tof_time(second.right, 100.195);
    second.imu.timestamp_s = 100.210;
    second.imu.yaw_rate_rad_s = 0.0;
    second.wheel.timing =
        MultiTofSyncSampleData::request_timing_for_estimate(100.210);
    second.wheel.linear_velocity_mps = 0.0;
    second.wheel.angular_velocity_rad_s = 0.0;
    second.packet_timestamp_s = 100.210;
    MultiTofSyncSampleData::set_tof_time(third.front, 100.300);
    MultiTofSyncSampleData::set_tof_time(third.left, 100.310);
    MultiTofSyncSampleData::set_tof_time(third.right, 100.295);
    third.imu.timestamp_s = 100.310;
    third.imu.yaw_rate_rad_s = 0.0;
    third.wheel.timing =
        MultiTofSyncSampleData::request_timing_for_estimate(100.310);
    third.wheel.linear_velocity_mps = 0.0;
    third.wheel.angular_velocity_rad_s = 0.0;
    third.packet_timestamp_s = 100.310;
    const std::vector<std::string> lines = {
        "# application replay fixture",
        MultiTofReplayLogCodec().encode_record(first),
        MultiTofReplayLogCodec().encode_record(second),
        MultiTofReplayLogCodec().encode_record(third)};
    std::ofstream(path) << lines[0] << "\n" << lines[1] << "\n"
                        << lines[2] << "\n" << lines[3] << "\n";

    Config config;
    config.runtime_sensor_source = "replay";
    config.runtime_operation = "mapping";
    config.runtime_replay_path = path;
    config.multi_tof_replay_time_mode = "record_packet_time";

    const auto adapter = build_replay_sensor_adapter(config);
    expect(adapter.ok, "configured replay adapter builds");
    expect(adapter.sensor && adapter.sensor->ready(),
           "configured replay adapter is ready");
    if (!adapter.ok || !adapter.sensor) {
        return 1;
    }

    RobotSlamApplication application(
        config, SensorSource::Replay, OperationMode::Mapping, adapter.sensor,
        nullptr);
    expect(application.initialize().ok, "replay Application initializes");
    const double first_time = adapter.sensor->first_packet_time_s();
    expect(application.step(first_time).core_step_executed,
           "first replay snapshot reaches Core");
    const auto second_result = application.step(first_time + 0.1);
    expect(second_result.core_step_executed,
           "second replay snapshot reaches Core");
    const auto third_result = application.step(first_time + 0.2);
    expect(third_result.core_step_executed,
           "third replay snapshot reaches Core");
    expect(application.report().canonical_snapshot_count == 3,
           "Application reports all canonical replay snapshots");
    expect(adapter.sensor->successful_snapshot_count() == 3,
           "adapter reports all successful snapshots");
    expect(application.core().report().current_map_revision == 1,
           "replay mapping uses the canonical map owner");

    const std::string config_path =
        "/tmp/robot_slam_application_replay_main.yaml";
    const std::string output_dir =
        "/tmp/robot_slam_application_replay_main";
    std::ofstream(config_path)
        << "runtime:\n"
        << "  sensor_source: replay\n"
        << "  operation: mapping\n"
        << "  replay_path: " << path << "\n"
        << "logging:\n"
        << "  output_dir: " << output_dir << "\n";
    std::vector<std::string> arguments = {
        "robot_slamd", "--config", config_path, "--output-dir", output_dir};
    std::vector<char *> argv;
    for (auto &argument : arguments) {
        argv.push_back(argument.data());
    }
    expect(real_main(static_cast<int>(argv.size()), argv.data()) == 0,
           "formal replay entry reaches one Application");

    return failures ? 1 : 0;
}
