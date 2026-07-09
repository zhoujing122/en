#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"

#include <string>
#include <vector>
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

    RealSensorReplayLog empty_log;
    RealSensorReplayPort empty(empty_log);
    expect(!empty.ready(), "empty log not ready");
    expect(empty.result().fault == RealSensorReplayFault::EmptyLog,
           "empty log fault");

    RealSensorReplayLogCodec codec;
    RealSensorReplayPort comment_only(
        codec.parse_lines({"# only comment", "# no packets"}));
    expect(!comment_only.ready(), "comment only not ready");
    expect(comment_only.result().fault == RealSensorReplayFault::NoPacketRecords,
           "comment only no packet fault");

    RealSensorReplayPort invalid_record(codec.parse_lines({"kind=packet,packet_time=abc"}));
    expect(!invalid_record.ready(), "invalid record not ready");
    auto invalid_snapshot = invalid_record.latest_snapshot(100.0);
    expect(!invalid_snapshot.has_tof && !invalid_snapshot.has_imu &&
               !invalid_snapshot.has_wheel,
           "invalid record returns empty snapshot");

    RealSensorReplayPort valid(RealSensorReplaySampleLog::valid_log());
    expect(valid.ready(), "valid log ready");

    std::vector<std::string> mixed_lines;
    mixed_lines.push_back(RealSensorReplaySampleLog::valid_log_lines()[1]);
    mixed_lines.push_back(RealSensorReplaySampleLog::invalid_latency_log_lines()[1]);
    RealSensorReplayPort mixed(codec.parse_lines(mixed_lines));
    auto first = mixed.latest_snapshot(100.0);
    expect(first.has_tof, "first mixed snapshot valid");
    auto second = mixed.latest_snapshot(100.0);
    expect(!second.has_tof && !second.has_imu && !second.has_wheel,
           "invalid packet returns empty not last valid snapshot");

    const auto status = mixed.status();
    expect(status.find("index=") != std::string::npos,
           "status has index");
    expect(status.find("status=") != std::string::npos,
           "status has status");
    expect(status.find("fault=") != std::string::npos,
           "status has fault");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
