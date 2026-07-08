#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"

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

    RealSensorReplayPort port(RealSensorReplaySampleLog::valid_log());
    expect(port.ready(), "valid log ready");
    auto first = port.latest_snapshot(100.0);
    expect(first.has_tof && first.has_imu && first.has_wheel,
           "first snapshot has sensors");
    expect(first.tof.timestamp_s == 100.0, "tof uses estimate");
    expect(first.wheel.timestamp_s == 100.0, "wheel uses estimate");
    expect(port.current_index() >= 2, "index advances past comment and packet");
    port.latest_snapshot(100.0);
    port.latest_snapshot(100.0);
    port.latest_snapshot(100.0);
    expect(port.result().status == RealSensorReplayStatus::Completed,
           "loop false completes");

    RealSensorReplayPort bad(RealSensorReplaySampleLog::invalid_latency_log());
    bad.latest_snapshot(100.0);
    expect(bad.result().rejected_packet_count > 0, "bad latency rejected");

    RealSensorReplayPort empty(RealSensorReplayLog{});
    expect(!empty.ready(), "empty log not ready");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
