#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"

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
} // namespace

int main() {
    using namespace robot_slamd;

    RealSensorReplayOptions defaults;
    expect(defaults.time_mode == RealSensorReplayTimeMode::RecordPacketTime,
           "default time mode record packet time");

    RealSensorReplayPort record_packet(RealSensorReplaySampleLog::valid_log());
    auto snapshot = record_packet.latest_snapshot(999.0);
    expect(snapshot.has_tof, "record packet time ignores external now drift");
    expect(std::abs(record_packet.result().last_validation_now_s - 100.0) < 1e-9,
           "record packet validation now recorded");

    RealSensorReplayOptions external_options;
    external_options.time_mode = RealSensorReplayTimeMode::ExternalNow;
    RealSensorReplayPort external(RealSensorReplaySampleLog::valid_log(),
                                  {},
                                  external_options);
    auto failed = external.latest_snapshot(999.0);
    expect(!failed.has_tof, "external now can fail stale drift");
    expect(external.result().rejected_packet_count > 0,
           "external now rejected count");

    RealSensorReplayOptions max_options;
    max_options.time_mode = RealSensorReplayTimeMode::RecordSensorMaxTime;
    RealSensorReplayPort sensor_max(RealSensorReplaySampleLog::valid_log(),
                                    {},
                                    max_options);
    auto max_snapshot = sensor_max.latest_snapshot(0.0);
    expect(max_snapshot.has_tof, "sensor max time passes valid log");
    expect(sensor_max.result().last_validation_now_s >=
               sensor_max.result().last_effective_sensor_time_s,
           "sensor max validation now uses sensor max");
    expect(sensor_max.result().last_packet_time_s > 0.0,
           "last packet time recorded");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
