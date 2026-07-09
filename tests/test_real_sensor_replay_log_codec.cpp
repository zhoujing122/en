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

    RealSensorReplayLogCodec codec;
    auto log = codec.parse_lines(RealSensorReplaySampleLog::valid_log_lines());
    int packets = 0;
    for (const auto &record : log.records) {
        if (record.kind == RealSensorReplayRecordKind::Packet) {
            packets++;
        }
    }
    expect(packets >= 3, "valid log parses packets");
    auto roundtrip = codec.parse_lines(codec.write_lines(log));
    expect(codec.write_lines(roundtrip) == codec.write_lines(log),
           "write parse deterministic");
    expect(log.records.front().kind == RealSensorReplayRecordKind::Comment,
           "comment accepted");
    expect(log.records[1].packet.tof.timing.request_latency_s > 0.0,
           "tof request fields kept");
    expect(log.records[1].packet.wheel.timing.request_latency_s > 0.0,
           "wheel request fields kept");
    expect(std::isnan(log.records[1].packet.tof.ranges_m[2]),
           "tof ranges supports nan");
    auto invalid = codec.parse_lines({"not_a_packet"});
    expect(invalid.records.front().kind == RealSensorReplayRecordKind::InvalidRecord,
           "invalid line invalid record");
    expect(invalid.records.front().message.find("parse_error") != std::string::npos,
           "invalid line parse error");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
