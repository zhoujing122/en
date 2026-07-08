#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_types.hpp"

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

    expect(to_string(RealSensorReplayRecordKind::Packet) == "Packet",
           "record kind string");
    expect(to_string(RealSensorReplayStatus::Parsed) == "Parsed",
           "status string");
    expect(to_string(RealSensorReplayFault::ParseError) == "ParseError",
           "fault string");
    expect(real_sensor_replay_record_kind_id(RealSensorReplayRecordKind::Packet) == 0,
           "record kind id");
    expect(real_sensor_replay_status_id(RealSensorReplayStatus::Parsed) == 1,
           "status id");
    expect(real_sensor_replay_fault_id(RealSensorReplayFault::ParseError) == 2,
           "fault id");

    RealSensorReplayResult result;
    expect(!result.ok, "default result not ok");
    RealSensorReplayLog log;
    expect(log.records.empty(), "default log empty");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
