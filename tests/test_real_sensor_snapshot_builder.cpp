#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_sample_data.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_snapshot_builder.hpp"

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

    RealSensorSnapshotBuilder builder;
    const auto packet = RealSensorSampleData::valid_packet(100.0);
    const auto result = builder.build_snapshot(packet, 100.0);
    expect(result.ok, "valid build ok");
    expect(result.snapshot.has_tof, "snapshot has tof");
    expect(result.snapshot.has_imu, "snapshot has imu");
    expect(result.snapshot.has_wheel, "snapshot has wheel");
    expect(result.snapshot.tof.timestamp_s ==
               packet.tof.timing.estimated_sample_time_s,
           "tof timestamp uses estimate");
    expect(result.snapshot.wheel.timestamp_s ==
               packet.wheel.timing.estimated_sample_time_s,
           "wheel timestamp uses estimate");
    expect(result.message == "real_sensor_snapshot_build_ok",
           "build ok message");

    expect(!builder.build_snapshot(
                RealSensorSampleData::missing_tof_packet(100.0),
                100.0)
                .ok,
           "missing tof fails");
    expect(!builder.build_snapshot(
                RealSensorSampleData::empty_tof_ranges_packet(100.0),
                100.0)
                .ok,
           "empty ranges fails");
    expect(!builder.build_snapshot(
                RealSensorSampleData::sync_too_large_packet(100.0),
                100.0)
                .ok,
           "sync too large fails");
    expect(!builder.build_snapshot(
                RealSensorSampleData::request_latency_too_high_packet(100.0),
                100.0)
                .ok,
           "latency too high fails");
    expect(builder.build_snapshot(packet, 100.0).snapshot.tof.ranges_m ==
               result.snapshot.tof.ranges_m,
           "deterministic");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
