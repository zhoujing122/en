#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_types.hpp"

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

    expect(to_string(RealSensorDataKind::Tof) == "Tof", "tof string");
    expect(to_string(RealSensorDataContractStatus::Accepted) == "Accepted",
           "accepted string");
    expect(to_string(RealSensorDataFault::MissingTof) == "MissingTof",
           "missing tof string");
    expect(to_string(RealSensorDataFault::InvalidRequestTiming) ==
               "InvalidRequestTiming",
           "invalid request timing string");
    expect(to_string(RealSensorDataFault::RequestLatencyTooHigh) ==
               "RequestLatencyTooHigh",
           "latency high string");
    expect(real_sensor_data_kind_id(RealSensorDataKind::Tof) == 0,
           "kind id stable");
    expect(real_sensor_data_contract_status_id(
               RealSensorDataContractStatus::Accepted) == 0,
           "status id stable");
    expect(real_sensor_data_fault_id(RealSensorDataFault::MissingTof) == 1,
           "fault id stable");

    const auto timing = make_request_timing(10.0, 10.2);
    expect(std::fabs(timing.estimated_sample_time_s - 10.1) < 1e-9,
           "estimated midpoint");
    expect(std::fabs(timing.request_latency_s - 0.2) < 1e-9,
           "request latency");

    RealSensorRawPacket packet;
    expect(!packet.has_tof, "default packet no tof");
    RealSensorDataContractResult contract;
    expect(!contract.ok, "default contract not ok");
    RealSensorSnapshotBuildResult build;
    expect(!build.ok, "default build not ok");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
