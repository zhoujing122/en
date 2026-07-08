#include "robot_slamd/autonomy/real_adapters/real_tof_imu_wheel_sensor_port_stub.hpp"

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

    RealTofImuWheelSensorPortStub port;
    expect(!port.ready(), "default not ready");

    const auto first = port.latest_snapshot(100.0);
    const auto second = port.latest_snapshot(101.0);
    expect(!first.has_tof, "no tof");
    expect(!first.has_imu, "no imu");
    expect(!first.has_wheel, "no wheel");
    expect(!second.has_tof && !second.has_imu && !second.has_wheel,
           "deterministic empty snapshot");
    expect(port.status().find("waiting_for_real_sensor_adapter_implementation") !=
               std::string::npos,
           "status names implementation wait");

    const auto status = port.stub_status();
    expect(status.kind == RealAdapterStubKind::Sensor, "sensor kind");
    expect(status.state == RealAdapterStubState::WaitingForImplementation,
           "waiting state");
    expect(!status.ready, "status not ready");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
