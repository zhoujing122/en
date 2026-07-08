#include "robot_slamd/autonomy/real_adapters/real_adapter_factory.hpp"

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

    RealAdapterFactory disabled;
    expect(!disabled.create_sensor_port(), "disabled sensor null");
    expect(!disabled.create_motion_port(), "disabled motion null");
    expect(!disabled.create_slam_backend(), "disabled backend null");

    RealAdapterFactoryOptions options;
    options.enabled = true;
    RealAdapterFactory enabled(options);
    expect(static_cast<bool>(enabled.create_sensor_port()), "sensor stub created");
    expect(static_cast<bool>(enabled.create_motion_port()), "motion stub created");
    expect(static_cast<bool>(enabled.create_slam_backend()), "backend stub created");

    options.allow_real_hardware_adapters = true;
    RealAdapterFactory still_stub_only(options);
    const auto sensor = still_stub_only.create_sensor_port();
    const auto motion = still_stub_only.create_motion_port();
    const auto backend = still_stub_only.create_slam_backend();
    expect(sensor && sensor->status().find("Stub") != std::string::npos,
           "real hardware option still sensor stub");
    expect(motion && motion->status().find("Stub") != std::string::npos,
           "real hardware option still motion stub");
    expect(backend && backend->status().find("Stub") != std::string::npos,
           "real hardware option still backend stub");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
