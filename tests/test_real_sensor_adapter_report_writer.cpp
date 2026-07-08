#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_adapter_report_writer.hpp"

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

    RealSensorAdapterAcceptanceRunner runner;
    RealSensorAdapterReportWriter writer;
    const auto report = writer.write_text_report(runner.run_once());

    expect(report.find("Real Sensor Adapter Data Contract Report") !=
               std::string::npos,
           "title");
    expect(report.find("overall ok") != std::string::npos, "overall");
    expect(report.find("valid packet result") != std::string::npos,
           "valid packet");
    expect(report.find("request-based ToF/Wheel timing note") !=
               std::string::npos,
           "timing note");
    expect(report.find("passed cases") != std::string::npos, "passed");
    expect(report.find("failed cases") != std::string::npos, "failed");
    expect(report.find("warnings") != std::string::npos, "warnings");
    expect(report.find("summary") != std::string::npos, "summary");
    expect(report.find("This report does not prove real ToF driver readiness.") !=
               std::string::npos,
           "tof disclaimer");
    expect(report.find("This report does not prove real IMU driver readiness.") !=
               std::string::npos,
           "imu disclaimer");
    expect(report.find("This report does not prove real Wheel driver readiness.") !=
               std::string::npos,
           "wheel disclaimer");
    expect(report.find("This report only validates the real sensor adapter data contract skeleton.") !=
               std::string::npos,
           "skeleton disclaimer");
    expect(report.find("ToF and Wheel timestamps are request-window estimates, not hardware capture timestamps.") !=
               std::string::npos,
           "timestamp disclaimer");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
