#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_report_writer.hpp"

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

    RealSensorReplayAcceptanceRunner runner;
    RealSensorReplayReportWriter writer;
    const auto text = writer.write_text_report(runner.run_once());
    expect(text.find("Real Sensor Replay Report") != std::string::npos,
           "title");
    expect(text.find("overall ok") != std::string::npos, "overall ok");
    expect(text.find("valid log result") != std::string::npos, "valid log");
    expect(text.find("invalid latency result") != std::string::npos,
           "invalid latency");
    expect(text.find("sync too large result") != std::string::npos,
           "sync too large");
    expect(text.find("passed cases") != std::string::npos, "passed cases");
    expect(text.find("request-window estimates") != std::string::npos,
           "request-window disclaimer");
    expect(text.find("does not read real ToF/IMU/Wheel devices") !=
               std::string::npos,
           "no hardware disclaimer");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
