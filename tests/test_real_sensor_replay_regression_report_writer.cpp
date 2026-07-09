#include "robot_slamd/autonomy/real_adapters/sensor_replay/regression/real_sensor_replay_regression_report_writer.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/regression/real_sensor_replay_regression_runner.hpp"

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

    RealSensorReplayRegressionRunner runner;
    RealSensorReplayRegressionReportWriter writer;
    const auto text = writer.write_text_report(runner.run_once());

    expect(text.find("Real Sensor Replay Regression Report") != std::string::npos,
           "title");
    expect(text.find("overall ok") != std::string::npos, "overall ok");
    expect(text.find("status") != std::string::npos, "status");
    expect(text.find("block reason") != std::string::npos, "block reason");
    expect(text.find("case count") != std::string::npos, "case count");
    expect(text.find("pass count") != std::string::npos, "pass count");
    expect(text.find("fail count") != std::string::npos, "fail count");
    expect(text.find("request-window estimates") != std::string::npos,
           "request-window disclaimer");
    expect(text.find("does not prove real hardware readiness") != std::string::npos,
           "hardware disclaimer");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
