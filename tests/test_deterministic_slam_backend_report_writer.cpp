#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_report_writer.hpp"

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
    ReplayToSlamBackendRegressionRunner runner;
    DeterministicSlamBackendReportWriter writer;
    const auto text = writer.write_text_report(runner.run_once());
    expect(text.find("Deterministic SLAM Backend Skeleton Report") != std::string::npos, "title");
    expect(text.find("coverage_ratio") != std::string::npos, "coverage");
    expect(text.find("yaw_coverage_ratio") != std::string::npos, "yaw coverage");
    expect(text.find("accepted update count") != std::string::npos, "accepted count");
    expect(text.find("does not prove real SLAM readiness") != std::string::npos, "not real slam");
    expect(text.find("No real hardware") != std::string::npos, "no hardware");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
