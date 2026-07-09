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
    const auto report = runner.run_once();
    expect(report.ok, "default runner ok");
    expect(report.case_count == 5, "five default cases");
    expect(report.pass_count == 5, "all cases match expectations");
    expect(report.summary == "real_sensor_replay_regression_passed",
           "summary passed");

    bool parse_error_detected = false;
    bool comment_only_rejected = false;
    for (const auto &entry : report.passed) {
        parse_error_detected = parse_error_detected ||
                               entry == "parse_error_log_parse_error_detected";
        comment_only_rejected = comment_only_rejected ||
                                entry == "comment_only_log_matched_expectation";
    }
    expect(parse_error_detected, "parse error log detected");
    expect(comment_only_rejected, "comment only rejected");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
