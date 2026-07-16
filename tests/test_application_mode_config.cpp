#include "robot_slamd/config/config.hpp"
#include "robot_slamd/runtime/application_mode.hpp"

#include <fstream>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_invalid(const robot_slamd::Config &config,
                    const char *message) {
    try {
        robot_slamd::validate_config(config);
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    } catch (const std::exception &) {
    }
}

void expect_load_rejected(const std::string &path,
                          const std::string &text,
                          const char *message) {
    std::ofstream(path) << text;
    try {
        (void)robot_slamd::load_config(path, "");
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    } catch (const std::exception &) {
    }
}

std::string read_file(const std::string &path) {
    std::ifstream input(path);
    return std::string(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config config;
    expect(config.runtime_sensor_source == "simulation",
           "default source is simulation, not Legacy");
    expect(config.runtime_operation == "mapping",
           "default operation is mapping");
    validate_config(config);

    SensorSource source = SensorSource::Real;
    OperationMode operation = OperationMode::Exploration;
    expect(parse_sensor_source("simulation", source) &&
               source == SensorSource::Simulation,
           "simulation source parses");
    expect(parse_sensor_source("replay", source) &&
               source == SensorSource::Replay,
           "replay source parses");
    expect(parse_sensor_source("real", source) &&
               source == SensorSource::Real,
           "real source parses");
    expect(!parse_sensor_source("legacy", source),
           "Legacy is not a sensor source");
    expect(!parse_sensor_source("sparse_shadow", source),
           "SparseShadow is not a sensor source");
    expect(parse_operation_mode("mapping", operation) &&
               operation == OperationMode::Mapping,
           "mapping operation parses");
    expect(parse_operation_mode("localization", operation) &&
               operation == OperationMode::Localization,
           "localization operation parses");
    expect(parse_operation_mode("exploration", operation) &&
               operation == OperationMode::Exploration,
           "exploration operation parses");
    expect(!parse_operation_mode("unknown", operation),
           "unknown operation does not fall back");

    Config bad = config;
    bad.runtime_sensor_source = "Legacy";
    expect_invalid(bad, "unknown source validation fails closed");
    bad = config;
    bad.runtime_operation = "SparseShadow";
    expect_invalid(bad, "unknown operation validation fails closed");
    bad = config;
    bad.runtime_sensor_source = "replay";
    expect_invalid(bad, "replay requires a path");

    const std::string valid_path =
        "/tmp/robot_slam_application_mode_config.yaml";
    std::ofstream(valid_path)
        << "runtime:\n"
        << "  sensor_source: replay\n"
        << "  operation: localization\n"
        << "  replay_path: /tmp/replay.log\n";
    const auto loaded = load_config(valid_path, "");
    expect(loaded.runtime_sensor_source == "replay",
           "new source key loads");
    expect(loaded.runtime_operation == "localization",
           "new operation key loads");
    expect(loaded.runtime_replay_path == "/tmp/replay.log",
           "replay path loads");

    expect_load_rejected(
        "/tmp/robot_slam_deprecated_runtime_key.yaml",
        "runtime:\n  slam_runtime_mode: legacy\n",
        "runtime.slam_runtime_mode is rejected");
    expect_load_rejected(
        "/tmp/robot_slam_deprecated_mode_key.yaml",
        "slam_runtime:\n  mode: sparse_shadow\n",
        "slam_runtime.mode is rejected");

    const std::string resolved =
        "/tmp/robot_slam_application_mode_resolved.yaml";
    write_resolved_config(loaded, resolved);
    const auto text = read_file(resolved);
    expect(text.find("sensor_source: replay") != std::string::npos,
           "resolved config writes source");
    expect(text.find("operation: localization") != std::string::npos,
           "resolved config writes operation");
    expect(text.find("slam_runtime:") == std::string::npos,
           "resolved config emits no deprecated runtime section");

    return failures ? 1 : 0;
}
