#include "robot_slamd/config/config.hpp"
#include "robot_slamd/runtime/slam_runtime_mode.hpp"

#include <fstream>
#include <iostream>
#include <string>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; }
}
void expect_throw(const char *message, const robot_slamd::Config &cfg) {
    try {
        robot_slamd::validate_config(cfg);
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    } catch (const std::exception &) {
    }
}
std::string read_file(const std::string &path) {
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
}

int main() {
    using namespace robot_slamd;
    Config cfg;
    expect(cfg.slam_runtime_mode == "legacy", "default runtime mode legacy");
    SlamRuntimeMode mode = SlamRuntimeMode::SparseShadow;
    expect(parse_slam_runtime_mode(cfg.slam_runtime_mode, mode), "default mode parses");
    expect(mode == SlamRuntimeMode::Legacy, "default resolves legacy");
    validate_config(cfg);

    cfg.slam_runtime_mode = "legacy";
    validate_config(cfg);
    cfg.slam_runtime_mode = "sparse_shadow";
    cfg.sparse_shadow_sensor_source = "deterministic_simulation";
    validate_config(cfg);
    expect(resolved_slam_runtime_mode(cfg.slam_runtime_mode) == SlamRuntimeMode::SparseShadow,
           "sparse shadow resolves");

    Config bad = cfg;
    bad.slam_runtime_mode = "";
    expect_throw("empty runtime mode rejected", bad);
    bad = cfg;
    bad.slam_runtime_mode = "SparseShadow";
    expect_throw("runtime mode is case sensitive", bad);
    bad = cfg;
    bad.slam_runtime_mode = "unknown";
    expect_throw("unknown runtime mode rejected", bad);
    bad = cfg;
    bad.sparse_shadow_sensor_source = "hardware";
    expect_throw("hardware sparse shadow source fail closed", bad);
    bad = cfg;
    bad.sparse_shadow_sensor_source = "replay";
    expect_throw("replay source not wired in D1.1", bad);

    const std::string resolved = "/tmp/m3d11_runtime_mode_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("slam_runtime:") != std::string::npos, "resolved has runtime section");
    expect(text.find("mode: sparse_shadow") != std::string::npos, "resolved writes sparse mode");
    expect(text.find("sparse_shadow_sensor_source: deterministic_simulation") != std::string::npos,
           "resolved writes sparse source");
    return failures ? 1 : 0;
}
