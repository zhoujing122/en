#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"

#include <cstdio>
#include <functional>
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
void expect_throw(const std::function<void()> &fn, const char *message) {
    try {
        fn();
        std::cerr << "FAIL: expected throw: " << message << "\n";
        failures++;
    } catch (const std::exception &) {}
}
std::string read_file_c(const char *path) {
    std::string out;
    char buf[512];
    FILE *f = std::fopen(path, "rb");
    if (!f) {
        return out;
    }
    while (true) {
        const auto n = std::fread(buf, 1, sizeof(buf), f);
        if (n > 0) {
            out.append(buf, n);
        }
        if (n < sizeof(buf)) {
            break;
        }
    }
    std::fclose(f);
    return out;
}
}

int main() {
    using namespace robot_slamd;
    Config cfg;
    validate_config(cfg);
    expect(!cfg.multi_tof_replay_enabled, "replay disabled default");
    expect(!cfg.multi_tof_replay_run_acceptance_on_startup,
           "startup default false");
    Config invalid = cfg;
    invalid.multi_tof_replay_run_acceptance_on_startup = true;
    expect_throw([&] { validate_config(invalid); }, "startup fatal");
    invalid = cfg;
    invalid.multi_tof_replay_reject_invalid_records = false;
    expect_throw([&] { validate_config(invalid); }, "reject invalid fatal");
    invalid = cfg;
    invalid.multi_tof_replay_require_snapshot_build = false;
    expect_throw([&] { validate_config(invalid); }, "snapshot build fatal");
    invalid = cfg;
    invalid.multi_tof_replay_time_mode = "bad";
    expect_throw([&] { validate_config(invalid); }, "unknown time mode fatal");
    invalid = cfg;
    invalid.motion_execution_writer_backend = "software_direction_speed_test_only";
    expect_throw([&] { validate_config(invalid); }, "old fail closed fatal");
    write_resolved_config(cfg, "/tmp/m3c3_resolved.yaml");
    const auto resolved = read_file_c("/tmp/m3c3_resolved.yaml");
    expect(resolved.find("multi_tof_replay:") != std::string::npos,
           "resolved block");
    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3c3_metrics.csv", metrics, loc, grid, health, cfg, encoder_stats);
    const auto metrics_text = read_file_c("/tmp/m3c3_metrics.csv");
    expect(metrics_text.find("multi_tof_replay_enabled_last") != std::string::npos,
           "metrics field");
    return failures ? 1 : 0;
}
