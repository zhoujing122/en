#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"
#include <cstdio>
#include <functional>
#include <iostream>
#include <string>
namespace {
int failures = 0;
void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } }
void expect_throw(const std::function<void()> &fn, const char *m) { try { fn(); std::cerr << "FAIL: expected throw: " << m << "\n"; failures++; } catch (const std::exception &) {} }
std::string read_file_c(const char *path) { std::string out; char buf[512]; FILE *f = std::fopen(path, "rb"); if (!f) return out; while (true) { const auto n = std::fread(buf, 1, sizeof(buf), f); if (n > 0) out.append(buf, n); if (n < sizeof(buf)) break; } std::fclose(f); return out; }
}
int main() {
    using namespace robot_slamd;
    Config cfg;
    validate_config(cfg);
    expect(!cfg.fake_relocalization_enabled, "fake relocalization disabled");
    expect(!cfg.fake_relocalization_allow_pose_writeback, "writeback false");
    Config invalid = cfg;
    invalid.fake_relocalization_allow_pose_writeback = true; expect_throw([&] { validate_config(invalid); }, "writeback fatal");
    invalid = cfg; invalid.fake_relocalization_run_on_startup = true; expect_throw([&] { validate_config(invalid); }, "startup fatal");
    invalid = cfg; invalid.fake_relocalization_min_valid_ranges = 0; expect_throw([&] { validate_config(invalid); }, "range fatal");
    invalid = cfg; invalid.fake_relocalization_min_confidence = -0.1; expect_throw([&] { validate_config(invalid); }, "confidence low fatal");
    invalid = cfg; invalid.fake_relocalization_min_confidence = 1.1; expect_throw([&] { validate_config(invalid); }, "confidence high fatal");
    invalid = cfg; invalid.fake_relocalization_min_confidence = 0.9; invalid.fake_relocalization_high_confidence_threshold = 0.8; expect_throw([&] { validate_config(invalid); }, "threshold fatal");
    expect(!cfg.fake_map_relocalization_runner_enabled, "runner disabled");
    invalid = cfg; invalid.fake_map_relocalization_runner_run_on_startup = true; expect_throw([&] { validate_config(invalid); }, "runner startup fatal");
    invalid = cfg; invalid.fake_map_relocalization_runner_require_no_pose_writeback = false; expect_throw([&] { validate_config(invalid); }, "runner writeback fatal");
    invalid = cfg; invalid.motion_execution_writer_backend = "software_direction_speed_test_only"; expect_throw([&] { validate_config(invalid); }, "writer fatal");
    invalid = cfg; invalid.motion_execution_hardware_write_enabled = true; expect_throw([&] { validate_config(invalid); }, "hardware fatal");
    write_resolved_config(cfg, "/tmp/m3b6_resolved.yaml");
    const auto resolved = read_file_c("/tmp/m3b6_resolved.yaml");
    expect(resolved.find("fake_relocalization:") != std::string::npos, "resolved relocalization");
    expect(resolved.find("fake_map_relocalization_runner:") != std::string::npos, "resolved runner");
    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b6_metrics.csv", metrics, loc, grid, health, cfg, encoder_stats);
    const auto metrics_text = read_file_c("/tmp/m3b6_metrics.csv");
    expect(metrics_text.find("fake_relocalization_enabled_last") != std::string::npos, "metrics relocalization");
    return failures ? 1 : 0;
}
