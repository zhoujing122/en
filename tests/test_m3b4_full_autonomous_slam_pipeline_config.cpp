#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"

#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) { if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; } }
void expect_throw(const std::function<void()> &fn, const char *message) { try { fn(); std::cerr << "FAIL: expected throw: " << message << "\n"; failures++; } catch (const std::exception &) {} }
std::string read_file(const std::string &path) { std::ifstream in(path); std::ostringstream ss; ss << in.rdbuf(); return ss.str(); }
}
int main() {
    using namespace robot_slamd;
    Config cfg;
    validate_config(cfg);
    expect(!cfg.full_autonomous_slam_fake_pipeline_enabled, "default disabled");
    expect(!cfg.full_autonomous_slam_fake_pipeline_run_on_startup, "startup false");
    Config invalid = cfg;
    invalid.full_autonomous_slam_fake_pipeline_run_on_startup = true;
    expect_throw([&] { validate_config(invalid); }, "startup fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_max_steps = 0; expect_throw([&] { validate_config(invalid); }, "max steps fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_max_active_scan_commands = -1; expect_throw([&] { validate_config(invalid); }, "active scan fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_min_backend_accepted_updates = 0; expect_throw([&] { validate_config(invalid); }, "backend count fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_require_shadow_motion_only = false; expect_throw([&] { validate_config(invalid); }, "shadow only fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_require_no_forward_backward = false; expect_throw([&] { validate_config(invalid); }, "forward backward fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_motion_settle_s = -0.1; expect_throw([&] { validate_config(invalid); }, "settle negative fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_motion_settle_s = 11.0; expect_throw([&] { validate_config(invalid); }, "settle high fatal");
    invalid = cfg; invalid.motion_execution_writer_backend = "software_direction_speed_test_only"; expect_throw([&] { validate_config(invalid); }, "writer backend fatal");
    invalid = cfg; invalid.motion_execution_software_motion_production_interface_enabled = true; expect_throw([&] { validate_config(invalid); }, "production fatal");
    invalid = cfg; invalid.motion_execution_allow_writer_dispatch = true; expect_throw([&] { validate_config(invalid); }, "dispatch fatal");
    invalid = cfg; invalid.motion_execution_hardware_write_enabled = true; expect_throw([&] { validate_config(invalid); }, "hardware fatal");
    write_resolved_config(cfg, "/tmp/m3b4_resolved.yaml");
    const auto resolved = read_file("/tmp/m3b4_resolved.yaml");
    expect(resolved.find("full_autonomous_slam_fake_pipeline:") != std::string::npos, "resolved block");
    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b4_metrics.csv", metrics, loc, grid, health, cfg, encoder_stats);
    const auto metrics_text = read_file("/tmp/m3b4_metrics.csv");
    expect(metrics_text.find("full_autonomous_slam_fake_pipeline_enabled_last") != std::string::npos, "metrics block");
    return failures ? 1 : 0;
}
