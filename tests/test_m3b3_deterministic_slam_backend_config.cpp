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
    expect(!cfg.deterministic_slam_backend_enabled, "backend disabled default");
    expect(!cfg.deterministic_slam_backend_ready, "ready false default");
    expect(!cfg.deterministic_slam_backend_allow_save_map, "save map false default");
    expect(!cfg.replay_to_slam_backend_regression_enabled, "regression disabled default");
    Config invalid = cfg;
    invalid.deterministic_slam_backend_run_regression_on_startup = true;
    expect_throw([&] { validate_config(invalid); }, "backend startup fatal");
    invalid = cfg; invalid.deterministic_slam_backend_min_valid_ranges = 0; expect_throw([&] { validate_config(invalid); }, "min ranges fatal");
    invalid = cfg; invalid.deterministic_slam_backend_min_valid_scan_count_for_good = 0; expect_throw([&] { validate_config(invalid); }, "min scans fatal");
    invalid = cfg; invalid.deterministic_slam_backend_min_valid_range_ratio = 1.5; expect_throw([&] { validate_config(invalid); }, "ratio fatal");
    invalid = cfg; invalid.deterministic_slam_backend_max_range_m = invalid.deterministic_slam_backend_min_range_m; expect_throw([&] { validate_config(invalid); }, "range fatal");
    invalid = cfg; invalid.replay_to_slam_backend_regression_run_on_startup = true; expect_throw([&] { validate_config(invalid); }, "regression startup fatal");
    invalid = cfg; invalid.motion_execution_writer_backend = "software_direction_speed_test_only"; expect_throw([&] { validate_config(invalid); }, "writer backend fatal");
    invalid = cfg; invalid.motion_execution_software_motion_production_interface_enabled = true; expect_throw([&] { validate_config(invalid); }, "production fatal");
    invalid = cfg; invalid.motion_execution_allow_writer_dispatch = true; expect_throw([&] { validate_config(invalid); }, "dispatch fatal");
    invalid = cfg; invalid.motion_execution_hardware_write_enabled = true; expect_throw([&] { validate_config(invalid); }, "hardware write fatal");
    const std::string resolved = "/tmp/m3b3_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("deterministic_slam_backend:") != std::string::npos, "resolved backend");
    expect(text.find("replay_to_slam_backend_regression:") != std::string::npos, "resolved regression");
    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b3_metrics.csv", metrics, loc, grid, health, cfg, encoder_stats);
    const auto metrics_text = read_file("/tmp/m3b3_metrics.csv");
    expect(metrics_text.find("deterministic_slam_backend_enabled_last") != std::string::npos, "metrics backend");
    expect(metrics_text.find("replay_to_slam_backend_regression_enabled_last") != std::string::npos, "metrics regression");
    if (failures) { std::cerr << failures << " failures\n"; return 1; }
    return 0;
}
