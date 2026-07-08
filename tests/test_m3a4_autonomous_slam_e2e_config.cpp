#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"

#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
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
    } catch (const std::exception &) {
    }
}

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}
} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg;
    validate_config(cfg);
    expect(!cfg.autonomous_slam_e2e_prelive_enabled, "e2e default disabled");
    expect(cfg.autonomous_slam_e2e_prelive_scenario_kind ==
               "active_scan_then_map_good",
           "default scenario kind");

    Config invalid = cfg;
    invalid.autonomous_slam_e2e_prelive_scenario_kind = "unknown";
    expect_throw([&] { validate_config(invalid); }, "unknown scenario fatal");
    invalid = cfg;
    invalid.autonomous_slam_e2e_prelive_max_iterations = 201;
    expect_throw([&] { validate_config(invalid); }, "max iterations fatal");
    invalid = cfg;
    invalid.autonomous_slam_e2e_prelive_step_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "step <=0 fatal");
    invalid = cfg;
    invalid.autonomous_slam_e2e_prelive_step_s = 1.1;
    expect_throw([&] { validate_config(invalid); }, "step >1 fatal");
    invalid = cfg;
    invalid.autonomous_slam_e2e_prelive_start_time_s =
        std::numeric_limits<double>::quiet_NaN();
    expect_throw([&] { validate_config(invalid); }, "start nan fatal");
    invalid = cfg;
    invalid.autonomous_slam_e2e_prelive_enabled = true;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "enabled with hardware write fatal");

    invalid = cfg;
    invalid.motion_execution_writer_backend = "software_direction_speed_test_only";
    expect_throw([&] { validate_config(invalid); }, "writer backend remains fatal");
    invalid = cfg;
    invalid.motion_execution_software_motion_production_interface_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "production interface remains fatal");
    invalid = cfg;
    invalid.motion_execution_allow_writer_dispatch = true;
    expect_throw([&] { validate_config(invalid); }, "writer dispatch remains fatal");
    invalid = cfg;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "hardware write remains fatal");
    invalid = cfg;
    invalid.motion_execution_software_motion_production_transport_backend = "loopback_shadow";
    invalid.motion_execution_software_motion_loopback_shadow_mode = false;
    expect_throw([&] { validate_config(invalid); }, "loopback non-shadow remains fatal");

    const std::string resolved = "/tmp/m3a4_e2e_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("autonomous_slam_e2e_prelive:") != std::string::npos,
           "resolved has e2e block");
    expect(text.find("scenario_kind: active_scan_then_map_good") != std::string::npos,
           "resolved has scenario kind");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3a4_e2e_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const auto metrics_text = read_file("/tmp/m3a4_e2e_metrics.csv");
    expect(metrics_text.find("autonomous_slam_e2e_prelive_enabled_last") !=
               std::string::npos,
           "metrics has e2e enabled");
    expect(metrics_text.find("autonomous_slam_e2e_prelive_warning_count_last") !=
               std::string::npos,
           "metrics has e2e warnings");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
