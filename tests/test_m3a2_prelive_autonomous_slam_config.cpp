#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"

#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool contains(const std::vector<std::string> &values, const std::string &needle) {
    for (const auto &value : values) {
        if (value == needle) {
            return true;
        }
    }
    return false;
}
} // namespace

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

int main() {
    using namespace robot_slamd;

    Config cfg;
    validate_config(cfg);
    expect(!cfg.prelive_autonomous_slam_enabled, "prelive default disabled");

    Config invalid = cfg;
    invalid.prelive_autonomous_slam_max_iterations = 201;
    expect_throw([&] { validate_config(invalid); }, "max iterations fatal");
    invalid = cfg;
    invalid.prelive_autonomous_slam_step_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "step zero fatal");
    invalid = cfg;
    invalid.prelive_autonomous_slam_step_s = 1.1;
    expect_throw([&] { validate_config(invalid); }, "step high fatal");
    invalid = cfg;
    invalid.prelive_autonomous_slam_start_time_s = std::numeric_limits<double>::quiet_NaN();
    expect_throw([&] { validate_config(invalid); }, "start time nan fatal");
    invalid = cfg;
    invalid.prelive_autonomous_slam_enabled = true;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "enabled cannot open hardware write");

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

    const std::string resolved = "/tmp/m3a2_prelive_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("prelive_autonomous_slam:") != std::string::npos, "resolved has prelive block");
    expect(text.find("allow_coordinator_incomplete_for_shadow: true") != std::string::npos, "resolved has shadow incomplete gate");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3a2_prelive_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const auto metrics_text = read_file("/tmp/m3a2_prelive_metrics.csv");
    expect(metrics_text.find("prelive_autonomous_slam_enabled_last") != std::string::npos, "metrics has prelive enabled");
    expect(metrics_text.find("prelive_autonomous_slam_coordinator_step_count") != std::string::npos, "metrics has coordinator step count");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
