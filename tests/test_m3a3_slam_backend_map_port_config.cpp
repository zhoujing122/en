#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"

#include <fstream>
#include <functional>
#include <iostream>
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
    expect(!cfg.slam_backend_binding_enabled, "slam backend binding default disabled");
    expect(cfg.slam_backend_binding_require_tof, "require tof default true");
    expect(cfg.slam_backend_binding_require_imu_or_wheel, "require imu or wheel default true");

    Config invalid = cfg;
    invalid.slam_backend_binding_max_input_age_s = 5.1;
    expect_throw([&] { validate_config(invalid); }, "max input age fatal");
    invalid = cfg;
    invalid.slam_backend_binding_max_update_latency_s = 10.1;
    expect_throw([&] { validate_config(invalid); }, "max update latency fatal");
    invalid = cfg;
    invalid.slam_backend_binding_min_integrated_scan_count_for_quality = -1;
    expect_throw([&] { validate_config(invalid); }, "min scan count fatal");

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

    const std::string resolved = "/tmp/m3a3_slam_backend_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("slam_backend_binding:") != std::string::npos, "resolved has slam backend block");
    expect(text.find("max_update_latency_s: 1.000000") != std::string::npos, "resolved has update latency");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3a3_slam_backend_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const auto metrics_text = read_file("/tmp/m3a3_slam_backend_metrics.csv");
    expect(metrics_text.find("slam_backend_binding_enabled_last") != std::string::npos, "metrics has binding enabled");
    expect(metrics_text.find("slam_backend_update_latency_s_last") != std::string::npos, "metrics has update latency");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
