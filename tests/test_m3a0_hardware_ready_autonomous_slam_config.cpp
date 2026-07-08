#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"

#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>

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
    expect(!cfg.autonomous_slam_enabled, "autonomous slam default disabled");
    expect(!cfg.autonomous_slam_allow_forward_backward, "forward backward default disabled");
    expect(cfg.autonomous_slam_require_tof, "tof required by default");
    expect(cfg.autonomous_slam_require_imu_or_wheel, "imu or wheel required by default");

    Config invalid = cfg;
    invalid.autonomous_slam_active_scan_speed = 0.051;
    expect_throw([&] { validate_config(invalid); }, "active scan speed fatal");
    invalid = cfg;
    invalid.autonomous_slam_active_scan_duration_s = 0.501;
    expect_throw([&] { validate_config(invalid); }, "active scan duration fatal");
    invalid = cfg;
    invalid.autonomous_slam_max_iterations = 1001;
    expect_throw([&] { validate_config(invalid); }, "max iterations fatal");
    invalid = cfg;
    invalid.autonomous_slam_sensor_timeout_s = 5.001;
    expect_throw([&] { validate_config(invalid); }, "sensor timeout fatal");
    invalid = cfg;
    invalid.autonomous_slam_motion_settle_timeout_s = 10.001;
    expect_throw([&] { validate_config(invalid); }, "settle timeout fatal");
    invalid = cfg;
    invalid.autonomous_slam_max_active_scan_commands = 101;
    expect_throw([&] { validate_config(invalid); }, "max active scan fatal");
    invalid = cfg;
    invalid.autonomous_slam_allow_forward_backward = true;
    expect_throw([&] { validate_config(invalid); }, "forward backward remains disabled");

    invalid = cfg;
    invalid.motion_execution_writer_backend = "software_direction_speed_test_only";
    expect_throw([&] { validate_config(invalid); }, "writer backend remains fatal");
    invalid = cfg;
    invalid.motion_execution_software_motion_production_interface_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "production interface remains fatal");
    invalid = cfg;
    invalid.motion_execution_allow_writer_dispatch = true;
    expect_throw([&] { validate_config(invalid); }, "allow writer dispatch remains fatal");
    invalid = cfg;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "hardware write remains fatal");
    invalid = cfg;
    invalid.motion_execution_software_motion_production_transport_backend = "loopback_shadow";
    invalid.motion_execution_software_motion_loopback_shadow_mode = false;
    expect_throw([&] { validate_config(invalid); }, "loopback non-shadow remains fatal");

    const std::string resolved = "/tmp/m3a0_autonomous_slam_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const std::string text = read_file(resolved);
    expect(text.find("autonomous_slam:") != std::string::npos,
           "resolved has autonomous slam block");
    expect(text.find("active_scan_speed: 0.050000") != std::string::npos,
           "resolved has active scan speed");
    expect(text.find("allow_forward_backward: false") != std::string::npos,
           "resolved has forward backward disabled");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3a0_autonomous_slam_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const std::string metrics_text = read_file("/tmp/m3a0_autonomous_slam_metrics.csv");
    expect(metrics_text.find("autonomous_slam_enabled_last") != std::string::npos,
           "metrics has autonomous slam enabled");
    expect(metrics_text.find("autonomous_slam_fault_count") != std::string::npos,
           "metrics has autonomous slam fault count");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
