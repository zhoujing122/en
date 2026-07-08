#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"

#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
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
    expect(!cfg.real_sensor_replay_enabled, "default replay disabled");
    expect(!cfg.real_sensor_replay_loop, "default loop false");
    expect(cfg.real_sensor_replay_fail_on_contract_error,
           "default fail on contract error");
    expect(cfg.real_sensor_replay_require_non_empty_log,
           "default require non empty log");

    Config invalid = cfg;
    invalid.real_sensor_replay_run_acceptance_on_startup = true;
    expect_throw([&] { validate_config(invalid); }, "startup acceptance fatal");
    invalid = cfg;
    invalid.real_sensor_replay_start_time_s =
        std::numeric_limits<double>::quiet_NaN();
    expect_throw([&] { validate_config(invalid); }, "start time nan");
    invalid = cfg;
    invalid.real_sensor_replay_enabled = true;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "enabled hardware fatal");

    invalid = cfg;
    invalid.motion_execution_writer_backend = "software_direction_speed_test_only";
    expect_throw([&] { validate_config(invalid); }, "writer backend fatal");
    invalid = cfg;
    invalid.motion_execution_software_motion_production_interface_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "production fatal");
    invalid = cfg;
    invalid.motion_execution_allow_writer_dispatch = true;
    expect_throw([&] { validate_config(invalid); }, "dispatch fatal");
    invalid = cfg;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "hardware write fatal");
    invalid = cfg;
    invalid.motion_execution_software_motion_production_transport_backend =
        "loopback_shadow";
    invalid.motion_execution_software_motion_loopback_shadow_mode = false;
    expect_throw([&] { validate_config(invalid); }, "loopback fatal");

    const std::string resolved = "/tmp/m3b2_real_sensor_replay_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("real_sensor_replay:") != std::string::npos,
           "resolved replay block");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b2_real_sensor_replay_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const auto metrics_text =
        read_file("/tmp/m3b2_real_sensor_replay_metrics.csv");
    expect(metrics_text.find("real_sensor_replay_enabled_last") !=
               std::string::npos,
           "metrics replay enabled field");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
