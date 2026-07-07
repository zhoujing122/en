#include "robot_slamd/config/config.hpp"

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

std::string source_dir() {
    std::string file = __FILE__;
    size_t slash = file.find_last_of("/\\");
    return slash == std::string::npos ? "." : file.substr(0, slash);
}
} // namespace

int main() {
    using namespace robot_slamd;
    const std::string cfg_path = source_dir() + "/../config/config.sim.yaml";
    Config cfg = load_config(cfg_path, "");
    validate_config(cfg);
    expect(cfg.motion_execution_software_motion_production_transport_backend == "none", "default backend none");
    expect(cfg.motion_execution_software_motion_loopback_shadow_mode, "default loopback shadow true");
    expect(!cfg.motion_execution_m2b1_preflight_enabled, "default preflight disabled");
    expect(cfg.motion_execution_m2b1_preflight_mode == "confirmed_lifted_live", "default confirmed live mode");
    expect(cfg.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized == 0.03, "default probe speed");
    expect(cfg.motion_execution_m2b1_preflight_direction_probe_max_duration_s == 0.30, "default probe duration");
    expect(!cfg.motion_execution_manual_arm_enable_live_motion, "default live motion disabled");

    Config loopback = cfg;
    loopback.motion_execution_software_motion_production_transport_backend = "loopback_shadow";
    loopback.motion_execution_software_motion_loopback_shadow_mode = true;
    validate_config(loopback);

    Config probe = cfg;
    probe.motion_execution_m2b1_preflight_mode = "lifted_direction_probe";
    validate_config(probe);

    Config invalid = cfg;
    invalid.motion_execution_m2b1_preflight_mode = "direction_probe";
    expect_throw([&] { validate_config(invalid); }, "illegal preflight mode fatal");

    invalid = cfg;
    invalid.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized = 0.031;
    expect_throw([&] { validate_config(invalid); }, "probe speed too high fatal");

    invalid = cfg;
    invalid.motion_execution_m2b1_preflight_direction_probe_max_duration_s = 0.301;
    expect_throw([&] { validate_config(invalid); }, "probe duration too long fatal");

    invalid = cfg;
    invalid.motion_execution_software_motion_production_transport_backend = "loopback_shadow";
    invalid.motion_execution_software_motion_loopback_shadow_mode = false;
    expect_throw([&] { validate_config(invalid); }, "loopback non-shadow fatal");

    invalid = cfg;
    invalid.motion_execution_software_motion_production_transport_backend = "real";
    expect_throw([&] { validate_config(invalid); }, "real backend fatal");

    invalid = cfg;
    invalid.motion_execution_m2b1_preflight_max_live_speed_normalized = 0.051;
    expect_throw([&] { validate_config(invalid); }, "speed too high fatal");

    invalid = cfg;
    invalid.motion_execution_m2b1_preflight_max_live_duration_s = 0.501;
    expect_throw([&] { validate_config(invalid); }, "duration too long fatal");

    invalid = cfg;
    invalid.motion_execution_allow_writer_dispatch = true;
    expect_throw([&] { validate_config(invalid); }, "allow_writer_dispatch remains fatal");

    invalid = cfg;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "hardware_write_enabled remains fatal");

    invalid = cfg;
    invalid.motion_execution_mode = "hardware";
    expect_throw([&] { validate_config(invalid); }, "mode hardware remains fatal");

    invalid = cfg;
    invalid.motion_execution_software_motion_production_interface_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "production interface enabled remains fatal");

    const std::string resolved = "/tmp/m2b1_preflight_resolved.yaml";
    write_resolved_config(probe, resolved);
    std::ifstream in(resolved);
    std::ostringstream ss;
    ss << in.rdbuf();
    std::string text = ss.str();
    expect(text.find("m2b1_preflight:") != std::string::npos, "resolved preflight block");
    expect(text.find("mode: lifted_direction_probe") != std::string::npos, "resolved preflight mode");
    expect(text.find("direction_probe_max_speed_normalized: 0.030000") != std::string::npos, "resolved probe speed");
    expect(text.find("direction_probe_max_duration_s: 0.300000") != std::string::npos, "resolved probe duration");
    expect(text.find("production_transport_backend: loopback_shadow") == std::string::npos, "resolved keeps probe backend default none");
    expect(text.find("manual_arm:") != std::string::npos, "resolved manual arm block");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
