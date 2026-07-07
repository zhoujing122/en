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

    expect(!cfg.motion_execution_algorithm_motion_enabled, "algorithm motion default disabled");
    expect(!cfg.motion_execution_algorithm_motion_allow_translation_commands, "translation default disabled");
    expect(cfg.motion_execution_algorithm_motion_allow_rotation_commands, "rotation default enabled");
    expect(!cfg.motion_execution_algorithm_motion_allow_recovery_commands, "recovery default disabled");
    expect(!cfg.motion_execution_algorithm_motion_allow_manual_test_commands, "manual default disabled");

    Config invalid = cfg;
    invalid.motion_execution_algorithm_motion_direction_probe_speed = 0.031;
    expect_throw([&] { validate_config(invalid); }, "direction probe speed fatal");
    invalid = cfg;
    invalid.motion_execution_algorithm_motion_active_scan_speed = 0.051;
    expect_throw([&] { validate_config(invalid); }, "active scan speed fatal");
    invalid = cfg;
    invalid.motion_execution_algorithm_motion_recovery_speed = 0.051;
    expect_throw([&] { validate_config(invalid); }, "recovery speed fatal");
    invalid = cfg;
    invalid.motion_execution_algorithm_motion_manual_test_speed = 0.031;
    expect_throw([&] { validate_config(invalid); }, "manual speed fatal");
    invalid = cfg;
    invalid.motion_execution_algorithm_motion_default_ttl_s = 0.501;
    expect_throw([&] { validate_config(invalid); }, "ttl high fatal");

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

    const std::string resolved = "/tmp/algorithm_motion_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const std::string text = read_file(resolved);
    expect(text.find("algorithm_motion:") != std::string::npos, "resolved has algorithm motion block");
    expect(text.find("enabled: false") != std::string::npos, "resolved enabled false present");
    expect(text.find("allow_translation_commands: false") != std::string::npos, "resolved translation false");
    expect(text.find("active_scan_speed: 0.050000") != std::string::npos, "resolved active scan speed");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
