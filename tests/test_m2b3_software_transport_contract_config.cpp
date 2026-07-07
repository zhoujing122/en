#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/software_motion_metrics_writer.hpp"

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

    expect(!cfg.motion_execution_software_transport_contract_enabled,
           "contract default disabled");
    expect(!cfg.motion_execution_software_transport_contract_allow_translation_commands,
           "translation default disabled");
    expect(cfg.motion_execution_software_transport_contract_allow_rotation_commands,
           "rotation default enabled");
    expect(cfg.motion_execution_software_transport_contract_require_shadow_mode,
           "shadow required by default");

    Config invalid = cfg;
    invalid.motion_execution_software_transport_contract_max_speed_normalized = 0.051;
    expect_throw([&] { validate_config(invalid); }, "max speed fatal");
    invalid = cfg;
    invalid.motion_execution_software_transport_contract_max_direction_probe_speed = 0.031;
    expect_throw([&] { validate_config(invalid); }, "direction probe fatal");
    invalid = cfg;
    invalid.motion_execution_software_transport_contract_max_ttl_s = 0.501;
    expect_throw([&] { validate_config(invalid); }, "ttl fatal");
    invalid = cfg;
    invalid.motion_execution_software_transport_contract_max_command_age_s = 1.001;
    expect_throw([&] { validate_config(invalid); }, "age fatal");

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

    const std::string resolved = "/tmp/m2b3_software_transport_contract_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const std::string text = read_file(resolved);
    expect(text.find("software_transport_contract:") != std::string::npos,
           "resolved has software transport contract block");
    expect(text.find("max_speed_normalized: 0.050000") != std::string::npos,
           "resolved max speed present");
    expect(text.find("allow_translation_commands: false") != std::string::npos,
           "resolved translation disabled present");

    RunMetrics metrics;
    std::ostringstream metrics_out;
    write_software_motion_metrics(metrics_out, metrics, cfg);
    const std::string metrics_text = metrics_out.str();
    expect(metrics_text.find("software_transport_contract_enabled_last") != std::string::npos,
           "metrics has contract enabled field");
    expect(metrics_text.find("software_transport_shadow_adapter_send_count") != std::string::npos,
           "metrics has shadow adapter field");
    expect(metrics_text.find("software_transport_acceptance_fail_count") != std::string::npos,
           "metrics has acceptance field");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
