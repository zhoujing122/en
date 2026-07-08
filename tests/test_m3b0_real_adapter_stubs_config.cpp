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
    expect(!cfg.real_adapter_stubs_enabled, "adapter stubs default disabled");
    expect(!cfg.real_adapter_stubs_allow_real_hardware_adapters,
           "real hardware adapters default false");
    expect(cfg.real_adapter_stubs_require_explicit_live_enable,
           "explicit live default true");
    expect(!cfg.live_handoff_readiness_enabled,
           "handoff readiness default disabled");

    Config invalid = cfg;
    invalid.real_adapter_stubs_allow_real_hardware_adapters = true;
    expect_throw([&] { validate_config(invalid); }, "real hardware fatal");
    invalid = cfg;
    invalid.real_adapter_stubs_require_explicit_live_enable = false;
    expect_throw([&] { validate_config(invalid); }, "explicit live fatal");
    invalid = cfg;
    invalid.live_handoff_readiness_allow_forward_backward = true;
    expect_throw([&] { validate_config(invalid); }, "forward backward fatal");
    invalid = cfg;
    invalid.real_adapter_stubs_enabled = true;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "enabled hardware write fatal");

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
    invalid.motion_execution_software_motion_production_transport_backend =
        "loopback_shadow";
    invalid.motion_execution_software_motion_loopback_shadow_mode = false;
    expect_throw([&] { validate_config(invalid); }, "loopback non-shadow fatal");

    const std::string resolved = "/tmp/m3b0_real_adapter_stubs_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("real_adapter_stubs:") != std::string::npos,
           "resolved has stubs");
    expect(text.find("live_handoff_readiness:") != std::string::npos,
           "resolved has handoff");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b0_real_adapter_stubs_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const auto metrics_text =
        read_file("/tmp/m3b0_real_adapter_stubs_metrics.csv");
    expect(metrics_text.find("real_adapter_stubs_enabled_last") !=
               std::string::npos,
           "metrics has stubs enabled");
    expect(metrics_text.find("live_handoff_readiness_enabled_last") !=
               std::string::npos,
           "metrics has handoff enabled");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
