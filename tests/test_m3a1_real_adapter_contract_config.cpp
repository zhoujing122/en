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
    expect(!cfg.real_adapter_contract_enabled, "real adapter contract default disabled");
    expect(cfg.real_adapter_contract_require_tof, "require tof default true");
    expect(cfg.real_adapter_contract_require_imu_or_wheel, "require imu or wheel default true");

    Config invalid = cfg;
    invalid.real_adapter_contract_tof_max_frame_age_s = 5.1;
    expect_throw([&] { validate_config(invalid); }, "tof frame age fatal");
    invalid = cfg;
    invalid.real_adapter_contract_tof_max_allowed_nan_ratio = 1.1;
    expect_throw([&] { validate_config(invalid); }, "tof nan ratio fatal");
    invalid = cfg;
    invalid.real_adapter_contract_tof_max_range_count = 2;
    invalid.real_adapter_contract_tof_min_range_count = 3;
    expect_throw([&] { validate_config(invalid); }, "tof range count fatal");
    invalid = cfg;
    invalid.real_adapter_contract_imu_max_abs_acc_mps2 = 201.0;
    expect_throw([&] { validate_config(invalid); }, "imu acc fatal");
    invalid = cfg;
    invalid.real_adapter_contract_wheel_max_abs_linear_mps = 10.1;
    expect_throw([&] { validate_config(invalid); }, "wheel linear fatal");

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

    const std::string resolved = "/tmp/m3a1_real_adapter_contract_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const std::string text = read_file(resolved);
    expect(text.find("real_adapter_contract:") != std::string::npos, "resolved has real adapter block");
    expect(text.find("tof_max_allowed_nan_ratio: 0.250000") != std::string::npos, "resolved has nan ratio");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3a1_real_adapter_contract_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const std::string metrics_text = read_file("/tmp/m3a1_real_adapter_contract_metrics.csv");
    expect(metrics_text.find("real_adapter_contract_enabled_last") != std::string::npos, "metrics has contract enabled");
    expect(metrics_text.find("real_adapter_map_quality_ok_last") != std::string::npos, "metrics has map quality ok");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
