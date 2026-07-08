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
    expect(!cfg.real_sensor_adapter_data_contract_enabled,
           "default disabled");
    expect(cfg.real_sensor_adapter_data_contract_require_tof,
           "default require tof");
    expect(cfg.real_sensor_adapter_data_contract_require_imu_or_wheel,
           "default require imu or wheel");
    expect(cfg.real_sensor_adapter_data_contract_require_request_timing,
           "default request timing");
    expect(cfg.real_sensor_adapter_data_contract_reject_request_latency_mismatch,
           "default reject latency mismatch");
    expect(cfg.real_sensor_adapter_data_contract_require_estimated_sample_time_in_window,
           "default require sample in window");
    expect(cfg.real_sensor_adapter_data_contract_require_estimated_sample_time_midpoint,
           "default require midpoint");
    expect(cfg.real_sensor_adapter_data_contract_reject_future_sensor_time,
           "default reject future sensor time");

    Config invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_packet_age_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "packet age <=0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_packet_age_s = 5.1;
    expect_throw([&] { validate_config(invalid); }, "packet age >5");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_sensor_sync_dt_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "sync <=0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_sensor_sync_dt_s = 1.1;
    expect_throw([&] { validate_config(invalid); }, "sync >1");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_request_latency_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "latency <=0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_request_latency_s = 2.1;
    expect_throw([&] { validate_config(invalid); }, "latency >2");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_tof_nan_ratio = -0.1;
    expect_throw([&] { validate_config(invalid); }, "nan ratio <0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_tof_nan_ratio = 1.1;
    expect_throw([&] { validate_config(invalid); }, "nan ratio >1");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_tof_range_m =
        invalid.real_sensor_adapter_data_contract_min_tof_range_m;
    expect_throw([&] { validate_config(invalid); }, "range max <= min");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_require_request_timing = false;
    expect_throw([&] { validate_config(invalid); }, "request timing fatal");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_run_acceptance_on_startup = true;
    expect_throw([&] { validate_config(invalid); }, "startup acceptance fatal");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_request_latency_mismatch_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "latency mismatch <=0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_estimated_sample_time_midpoint_error_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "midpoint error <=0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_future_timestamp_skew_s = -0.1;
    expect_throw([&] { validate_config(invalid); }, "future skew <0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_max_packet_sensor_time_dt_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "packet sensor dt <=0");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_require_estimated_sample_time_in_window = false;
    expect_throw([&] { validate_config(invalid); }, "sample window fatal");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_require_estimated_sample_time_midpoint = false;
    expect_throw([&] { validate_config(invalid); }, "midpoint fatal");
    invalid = cfg;
    invalid.real_sensor_adapter_data_contract_reject_future_sensor_time = false;
    expect_throw([&] { validate_config(invalid); }, "future reject fatal");

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

    const std::string resolved = "/tmp/m3b1_real_sensor_contract_resolved.yaml";
    write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("real_sensor_adapter_data_contract:") !=
               std::string::npos,
           "resolved block");
    expect(text.find("max_request_latency_mismatch_s") !=
               std::string::npos,
           "resolved hardening field");

    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b1_real_sensor_contract_metrics.csv",
                      metrics,
                      loc,
                      grid,
                      health,
                      cfg,
                      encoder_stats);
    const auto metrics_text =
        read_file("/tmp/m3b1_real_sensor_contract_metrics.csv");
    expect(metrics_text.find("real_sensor_adapter_data_contract_enabled_last") !=
               std::string::npos,
           "metrics enabled field");
    expect(metrics_text.find("real_sensor_adapter_warning_count_last") !=
               std::string::npos,
           "metrics warning field");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
