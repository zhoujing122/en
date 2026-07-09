#include "robot_slamd/config/config.hpp"
#include "robot_slamd/core/metrics.hpp"
#include <cstdio>
#include <functional>
#include <iostream>
#include <string>
namespace {
int failures = 0;
void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } }
void expect_throw(const std::function<void()> &fn, const char *m) { try { fn(); std::cerr << "FAIL: expected throw: " << m << "\n"; failures++; } catch (const std::exception &) {} }
std::string read_file_c(const char *path) { std::string out; char buf[512]; FILE *f = std::fopen(path, "rb"); if (!f) return out; while (true) { const auto n = std::fread(buf, 1, sizeof(buf), f); if (n > 0) out.append(buf, n); if (n < sizeof(buf)) break; } std::fclose(f); return out; }
}
int main() {
    using namespace robot_slamd;
    Config cfg;
    validate_config(cfg);
    expect(!cfg.fake_map_artifact_enabled, "fake map disabled");
    expect(!cfg.fake_map_artifact_load_enabled, "load disabled");
    expect(cfg.full_autonomous_slam_fake_pipeline_phase_aware_sensor_consumption, "phase default");
    expect(cfg.full_autonomous_slam_fake_pipeline_require_phase_aware_sensor_consumption, "require phase default");
    expect(cfg.full_autonomous_slam_fake_pipeline_build_fake_map_on_completed, "build default");
    expect(cfg.full_autonomous_slam_fake_pipeline_save_fake_map_on_completed, "save default");
    expect(cfg.full_autonomous_slam_fake_pipeline_require_fake_map_saved, "require save default");
    Config invalid = cfg;
    invalid.full_autonomous_slam_fake_pipeline_fake_map_id_prefix.clear(); expect_throw([&] { validate_config(invalid); }, "prefix fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_phase_aware_sensor_consumption = false; expect_throw([&] { validate_config(invalid); }, "phase fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_save_fake_map_on_completed = false; expect_throw([&] { validate_config(invalid); }, "save fatal");
    invalid = cfg; invalid.full_autonomous_slam_fake_pipeline_require_fake_map_saved = false; expect_throw([&] { validate_config(invalid); }, "require save fatal");
    invalid = cfg; invalid.motion_execution_writer_backend = "software_direction_speed_test_only"; expect_throw([&] { validate_config(invalid); }, "writer fatal");
    invalid = cfg; invalid.motion_execution_hardware_write_enabled = true; expect_throw([&] { validate_config(invalid); }, "hardware fatal");
    write_resolved_config(cfg, "/tmp/m3b5_resolved.yaml");
    const auto resolved = read_file_c("/tmp/m3b5_resolved.yaml");
    expect(resolved.find("fake_map_artifact:") != std::string::npos, "resolved fake map");
    expect(resolved.find("phase_aware_sensor_consumption") != std::string::npos, "resolved phase");
    RunMetrics metrics;
    Localizer loc(cfg);
    ChunkedGrid grid(cfg);
    TofHealth health;
    EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b5_metrics.csv", metrics, loc, grid, health, cfg, encoder_stats);
    const auto metrics_text = read_file_c("/tmp/m3b5_metrics.csv");
    expect(metrics_text.find("fake_map_artifact_enabled_last") != std::string::npos, "metrics fake map");
    return failures ? 1 : 0;
}
