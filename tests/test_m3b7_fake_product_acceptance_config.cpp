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
    Config cfg; validate_config(cfg);
    expect(!cfg.fake_relocalization_readiness_gate_enabled, "gate disabled");
    expect(!cfg.fake_autonomous_slam_product_acceptance_enabled, "acceptance disabled");
    Config invalid = cfg; invalid.fake_autonomous_slam_product_acceptance_run_on_startup = true; expect_throw([&] { validate_config(invalid); }, "startup fatal");
    invalid = cfg; invalid.fake_autonomous_slam_product_acceptance_require_no_pose_writeback = false; expect_throw([&] { validate_config(invalid); }, "writeback fatal");
    invalid = cfg; invalid.fake_autonomous_slam_product_acceptance_require_no_forward_backward = false; expect_throw([&] { validate_config(invalid); }, "forward fatal");
    invalid = cfg; invalid.fake_relocalization_readiness_gate_min_map_yaw_coverage_ratio = 0.1; expect_throw([&] { validate_config(invalid); }, "gate yaw fatal");
    invalid = cfg; invalid.fake_relocalization_min_map_yaw_coverage_ratio = 0.1; expect_throw([&] { validate_config(invalid); }, "relocalization yaw fatal");
    invalid = cfg; invalid.motion_execution_writer_backend = "software_direction_speed_test_only"; expect_throw([&] { validate_config(invalid); }, "old writer fatal");
    write_resolved_config(cfg, "/tmp/m3b7_resolved.yaml");
    const auto resolved = read_file_c("/tmp/m3b7_resolved.yaml");
    expect(resolved.find("fake_relocalization_readiness_gate:") != std::string::npos, "resolved gate");
    expect(resolved.find("fake_autonomous_slam_product_acceptance:") != std::string::npos, "resolved acceptance");
    RunMetrics metrics; Localizer loc(cfg); ChunkedGrid grid(cfg); TofHealth health; EncoderStats encoder_stats;
    write_run_metrics("/tmp/m3b7_metrics.csv", metrics, loc, grid, health, cfg, encoder_stats);
    const auto metrics_text = read_file_c("/tmp/m3b7_metrics.csv");
    expect(metrics_text.find("fake_autonomous_slam_product_acceptance_enabled_last") != std::string::npos, "metrics acceptance");
    return failures ? 1 : 0;
}
