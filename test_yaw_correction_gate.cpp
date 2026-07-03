#include "config.hpp"
#include "yaw_correction_gate.hpp"

#include <functional>
#include <iostream>

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

void expect_near(double actual, double expected, double tol, const char *message) {
    if (std::fabs(actual - expected) > tol) {
        std::cerr << "FAIL: " << message << " actual=" << actual << " expected=" << expected << "\n";
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

void expect_no_throw(const std::function<void()> &fn, const char *message) {
    try {
        fn();
    } catch (const std::exception &e) {
        std::cerr << "FAIL: unexpected throw: " << message << ": " << e.what() << "\n";
        failures++;
    }
}

robot_slamd::Config gate_config() {
    robot_slamd::Config c;
    c.yaw_correction_enabled = true;
    c.yaw_correction_mode = "dry_run";
    c.yaw_correction_log_hz = 10.0;
    c.yaw_correction_require_yaw_match_usable = true;
    c.yaw_correction_require_mapping_state_ok = true;
    c.yaw_correction_require_low_speed_or_static = true;
    c.yaw_correction_require_active_scan_complete = true;
    c.yaw_correction_max_linear_speed_mps = 0.05;
    c.yaw_correction_max_abs_yaw_rate_dps = 5.0;
    c.yaw_correction_min_best_score = 0.20;
    c.yaw_correction_min_score_margin = 0.08;
    c.yaw_correction_min_inlier_ratio = 0.20;
    c.yaw_correction_max_curve_flatness = 0.80;
    c.yaw_correction_max_candidate_abs_deg = 5.0;
    c.yaw_correction_min_candidate_abs_deg = 0.3;
    c.yaw_correction_gain = 0.15;
    c.yaw_correction_max_step_deg = 1.0;
    c.yaw_correction_min_step_deg = 0.05;
    c.yaw_correction_consistency_window = 3;
    c.yaw_correction_max_consistency_spread_deg = 1.0;
    c.yaw_correction_cooldown_s = 5.0;
    c.yaw_correction_writeback_enabled = false;
    return c;
}

robot_slamd::SparseScanYawMatchSummary match(uint64_t id, double t, double delta_deg = 2.0) {
    robot_slamd::SparseScanYawMatchSummary s;
    s.scan_id = id;
    s.timestamp_s = t;
    s.attempted = true;
    s.usable = true;
    s.reason = "ok";
    s.best_yaw_delta_deg = delta_deg;
    s.best_score = 0.5;
    s.score_margin = 0.2;
    s.inlier_ratio = 0.6;
    s.curve_flat = false;
    s.curve_flatness = 0.2;
    s.multimodal = false;
    return s;
}

robot_slamd::YawCorrectionGateInput input(uint64_t id, double t, double delta_deg = 2.0) {
    robot_slamd::YawCorrectionGateInput in;
    in.timestamp_s = t;
    in.yaw_match = match(id, t, delta_deg);
    in.map_quality.quality_score = 0.6;
    in.map_quality.quality_level = "GOOD";
    in.supervisor.state = "MAPPING";
    in.supervisor.mapping_allowed = true;
    in.active_scan.state = "COMPLETED";
    in.active_scan.completed = true;
    in.active_scan_command.state = "COMPLETED";
    in.active_scan_command.completed = true;
    in.current_yaw_rad = 0.0;
    in.linear_speed_mps = 0.0;
    in.yaw_rate_radps = 0.0;
    in.localization_valid = true;
    return in;
}

robot_slamd::YawCorrectionGateSnapshot update_once(robot_slamd::YawCorrectionGate &gate, const robot_slamd::YawCorrectionGateInput &in) {
    gate.update(in);
    return gate.snapshot();
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg = gate_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid yaw_correction config should pass");

    YawCorrectionGate gate(cfg);
    expect(gate.snapshot().state == "IDLE", "default enabled gate should start IDLE");

    Config disabled_cfg = cfg;
    disabled_cfg.yaw_correction_enabled = false;
    YawCorrectionGate disabled(disabled_cfg);
    auto disabled_snap = update_once(disabled, input(1, 1.0));
    expect(disabled_snap.state == "DISABLED" && disabled_snap.reason == "disabled", "disabled gate should stay DISABLED");

    Config invalid = cfg;
    invalid.yaw_correction_mode = "writeback";
    expect_throw([&] { validate_config(invalid); }, "mode=writeback should be fatal");
    invalid = cfg;
    invalid.yaw_correction_writeback_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "writeback_enabled=true should be fatal");
    invalid = cfg;
    invalid.yaw_correction_log_hz = 0.0;
    expect_throw([&] { validate_config(invalid); }, "log_hz <= 0 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_linear_speed_mps = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative max linear speed should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_abs_yaw_rate_dps = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative max yaw rate should be fatal");
    invalid = cfg;
    invalid.yaw_correction_gain = 0.0;
    expect_throw([&] { validate_config(invalid); }, "gain <= 0 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_gain = 1.1;
    expect_throw([&] { validate_config(invalid); }, "gain > 1 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_step_deg = 0.0;
    expect_throw([&] { validate_config(invalid); }, "max_step <= 0 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_step_deg = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative min_step should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_step_deg = 2.0;
    expect_throw([&] { validate_config(invalid); }, "min_step > max_step should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_candidate_abs_deg = 0.0;
    expect_throw([&] { validate_config(invalid); }, "max_candidate_abs <= 0 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_candidate_abs_deg = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative min_candidate_abs should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_candidate_abs_deg = 6.0;
    expect_throw([&] { validate_config(invalid); }, "min_candidate_abs > max_candidate_abs should be fatal");
    invalid = cfg;
    invalid.yaw_correction_consistency_window = 0;
    expect_throw([&] { validate_config(invalid); }, "consistency_window < 1 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_consistency_spread_deg = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative consistency spread should be fatal");
    invalid = cfg;
    invalid.yaw_correction_cooldown_s = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative cooldown should be fatal");

    YawCorrectionGate seen(cfg);
    auto first = update_once(seen, input(1, 1.0, 2.0));
    expect(first.state == "CANDIDATE_SEEN" && first.candidate_seen, "usable yaw match should enter CANDIDATE_SEEN");
    expect(!first.would_apply && first.reason == "consistency_not_enough", "first candidate should wait for consistency");

    auto reject_case = [&](const char *expected, const std::function<void(YawCorrectionGateInput &)> &mutate) {
        YawCorrectionGate g(cfg);
        auto in = input(10, 1.0, 2.0);
        mutate(in);
        auto s = update_once(g, in);
        expect(s.state == "REJECTED" && s.rejected && s.reason == expected, expected);
    };

    reject_case("yaw_match_not_usable", [](auto &in) { in.yaw_match.usable = false; });
    reject_case("yaw_match_reason_not_ok", [](auto &in) { in.yaw_match.reason = "low_best_score"; });
    reject_case("low_best_score", [](auto &in) { in.yaw_match.best_score = 0.1; });
    reject_case("low_score_margin", [](auto &in) { in.yaw_match.score_margin = 0.01; });
    reject_case("low_inlier_ratio", [](auto &in) { in.yaw_match.inlier_ratio = 0.1; });
    reject_case("curve_flat", [](auto &in) { in.yaw_match.curve_flat = true; });
    reject_case("curve_flat", [](auto &in) { in.yaw_match.curve_flatness = 0.9; });
    reject_case("multimodal", [](auto &in) { in.yaw_match.multimodal = true; });
    reject_case("candidate_too_large", [](auto &in) { in.yaw_match.best_yaw_delta_deg = 8.0; });
    reject_case("candidate_too_small", [](auto &in) { in.yaw_match.best_yaw_delta_deg = 0.1; });
    reject_case("robot_moving", [](auto &in) { in.linear_speed_mps = 0.2; });
    reject_case("yaw_rate_too_high", [](auto &in) { in.yaw_rate_radps = deg2rad(8.0); });
    reject_case("localization_invalid", [](auto &in) { in.localization_valid = false; });
    reject_case("supervisor_lost", [](auto &in) { in.supervisor.state = "LOST"; });
    reject_case("active_scan_not_complete", [](auto &in) { in.active_scan.completed = false; in.active_scan.state = "MAPPING"; in.active_scan_command.completed = false; in.active_scan_command.state = "IDLE"; });

    YawCorrectionGate consistency(cfg);
    auto c1 = update_once(consistency, input(20, 1.0, 2.0));
    auto c2 = update_once(consistency, input(21, 2.0, 2.2));
    expect(!c1.would_apply && !c2.would_apply, "consistency_window not enough should not would_apply");
    expect(c2.state == "CONSISTENCY_CHECK" && c2.consistency_count == 2, "second candidate should be in consistency check");

    YawCorrectionGate spread(cfg);
    update_once(spread, input(30, 1.0, 2.0));
    update_once(spread, input(31, 2.0, 2.2));
    auto spread_s = update_once(spread, input(32, 3.0, 3.5));
    expect(spread_s.state == "REJECTED" && spread_s.reason == "consistency_spread_too_large", "wide candidate spread should reject");

    YawCorrectionGate apply(cfg);
    update_once(apply, input(40, 1.0, 2.0));
    update_once(apply, input(41, 2.0, 2.1));
    auto apply_s = update_once(apply, input(42, 3.0, 2.05));
    expect(apply_s.state == "WOULD_APPLY" && apply_s.would_apply && !apply_s.rejected, "consistent candidates should would_apply in dry-run");
    expect_near(apply_s.suggested_correction_deg, 2.05 * cfg.yaw_correction_gain, 1e-9, "correction_gain should scale candidate");
    expect_near(apply_s.suggested_new_yaw_rad, deg2rad(apply_s.suggested_correction_deg), 1e-9, "suggested_new_yaw should add correction to current yaw");
    auto stats = apply.run_stats(4.0);
    expect(stats.would_apply_count == 1 && stats.candidates_seen == 3, "run stats should count dry-run apply and candidates");

    Config limit_cfg = cfg;
    limit_cfg.yaw_correction_consistency_window = 1;
    limit_cfg.yaw_correction_gain = 1.0;
    limit_cfg.yaw_correction_max_step_deg = 1.0;
    limit_cfg.yaw_correction_max_candidate_abs_deg = 10.0;
    YawCorrectionGate limit(limit_cfg);
    auto limit_s = update_once(limit, input(50, 1.0, 4.0));
    expect(limit_s.would_apply, "window=1 should allow immediate apply");
    expect_near(limit_s.suggested_correction_deg, 1.0, 1e-9, "max_step should clamp correction");

    Config wrap_cfg = limit_cfg;
    wrap_cfg.yaw_correction_max_step_deg = 5.0;
    YawCorrectionGate wrap(wrap_cfg);
    auto wrap_in = input(60, 1.0, 10.0);
    wrap_in.current_yaw_rad = deg2rad(179.0);
    auto wrap_s = update_once(wrap, wrap_in);
    expect(wrap_s.would_apply, "wrap test should would_apply");
    expect_near(wrap_s.suggested_new_yaw_rad, deg2rad(-176.0), 1e-9, "suggested_new_yaw should wrap through pi");

    auto cooldown_in = input(61, 2.0, 2.0);
    auto cooldown_s = update_once(wrap, cooldown_in);
    expect(cooldown_s.state == "COOLDOWN" && cooldown_s.reason == "cooldown" && cooldown_s.rejected, "cooldown should block repeated candidate");

    YawCorrectionGate duplicate(cfg);
    update_once(duplicate, input(70, 1.0, 2.0));
    auto duplicate_s = update_once(duplicate, input(70, 1.0, 2.0));
    auto duplicate_stats = duplicate.run_stats(2.0);
    expect(duplicate_stats.candidates_seen == 1, "duplicate scan_id/timestamp should not count twice");
    expect(!duplicate_s.would_apply, "duplicate candidate should not advance consistency");

    if (failures) {
        std::cerr << failures << " failure(s)\n";
        return 1;
    }
    return 0;
}
