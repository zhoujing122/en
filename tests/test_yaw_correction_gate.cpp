#include "robot_slamd/config/config.hpp"
#include "robot_slamd/active_scan/yaw_correction_gate.hpp"

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
    c.yaw_correction_scan_completion_source = "either";
    c.yaw_correction_allow_yaw_match_evidence_complete = true;
    c.yaw_correction_min_match_observed_yaw_delta_deg = 90.0;
    c.yaw_correction_min_match_valid_samples = 15;
    c.yaw_correction_min_match_valid_bins = 8;
    c.yaw_correction_min_match_valid_bin_ratio = 0.10;
    c.yaw_correction_require_yaw_match_attempted = true;
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
    s.observed_yaw_delta_deg = 120.0;
    s.valid_samples = 20;
    s.valid_bins = 10;
    s.valid_bin_ratio = 0.25;
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

void clear_active_scan_completion(robot_slamd::YawCorrectionGateInput &in) {
    in.active_scan.state = "IDLE";
    in.active_scan.completed = false;
    in.active_scan.useful_scan_observed = false;
    in.active_scan_command.state = "IDLE";
    in.active_scan_command.completed = false;
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
    expect(cfg.yaw_correction_scan_completion_source == "either", "default scan completion source should be either");

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
    invalid = cfg;
    invalid.yaw_correction_scan_completion_source = "bad_source";
    expect_throw([&] { validate_config(invalid); }, "invalid scan_completion_source should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_match_observed_yaw_delta_deg = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative min_match_observed_yaw_delta_deg should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_match_valid_samples = -1;
    expect_throw([&] { validate_config(invalid); }, "negative min_match_valid_samples should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_match_valid_bins = -1;
    expect_throw([&] { validate_config(invalid); }, "negative min_match_valid_bins should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_match_valid_bin_ratio = 1.1;
    expect_throw([&] { validate_config(invalid); }, "invalid min_match_valid_bin_ratio should be fatal");

    YawCorrectionGate seen(cfg);
    auto first = update_once(seen, input(1, 1.0, 2.0));
    expect(first.state == "CANDIDATE_SEEN" && first.candidate_seen, "usable yaw match should enter CANDIDATE_SEEN");
    expect(first.match_scan_id == 1 && std::fabs(first.match_timestamp_s - 1.0) < 1e-9, "gate snapshot should carry match identity");
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

    Config evidence_cfg = cfg;
    evidence_cfg.yaw_correction_consistency_window = 1;
    YawCorrectionGate old_active_path(evidence_cfg);
    auto active_path_in = input(80, 1.0, 2.0);
    active_path_in.yaw_match.observed_yaw_delta_deg = 10.0;
    active_path_in.yaw_match.valid_samples = 1;
    active_path_in.yaw_match.valid_bins = 1;
    active_path_in.yaw_match.valid_bin_ratio = 0.01;
    auto active_path_s = update_once(old_active_path, active_path_in);
    expect(active_path_s.would_apply && active_path_s.active_scan_evidence_ok && !active_path_s.yaw_match_evidence_ok && active_path_s.scan_evidence_ok, "active_scan completed evidence should still pass default either mode");

    YawCorrectionGate natural_rotation_path(evidence_cfg);
    auto natural_in = input(81, 1.0, 2.0);
    clear_active_scan_completion(natural_in);
    auto natural_s = update_once(natural_rotation_path, natural_in);
    expect(natural_s.would_apply && !natural_s.active_scan_evidence_ok && natural_s.yaw_match_evidence_ok && natural_s.scan_evidence_ok, "yaw_match evidence should pass when active scan is not completed in either mode");

    Config active_only_cfg = evidence_cfg;
    active_only_cfg.yaw_correction_scan_completion_source = "active_scan_only";
    YawCorrectionGate active_only(active_only_cfg);
    auto active_only_in = input(82, 1.0, 2.0);
    clear_active_scan_completion(active_only_in);
    auto active_only_s = update_once(active_only, active_only_in);
    expect(active_only_s.state == "REJECTED" && active_only_s.reason == "active_scan_evidence_not_complete", "active_scan_only should reject without active scan evidence");

    auto reject_yaw_match_source = [&](const char *expected, const std::function<void(YawCorrectionGateInput &)> &mutate) {
        Config yaw_only_cfg = evidence_cfg;
        yaw_only_cfg.yaw_correction_scan_completion_source = "yaw_match_only";
        YawCorrectionGate g(yaw_only_cfg);
        auto in = input(90, 1.0, 2.0);
        mutate(in);
        auto s = update_once(g, in);
        expect(s.state == "REJECTED" && s.reason == expected && s.active_scan_evidence_ok && !s.yaw_match_evidence_ok && !s.scan_evidence_ok, expected);
    };
    reject_yaw_match_source("match_yaw_coverage_too_low", [](auto &in) { in.yaw_match.observed_yaw_delta_deg = 20.0; });
    reject_yaw_match_source("match_valid_samples_too_low", [](auto &in) { in.yaw_match.valid_samples = 1; });
    reject_yaw_match_source("match_valid_bins_too_low", [](auto &in) { in.yaw_match.valid_bins = 1; });
    reject_yaw_match_source("match_valid_bin_ratio_too_low", [](auto &in) { in.yaw_match.valid_bin_ratio = 0.01; });

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

    YawCorrectionGate feedback(cfg);
    update_once(feedback, input(70, 1.0, 2.0));
    update_once(feedback, input(71, 2.0, 2.1));
    auto feedback_apply = update_once(feedback, input(72, 3.0, 2.05));
    expect(feedback_apply.would_apply, "feedback setup should would_apply");
    feedback.notify_yaw_correction_applied(3.1, feedback_apply.match_scan_id, feedback_apply.match_timestamp_s, feedback_apply.suggested_correction_deg);
    auto feedback_snap = feedback.snapshot();
    expect(feedback_snap.state == "COOLDOWN" && feedback_snap.reason == "apply_feedback_cooldown", "apply feedback should put gate into cooldown");
    expect(!feedback_snap.candidate_seen && !feedback_snap.would_apply && !feedback_snap.rejected, "apply feedback should clear candidate booleans");
    expect(feedback_snap.candidate_yaw_delta_deg == 0.0 && feedback_snap.suggested_correction_deg == 0.0, "apply feedback should clear candidate correction fields");
    expect(feedback_snap.best_score == 0.0 && feedback_snap.score_margin == 0.0 && feedback_snap.inlier_ratio == 0.0, "apply feedback should clear score fields");
    expect(feedback_snap.consistency_count == 0 && feedback_snap.consistency_spread_deg == 0.0, "apply feedback should clear consistency fields");
    auto feedback_stats = feedback.run_stats(3.2);
    expect(feedback_stats.apply_feedback_count == 1 && feedback_stats.window_reset_count == 1, "apply feedback should reset gate window and update stats");
    auto after_feedback = update_once(feedback, input(73, 10.0, 2.05));
    expect(!after_feedback.would_apply && after_feedback.consistency_count == 1, "after apply feedback a new candidate must rebuild consistency");
    auto old_again = update_once(feedback, input(72, 3.0, 2.05));
    expect(!old_again.would_apply, "old applied scan_id/timestamp should not be reused");

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
