#include "config.hpp"
#include "yaw_correction_apply.hpp"

#include <cmath>
#include <functional>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

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

robot_slamd::Config writeback_config() {
    robot_slamd::Config c;
    c.yaw_correction_enabled = true;
    c.yaw_correction_mode = "writeback";
    c.yaw_correction_writeback_enabled = true;
    c.yaw_correction_writeback_acknowledgement = c.yaw_correction_required_writeback_acknowledgement;
    c.yaw_correction_log_hz = 10.0;
    c.yaw_correction_max_linear_speed_mps = 0.05;
    c.yaw_correction_max_abs_yaw_rate_dps = 5.0;
    c.yaw_correction_max_writeback_abs_deg = 1.0;
    c.yaw_correction_max_total_writeback_per_session_deg = 3.0;
    c.yaw_correction_max_writeback_count_per_session = 5;
    c.yaw_correction_min_seconds_between_writebacks = 5.0;
    c.yaw_correction_require_gate_would_apply = true;
    c.yaw_correction_require_gate_reason = "would_apply_dry_run";
    c.yaw_correction_require_scan_evidence_ok = true;
    c.yaw_correction_require_yaw_match_evidence_ok = true;
    return c;
}

robot_slamd::YawCorrectionGateSnapshot gate_snapshot(double correction_deg = 0.5) {
    robot_slamd::YawCorrectionGateSnapshot g;
    g.timestamp_s = 1.0;
    g.state = "WOULD_APPLY";
    g.reason = "would_apply_dry_run";
    g.candidate_seen = true;
    g.would_apply = true;
    g.rejected = false;
    g.candidate_yaw_delta_deg = 2.0;
    g.suggested_correction_deg = correction_deg;
    g.suggested_new_yaw_rad = robot_slamd::deg2rad(90.0);
    g.best_score = 0.5;
    g.score_margin = 0.2;
    g.inlier_ratio = 0.6;
    g.scan_evidence_ok = true;
    g.yaw_match_evidence_ok = true;
    return g;
}

robot_slamd::MappingSupervisorSnapshot supervisor_snapshot(const std::string &state = "MAPPING") {
    robot_slamd::MappingSupervisorSnapshot s;
    s.state = state;
    s.mapping_allowed = state != "LOST";
    return s;
}

robot_slamd::YawCorrectionApplySnapshot apply_once(robot_slamd::YawCorrectionApply &apply,
                                                   robot_slamd::Localizer &loc,
                                                   const robot_slamd::YawCorrectionGateSnapshot &gate,
                                                   double t = 1.0,
                                                   double linear_speed = 0.0,
                                                   double yaw_rate_radps = 0.0,
                                                   const std::string &supervisor_state = "MAPPING",
                                                   bool localization_valid = true) {
    return apply.update(t, gate, loc, loc.pose().yaw, linear_speed, yaw_rate_radps,
                        supervisor_snapshot(supervisor_state), localization_valid);
}

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void static_safety_search() {
    std::string file = __FILE__;
    size_t slash = file.find_last_of("/\\");
    std::string dir = slash == std::string::npos ? std::string() : file.substr(0, slash + 1);
    std::vector<std::string> production = {
        dir + "yaw_correction_apply.hpp",
        dir + "localizer.hpp",
        dir + "app.hpp",
    };
    std::vector<std::string> forbidden = {
        "apply_pose_correction",
        "set_pose",
        "reset_pose",
        "relocalization",
        "motor_write",
        "bl4820_write",
        "pwm",
        "fr_output",
    };
    for (const auto &path : production) {
        std::string content = read_file(path);
        expect(!content.empty(), "production file should be readable for safety search");
        for (const auto &needle : forbidden) {
            expect(content.find(needle) == std::string::npos, ("production code must not contain dangerous token " + needle).c_str());
        }
    }
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config dry;
    YawCorrectionApply dry_apply(dry);
    Localizer dry_loc(dry);
    auto dry_snap = apply_once(dry_apply, dry_loc, gate_snapshot(), 1.0);
    expect(!dry_snap.attempted && !dry_snap.applied && dry_snap.reason == "not_writeback_mode", "default dry_run should not attempt writeback");
    expect_near(dry_loc.pose().yaw, 0.0, 1e-12, "default dry_run should not change yaw");

    Config invalid = writeback_config();
    invalid.yaw_correction_writeback_enabled = false;
    expect_throw([&] { validate_config(invalid); }, "mode=writeback without writeback_enabled should be fatal");
    invalid = writeback_config();
    invalid.yaw_correction_writeback_acknowledgement = "wrong";
    expect_throw([&] { validate_config(invalid); }, "writeback ack mismatch should be fatal");
    invalid = writeback_config();
    invalid.yaw_correction_mode = "dry_run";
    invalid.yaw_correction_writeback_enabled = true;
    invalid.yaw_correction_writeback_acknowledgement = "wrong";
    expect_throw([&] { validate_config(invalid); }, "writeback_enabled true with bad ack should be fatal even in dry_run");
    Config cfg = writeback_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid writeback config with ack should pass");
    invalid = cfg;
    invalid.yaw_correction_mode = "bad";
    expect_throw([&] { validate_config(invalid); }, "unsupported yaw correction mode should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_writeback_abs_deg = 0.0;
    expect_throw([&] { validate_config(invalid); }, "max_writeback_abs_deg <= 0 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_total_writeback_per_session_deg = 0.0;
    expect_throw([&] { validate_config(invalid); }, "max_total_writeback_per_session_deg <= 0 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_max_writeback_count_per_session = 0;
    expect_throw([&] { validate_config(invalid); }, "max_writeback_count_per_session <= 0 should be fatal");
    invalid = cfg;
    invalid.yaw_correction_min_seconds_between_writebacks = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative min_seconds_between_writebacks should be fatal");

    auto reject_case = [&](const char *expected, const std::function<void(YawCorrectionGateSnapshot &)> &mutate,
                           double linear_speed = 0.0, double yaw_rate_radps = 0.0,
                           const std::string &supervisor_state = "MAPPING", bool localization_valid = true) {
        Config rcfg = writeback_config();
        YawCorrectionApply apply(rcfg);
        Localizer loc(rcfg);
        auto g = gate_snapshot();
        mutate(g);
        auto s = apply_once(apply, loc, g, 1.0, linear_speed, yaw_rate_radps, supervisor_state, localization_valid);
        expect(s.attempted && !s.applied && s.reason == expected, expected);
        expect_near(loc.pose().yaw, 0.0, 1e-12, "rejected writeback should not change yaw");
    };

    reject_case("gate_not_would_apply", [](auto &g) { g.would_apply = false; });
    reject_case("gate_rejected", [](auto &g) { g.rejected = true; g.would_apply = false; });
    reject_case("gate_scan_evidence_not_ok", [](auto &g) { g.scan_evidence_ok = false; });
    reject_case("gate_yaw_match_evidence_not_ok", [](auto &g) { g.yaw_match_evidence_ok = false; });
    reject_case("candidate_not_finite", [](auto &g) { g.suggested_correction_deg = std::numeric_limits<double>::quiet_NaN(); });
    reject_case("correction_too_large", [](auto &g) { g.suggested_correction_deg = 2.0; });
    reject_case("robot_moving", [](auto &) {}, 0.2);
    reject_case("yaw_rate_too_high", [](auto &) {}, 0.0, deg2rad(8.0));
    reject_case("supervisor_lost", [](auto &) {}, 0.0, 0.0, "LOST");
    reject_case("localization_invalid", [](auto &) {}, 0.0, 0.0, "MAPPING", false);

    {
        Config ok = writeback_config();
        YawCorrectionApply apply(ok);
        Localizer loc(ok);
        double old_x = loc.pose().x;
        double old_y = loc.pose().y;
        auto s = apply_once(apply, loc, gate_snapshot(0.5), 1.0);
        expect(s.applied && s.reason == "applied", "normal gated writeback should apply");
        expect_near(loc.pose().yaw, deg2rad(0.5), 1e-12, "writeback should apply delta yaw");
        expect_near(loc.pose().x, old_x, 1e-12, "writeback must not change x");
        expect_near(loc.pose().y, old_y, 1e-12, "writeback must not change y");
        expect(apply.run_stats().attempts == 1 && apply.run_stats().success_count == 1, "success metrics should increment");
    }

    {
        Config ok = writeback_config();
        YawCorrectionApply apply(ok);
        Localizer loc(ok);
        auto first = apply_once(apply, loc, gate_snapshot(0.5), 1.0);
        auto second = apply_once(apply, loc, gate_snapshot(0.5), 2.0);
        expect(first.applied, "first cooldown test apply should succeed");
        expect(!second.applied && second.reason == "cooldown", "second apply inside cooldown should reject");
        expect(apply.run_stats().cooldown_reject_count == 1, "cooldown metric should increment");
    }

    {
        Config ok = writeback_config();
        ok.yaw_correction_min_seconds_between_writebacks = 0.0;
        ok.yaw_correction_max_writeback_count_per_session = 1;
        YawCorrectionApply apply(ok);
        Localizer loc(ok);
        expect(apply_once(apply, loc, gate_snapshot(0.5), 1.0).applied, "first count-limit apply should succeed");
        auto second = apply_once(apply, loc, gate_snapshot(0.5), 2.0);
        expect(!second.applied && second.reason == "session_writeback_count_limit", "session count limit should reject");
    }

    {
        Config ok = writeback_config();
        ok.yaw_correction_min_seconds_between_writebacks = 0.0;
        ok.yaw_correction_max_total_writeback_per_session_deg = 0.75;
        YawCorrectionApply apply(ok);
        Localizer loc(ok);
        expect(apply_once(apply, loc, gate_snapshot(0.5), 1.0).applied, "first angle-limit apply should succeed");
        auto second = apply_once(apply, loc, gate_snapshot(0.5), 2.0);
        expect(!second.applied && second.reason == "session_writeback_angle_limit", "session total angle limit should reject");
    }

    {
        Config ok = writeback_config();
        ok.yaw_correction_max_writeback_abs_deg = 10.0;
        ok.yaw_correction_max_total_writeback_per_session_deg = 20.0;
        Config loc_cfg = ok;
        loc_cfg.yaw_correction_max_writeback_abs_deg = 200.0;
        YawCorrectionApply apply(ok);
        Localizer loc(loc_cfg);
        expect(loc.apply_yaw_correction_only(deg2rad(179.0), "seed"), "wrap test seed should succeed");
        auto s = apply_once(apply, loc, gate_snapshot(5.0), 1.0);
        expect(s.applied, "wrap test apply should succeed");
        expect_near(loc.pose().yaw, deg2rad(-176.0), 1e-12, "yaw writeback should wrap pi");
    }

    {
        Config ok = writeback_config();
        YawCorrectionApply apply(ok);
        Localizer loc(ok);
        auto g = gate_snapshot(0.5);
        g.suggested_new_yaw_rad = deg2rad(90.0);
        auto s = apply_once(apply, loc, g, 1.0);
        expect(s.applied, "delta-only test apply should succeed");
        expect_near(loc.pose().yaw, deg2rad(0.5), 1e-12, "writeback must use delta, not absolute suggested_new_yaw_rad");
    }

    {
        Config apply_cfg = writeback_config();
        apply_cfg.yaw_correction_max_writeback_abs_deg = 5.0;
        Config loc_cfg = apply_cfg;
        loc_cfg.yaw_correction_max_writeback_abs_deg = 1.0;
        YawCorrectionApply apply(apply_cfg);
        Localizer loc(loc_cfg);
        auto s = apply_once(apply, loc, gate_snapshot(2.0), 1.0);
        expect(!s.applied && s.reason == "localizer_rejected", "localizer rejection should be surfaced");
    }

    {
        Config ok = writeback_config();
        YawCorrectionApply apply(ok);
        Localizer loc(ok);
        auto g = gate_snapshot(2.0);
        auto s = apply_once(apply, loc, g, 1.0);
        expect(!s.applied && s.reason == "correction_too_large", "rejection metrics scenario should reject");
        auto stats = apply.run_stats();
        expect(stats.attempts == 1 && stats.rejected_count == 1 && stats.safety_reject_count == 1, "rejection metrics should increment");
    }

    {
        std::ostringstream out;
        write_yaw_correction_apply_header(out);
        write_yaw_correction_apply_row(out, gate_snapshot().would_apply ? YawCorrectionApplySnapshot{} : YawCorrectionApplySnapshot{});
        std::string csv = out.str();
        expect(csv.find("timestamp_s,state,reason,attempted,applied") != std::string::npos, "apply csv header should contain leading fields");
        expect(csv.find("gate_scan_evidence_ok,gate_yaw_match_evidence_ok") != std::string::npos, "apply csv header should contain gate evidence fields");
    }

    static_safety_search();

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
