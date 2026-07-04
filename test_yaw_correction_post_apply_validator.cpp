#include "config.hpp"
#include "yaw_correction_post_apply_validator.hpp"

#include <cmath>
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

void expect_near(double a, double b, double eps, const char *message) {
    if (std::fabs(a - b) > eps) {
        std::cerr << "FAIL: " << message << " got=" << a << " expected=" << b << "\n";
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

robot_slamd::Config cfg() {
    robot_slamd::Config c;
    c.yaw_correction_post_apply_enabled = true;
    c.yaw_correction_post_apply_timeout_s = 20.0;
    c.yaw_correction_post_apply_min_improvement_deg = 0.2;
    c.yaw_correction_post_apply_min_improvement_fraction_of_applied_delta = 0.3;
    c.yaw_correction_post_apply_min_absolute_improvement_deg = 0.05;
    c.yaw_correction_post_apply_max_allowed_worse_deg = 0.5;
    c.yaw_correction_post_apply_require_new_scan_id = true;
    c.yaw_correction_post_apply_require_newer_match_timestamp = true;
    c.yaw_correction_post_apply_max_post_apply_candidate_abs_deg = 10.0;
    c.yaw_correction_post_apply_log_enabled = true;
    return c;
}

robot_slamd::SparseScanYawMatchSummary yaw_match(uint64_t scan_id, double t, double candidate_deg) {
    robot_slamd::SparseScanYawMatchSummary s;
    s.scan_id = scan_id;
    s.timestamp_s = t;
    s.attempted = true;
    s.usable = true;
    s.reason = "ok";
    s.best_yaw_delta_deg = candidate_deg;
    s.best_score = 0.5;
    s.score_margin = 0.2;
    s.inlier_ratio = 0.5;
    s.curve_flat = false;
    s.multimodal = false;
    return s;
}

robot_slamd::YawCorrectionGateSnapshot gate(double candidate_deg = 5.0) {
    robot_slamd::YawCorrectionGateSnapshot g;
    g.candidate_seen = true;
    g.would_apply = true;
    g.reason = "would_apply_dry_run";
    g.candidate_yaw_delta_deg = candidate_deg;
    g.scan_evidence_ok = true;
    g.yaw_match_evidence_ok = true;
    g.match_scan_id = 10;
    g.match_timestamp_s = 1.0;
    return g;
}

robot_slamd::YawCorrectionPostApplyInput input(double t) {
    robot_slamd::YawCorrectionPostApplyInput in;
    in.timestamp_s = t;
    in.latest_gate = gate();
    in.latest_yaw_match = yaw_match(0, 0.0, 0.0);
    in.supervisor.state = "MAPPING";
    return in;
}

robot_slamd::YawCorrectionPostApplySnapshot start_wait(robot_slamd::YawCorrectionPostApplyValidator &v,
                                                       double old_candidate_deg = 5.0,
                                                       double applied_delta_deg = 1.0,
                                                       uint64_t applied_scan_id = 10,
                                                       double applied_match_timestamp_s = 1.0) {
    auto in = input(1.0);
    in.apply_happened = true;
    in.applied_scan_id = applied_scan_id;
    in.applied_match_timestamp_s = applied_match_timestamp_s;
    in.applied_delta_deg = applied_delta_deg;
    in.latest_gate = gate(old_candidate_deg);
    v.update(in);
    return v.snapshot();
}
} // namespace

int main() {
    using namespace robot_slamd;
    Config c = cfg();
    try { validate_config(c); } catch (const std::exception &e) { std::cerr << "FAIL: valid post apply config threw: " << e.what() << "\n"; failures++; }
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_timeout_s = 0.0; validate_config(bad); }, "timeout <= 0 should be fatal");
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_min_improvement_deg = -0.1; validate_config(bad); }, "negative legacy min improvement should be fatal");
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_min_improvement_fraction_of_applied_delta = -0.1; validate_config(bad); }, "negative applied delta fraction should be fatal");
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_min_absolute_improvement_deg = -0.1; validate_config(bad); }, "negative absolute improvement should be fatal");
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_max_allowed_worse_deg = -0.1; validate_config(bad); }, "negative max worse should be fatal");
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_max_post_apply_candidate_abs_deg = 0.0; validate_config(bad); }, "non-positive max candidate should be fatal");

    {
        Config disabled = c;
        disabled.yaw_correction_post_apply_enabled = false;
        YawCorrectionPostApplyValidator v(disabled);
        auto s = v.snapshot();
        expect(s.state == "DISABLED" && s.reason == "disabled", "disabled validator should start disabled");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        auto s = start_wait(v);
        expect(s.state == "WAITING_FOR_NEXT_MATCH" && s.reason == "waiting_for_next_match", "apply should enter waiting");
        expect_near(s.expected_min_improvement_deg, 0.3, 1e-12, "expected improvement should be derived from applied delta");
        auto same = input(2.0);
        same.latest_yaw_match = yaw_match(10, 2.0, 4.0);
        v.update(same);
        expect(v.snapshot().reason == "waiting_for_new_scan_id", "same scan id should be ignored while waiting");
        expect(v.run_stats().validated_count == 0 && v.run_stats().suspect_count == 0 && v.run_stats().failed_count == 0, "waiting same scan should not increment terminal counters");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto same_ts = input(2.0);
        same_ts.latest_yaw_match = yaw_match(11, 1.0, 4.0);
        v.update(same_ts);
        expect(v.snapshot().state == "WAITING_FOR_NEXT_MATCH" && v.snapshot().reason == "waiting_for_newer_match_timestamp", "same timestamp should keep waiting");
        auto older_ts = input(2.5);
        older_ts.latest_yaw_match = yaw_match(12, 0.5, 4.0);
        v.update(older_ts);
        expect(v.snapshot().state == "WAITING_FOR_NEXT_MATCH" && v.snapshot().reason == "waiting_for_newer_match_timestamp", "older timestamp should keep waiting");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v, 5.0, 1.0);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(11, 2.0, 4.8);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "SUSPECT" && s.reason == "insufficient_improvement", "0.2 deg improvement should be insufficient for 1 deg applied delta");
        expect_near(s.expected_min_improvement_deg, 0.3, 1e-12, "expected min improvement should be 30 percent of applied delta");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v, 5.0, 1.0);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(12, 2.0, 4.6);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "VALIDATED" && s.validated && s.reason == "validated", "0.4 deg improvement should validate for 1 deg applied delta");
        expect(v.run_stats().validated_count == 1, "validated metric should increment");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v, 5.0, 0.2);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(13, 2.0, 4.95);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "SUSPECT" && s.reason == "insufficient_improvement", "small applied delta should still require fraction/min improvement");
        expect_near(s.expected_min_improvement_deg, 0.06, 1e-12, "expected min improvement should use max(abs min, fraction of applied delta)");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v, 5.0, 1.0);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(14, 2.0, 5.7);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "SUSPECT" && s.reason == "candidate_worse", "worse candidate beyond threshold should be suspect");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v, 5.0, 1.0);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(15, 2.0, 12.0);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "FAILED" && s.reason == "post_candidate_too_large", "post candidate above max should fail");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(16, 2.0, 4.0);
        next.latest_yaw_match.usable = false;
        v.update(next);
        expect(v.snapshot().state == "SUSPECT" && v.snapshot().reason == "post_match_not_usable", "not usable post match should be suspect");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(17, 2.0, 4.0);
        next.latest_yaw_match.curve_flat = true;
        v.update(next);
        expect(v.snapshot().state == "FAILED" && v.snapshot().reason == "post_match_curve_flat", "flat post match should not validate");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(18, 2.0, 4.0);
        next.latest_yaw_match.multimodal = true;
        v.update(next);
        expect(v.snapshot().state == "FAILED" && v.snapshot().reason == "post_match_multimodal", "multimodal post match should not validate");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto timeout = input(30.0);
        v.update(timeout);
        auto s = v.snapshot();
        expect(s.state == "TIMEOUT" && s.timeout, "timeout should be reported");
        expect(v.run_stats().timeout_count == 1, "timeout metric should increment");
    }
    {
        std::ostringstream out;
        write_yaw_correction_post_apply_header(out);
        write_yaw_correction_post_apply_row(out, YawCorrectionPostApplySnapshot{});
        std::string csv = out.str();
        expect(csv.find("timestamp_s,state,reason,applied_scan_id") != std::string::npos, "post apply csv header should be stable");
        expect(csv.find("expected_min_improvement_deg,applied_match_timestamp_s,new_match_timestamp_s") != std::string::npos, "post apply csv header should append 4D+ fields");
    }

    if (failures) {
        std::cerr << failures << " failure(s)\n";
        return 1;
    }
    return 0;
}
