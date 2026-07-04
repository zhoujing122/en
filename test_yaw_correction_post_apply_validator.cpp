#include "config.hpp"
#include "yaw_correction_post_apply_validator.hpp"

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

robot_slamd::Config cfg() {
    robot_slamd::Config c;
    c.yaw_correction_post_apply_enabled = true;
    c.yaw_correction_post_apply_timeout_s = 20.0;
    c.yaw_correction_post_apply_min_improvement_deg = 0.2;
    c.yaw_correction_post_apply_max_allowed_worse_deg = 0.5;
    c.yaw_correction_post_apply_require_new_scan_id = true;
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

robot_slamd::YawCorrectionGateSnapshot gate(double candidate_deg = 2.0) {
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

robot_slamd::YawCorrectionPostApplySnapshot start_wait(robot_slamd::YawCorrectionPostApplyValidator &v) {
    auto in = input(1.0);
    in.apply_happened = true;
    in.applied_scan_id = 10;
    in.applied_match_timestamp_s = 1.0;
    in.applied_delta_deg = 0.3;
    v.update(in);
    return v.snapshot();
}
} // namespace

int main() {
    using namespace robot_slamd;
    Config c = cfg();
    try { validate_config(c); } catch (const std::exception &e) { std::cerr << "FAIL: valid post apply config threw: " << e.what() << "\n"; failures++; }
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_timeout_s = 0.0; validate_config(bad); }, "timeout <= 0 should be fatal");
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_min_improvement_deg = -0.1; validate_config(bad); }, "negative min improvement should be fatal");
    expect_throw([&] { Config bad = c; bad.yaw_correction_post_apply_max_allowed_worse_deg = -0.1; validate_config(bad); }, "negative max worse should be fatal");

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
        auto same = input(2.0);
        same.latest_yaw_match = yaw_match(10, 2.0, 0.5);
        v.update(same);
        expect(v.snapshot().reason == "waiting_for_new_scan_id", "same scan id should be ignored while waiting");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(11, 2.0, 1.5);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "VALIDATED" && s.validated && s.improvement_deg >= 0.2, "smaller next candidate should validate");
        expect(v.run_stats().validated_count == 1, "validated metric should increment");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(12, 2.0, 2.4);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "SUSPECT" && s.suspect && s.reason == "insufficient_improvement", "not enough improvement should be suspect");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(13, 2.0, 3.0);
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "SUSPECT" && s.reason == "candidate_worse", "worse candidate should be suspect");
    }
    {
        YawCorrectionPostApplyValidator v(c);
        start_wait(v);
        auto next = input(2.0);
        next.latest_yaw_match = yaw_match(14, 2.0, 1.0);
        next.latest_yaw_match.multimodal = true;
        v.update(next);
        auto s = v.snapshot();
        expect(s.state == "FAILED" && s.failed && s.reason == "post_match_multimodal", "multimodal post match should fail/suspect without rollback");
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
    }

    if (failures) {
        std::cerr << failures << " failure(s)\n";
        return 1;
    }
    return 0;
}
