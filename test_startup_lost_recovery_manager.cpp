#include "config.hpp"
#include "startup_lost_recovery_manager.hpp"

#include <cmath>
#include <functional>
#include <fstream>
#include <iostream>
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

robot_slamd::Config recovery_config() {
    robot_slamd::Config c;
    c.recovery_enabled = true;
    c.recovery_mode = "observe_only";
    c.recovery_log_hz = 10.0;
    c.recovery_startup_recovery_enabled = true;
    c.recovery_lost_recovery_enabled = true;
    c.recovery_degraded_recovery_enabled = true;
    c.recovery_startup_grace_s = 3.0;
    c.recovery_lost_confirm_s = 2.0;
    c.recovery_degraded_confirm_s = 5.0;
    c.recovery_require_map_quality_not_invalid = true;
    c.recovery_require_min_known_cells = 50;
    c.recovery_require_min_occupied_cells = 5;
    c.recovery_require_tof_recent = true;
    c.recovery_max_tof_age_s = 1.0;
    c.recovery_require_localizer_initialized_for_local_recovery = true;
    c.recovery_allow_uninitialized_startup_recovery_observe_only = true;
    c.recovery_request_recovery_scan = true;
    c.recovery_recovery_scan_cooldown_s = 10.0;
    c.recovery_require_yaw_correction_stable = true;
    c.recovery_max_recent_yaw_apply_count = 3;
    c.recovery_min_post_apply_validated_count = 0;
    c.recovery_block_if_post_apply_failed_recent = true;
    return c;
}

robot_slamd::RecoveryManagerInput base_input(double t, const std::string &supervisor_state = "MAPPING") {
    robot_slamd::RecoveryManagerInput in;
    in.timestamp_s = t;
    in.localizer_initialized = true;
    in.current_pose.x = 1.0;
    in.current_pose.y = 2.0;
    in.current_pose.yaw = 0.1;
    in.linear_speed_mps = 0.0;
    in.yaw_rate_radps = 0.0;
    in.map_quality.quality_level = "GOOD";
    in.map_quality.quality_score = 0.7;
    in.map_quality.known_cells = 100;
    in.map_quality.occupied_cells = 10;
    in.supervisor.state = supervisor_state;
    in.supervisor.mapping_allowed = supervisor_state != "LOST";
    in.tof_recent = true;
    in.latest_tof_age_s = 0.1;
    in.map_known_cells = 100;
    in.map_occupied_cells = 10;
    in.latest_post_apply.state = "IDLE";
    return in;
}

void add_usable_yaw_match(robot_slamd::RecoveryManagerInput &in, uint64_t scan_id = 10, double timestamp_s = 20.0, double delta_deg = 1.5) {
    in.latest_yaw_match.scan_id = scan_id;
    in.latest_yaw_match.timestamp_s = timestamp_s;
    in.latest_yaw_match.attempted = true;
    in.latest_yaw_match.usable = true;
    in.latest_yaw_match.reason = "ok";
    in.latest_yaw_match.best_yaw_delta_deg = delta_deg;
    in.latest_yaw_match.best_score = 0.6;
    in.latest_yaw_match.score_margin = 0.2;
    in.latest_yaw_match.inlier_ratio = 0.5;
    in.latest_yaw_gate.scan_evidence_ok = true;
    in.latest_yaw_gate.yaw_match_evidence_ok = true;
    in.latest_yaw_gate.match_scan_id = scan_id;
    in.latest_yaw_gate.match_timestamp_s = timestamp_s;
}

robot_slamd::RecoveryManagerSnapshot update_once(robot_slamd::RecoveryManager &mgr, const robot_slamd::RecoveryManagerInput &in) {
    mgr.update(in);
    return mgr.snapshot();
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
        dir + "startup_lost_recovery_manager.hpp",
        dir + "app.hpp",
    };
    std::vector<std::string> forbidden = {
        "apply_pose_correction",
        "set_pose",
        "reset_pose",
        "relocalization_writeback",
        "lost_recovery_writeback",
        "startup_relocalization_writeback",
        "motor_write",
        "bl4820_write",
        "pwm",
        "fr_output",
    };
    for (const auto &path : production) {
        const std::string text = read_file(path);
        for (const auto &needle : forbidden) {
            expect(text.find(needle) == std::string::npos, ("production code should not contain " + needle).c_str());
        }
    }
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg = recovery_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid recovery config should pass");

    RecoveryManager mgr(cfg);
    expect(mgr.snapshot().state == "IDLE", "default recovery manager starts IDLE");

    Config disabled_cfg = cfg;
    disabled_cfg.recovery_enabled = false;
    RecoveryManager disabled(disabled_cfg);
    auto disabled_snap = update_once(disabled, base_input(1.0));
    expect(disabled_snap.state == "DISABLED" && disabled_snap.reason == "disabled", "disabled recovery manager stays DISABLED");

    Config invalid = cfg;
    invalid.recovery_mode = "writeback";
    expect_throw([&] { validate_config(invalid); }, "recovery mode writeback should be fatal");
    invalid = cfg;
    invalid.recovery_mode = "apply";
    expect_throw([&] { validate_config(invalid); }, "recovery mode apply should be fatal");
    invalid = cfg;
    invalid.recovery_mode = "relocalize";
    expect_throw([&] { validate_config(invalid); }, "recovery mode relocalize should be fatal");
    invalid = cfg;
    invalid.recovery_log_hz = 0.0;
    expect_throw([&] { validate_config(invalid); }, "recovery log_hz <= 0 should be fatal");
    invalid = cfg;
    invalid.recovery_startup_grace_s = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative startup_grace_s should be fatal");
    invalid = cfg;
    invalid.recovery_lost_confirm_s = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative lost_confirm_s should be fatal");
    invalid = cfg;
    invalid.recovery_degraded_confirm_s = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative degraded_confirm_s should be fatal");
    invalid = cfg;
    invalid.recovery_require_min_known_cells = -1;
    expect_throw([&] { validate_config(invalid); }, "negative min known cells should be fatal");
    invalid = cfg;
    invalid.recovery_require_min_occupied_cells = -1;
    expect_throw([&] { validate_config(invalid); }, "negative min occupied cells should be fatal");
    invalid = cfg;
    invalid.recovery_max_tof_age_s = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative max_tof_age_s should be fatal");
    invalid = cfg;
    invalid.recovery_recovery_scan_cooldown_s = -0.1;
    expect_throw([&] { validate_config(invalid); }, "negative recovery cooldown should be fatal");
    invalid = cfg;
    invalid.recovery_max_recent_yaw_apply_count = -1;
    expect_throw([&] { validate_config(invalid); }, "negative max recent yaw apply should be fatal");
    invalid = cfg;
    invalid.recovery_min_post_apply_validated_count = -1;
    expect_throw([&] { validate_config(invalid); }, "negative min post apply validated should be fatal");

    Config observe_cfg = cfg;
    observe_cfg.recovery_request_recovery_scan = false;
    RecoveryManager startup(observe_cfg);
    auto startup_snap = update_once(startup, base_input(1.0));
    expect(startup_snap.state == "STARTUP_OBSERVE" && startup_snap.startup_recovery_observe, "startup grace should enter STARTUP_OBSERVE when scan request disabled");

    RecoveryManager mapping_ok(cfg);
    auto mapping_snap = update_once(mapping_ok, base_input(10.0, "MAPPING"));
    expect(mapping_snap.state == "MAPPING_OK" && !mapping_snap.recovery_scan_recommended, "normal supervisor should enter MAPPING_OK");

    Config no_startup = cfg;
    no_startup.recovery_startup_recovery_enabled = false;
    RecoveryManager degraded(no_startup);
    auto degraded_wait = update_once(degraded, base_input(10.0, "DEGRADED"));
    expect(degraded_wait.state == "DEGRADED_OBSERVE" && degraded_wait.reason == "degraded_confirm_wait", "DEGRADED before confirm should wait");
    auto degraded_obs = update_once(degraded, base_input(16.0, "DEGRADED"));
    expect(degraded_obs.degraded_recovery_observe && degraded_obs.recovery_scan_recommended, "DEGRADED after confirm should recommend recovery scan with evidence");

    RecoveryManager lost(no_startup);
    auto lost_wait = update_once(lost, base_input(10.0, "LOST"));
    expect(lost_wait.state == "LOST_OBSERVE" && lost_wait.reason == "lost_confirm_wait", "LOST before confirm should wait");
    auto lost_obs = update_once(lost, base_input(13.0, "LOST"));
    expect(lost_obs.lost_recovery_observe && lost_obs.recovery_scan_recommended, "LOST after confirm should recommend recovery scan with evidence");

    RecoveryManager missing_map(no_startup);
    update_once(missing_map, base_input(10.0, "DEGRADED"));
    auto low_map = base_input(16.0, "DEGRADED");
    low_map.map_known_cells = 0;
    low_map.map_quality.known_cells = 0;
    auto low_map_snap = update_once(missing_map, low_map);
    expect(low_map_snap.state == "REJECTED" && low_map_snap.missing_evidence == "map", "missing map evidence should reject with map evidence");

    RecoveryManager old_tof(no_startup);
    update_once(old_tof, base_input(10.0, "DEGRADED"));
    auto stale = base_input(16.0, "DEGRADED");
    stale.tof_recent = false;
    stale.latest_tof_age_s = 5.0;
    auto stale_snap = update_once(old_tof, stale);
    expect(stale_snap.state == "REJECTED" && stale_snap.missing_evidence == "tof", "stale ToF should reject with tof evidence");

    RecoveryManager uninit_degraded(no_startup);
    update_once(uninit_degraded, base_input(10.0, "DEGRADED"));
    auto uninit = base_input(16.0, "DEGRADED");
    uninit.localizer_initialized = false;
    auto uninit_snap = update_once(uninit_degraded, uninit);
    expect(uninit_snap.state == "REJECTED" && uninit_snap.missing_evidence == "localizer", "degraded recovery should require initialized localizer");

    RecoveryManager startup_candidate(observe_cfg);
    auto startup_uninit = base_input(1.0, "MAPPING");
    startup_uninit.localizer_initialized = false;
    add_usable_yaw_match(startup_uninit, 1, 1.0);
    auto startup_candidate_snap = update_once(startup_candidate, startup_uninit);
    expect(startup_candidate_snap.candidate_seen && !startup_candidate_snap.candidate_ready, "startup uninitialized can see candidate but not ready");

    RecoveryManager unusable_match(no_startup);
    update_once(unusable_match, base_input(10.0, "DEGRADED"));
    auto no_candidate = base_input(16.0, "DEGRADED");
    no_candidate.latest_yaw_match.attempted = true;
    no_candidate.latest_yaw_match.usable = false;
    auto no_candidate_snap = update_once(unusable_match, no_candidate);
    expect(!no_candidate_snap.candidate_seen, "unusable yaw match should not be candidate_seen");

    RecoveryManager lost_untrusted(no_startup);
    update_once(lost_untrusted, base_input(10.0, "LOST"));
    auto lost_candidate = base_input(13.0, "LOST");
    lost_candidate.localizer_initialized = false;
    add_usable_yaw_match(lost_candidate, 2, 13.0);
    auto lost_candidate_snap = update_once(lost_untrusted, lost_candidate);
    expect(lost_candidate_snap.candidate_seen && !lost_candidate_snap.candidate_ready, "lost with untrusted pose should not be candidate_ready");
    expect(lost_candidate_snap.reason == "pose_reference_untrusted", "lost untrusted pose reason should be explicit");

    RecoveryManager failed_post(no_startup);
    update_once(failed_post, base_input(10.0, "DEGRADED"));
    auto failed = base_input(16.0, "DEGRADED");
    failed.latest_post_apply.state = "FAILED";
    auto failed_snap = update_once(failed_post, failed);
    expect(failed_snap.state == "REJECTED" && failed_snap.missing_evidence == "yaw_correction", "failed post-apply should block recovery");

    RecoveryManager cooldown(no_startup);
    update_once(cooldown, base_input(10.0, "DEGRADED"));
    auto first_recommend = update_once(cooldown, base_input(16.0, "DEGRADED"));
    auto second_recommend = update_once(cooldown, base_input(17.0, "DEGRADED"));
    expect(first_recommend.recovery_scan_recommended, "first eligible recovery should recommend scan");
    expect(!second_recommend.recovery_scan_recommended && second_recommend.waiting_for_recovery_scan, "cooldown should prevent repeated scan recommendation");

    RecoveryManager candidate_ready(no_startup);
    update_once(candidate_ready, base_input(10.0, "DEGRADED"));
    auto candidate = base_input(16.0, "DEGRADED");
    add_usable_yaw_match(candidate, 3, 16.0);
    auto ready = update_once(candidate_ready, candidate);
    expect(ready.candidate_seen && ready.candidate_ready && ready.state == "CANDIDATE_READY", "usable new yaw match should become candidate_ready in degraded context");

    std::ostringstream csv;
    write_recovery_manager_header(csv);
    expect(csv.str().find("timestamp_s,state,previous_state,reason,recovery_type") == 0, "recovery csv header should start with stable columns");
    expect(csv.str().find("latest_yaw_match_inlier_ratio") != std::string::npos, "recovery csv header should include yaw match fields");

    auto stats = candidate_ready.run_stats(20.0);
    expect(stats.candidate_ready_count > 0, "recovery stats should count candidate ready");
    expect(stats.last_match_scan_id == 3, "recovery stats should carry last match scan id");

    static_safety_search();

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
