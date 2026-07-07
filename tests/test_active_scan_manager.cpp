#include "robot_slamd/config/config.hpp"
#include "robot_slamd/active_scan/active_scan_manager.hpp"

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

robot_slamd::Config test_config() {
    robot_slamd::Config c;
    c.active_scan_log_hz = 10.0;
    c.active_scan_request_hold_s = 0.5;
    c.active_scan_min_interval_s = 1.0;
    c.active_scan_cooldown_s = 1.0;
    c.active_scan_max_pending_s = 5.0;
    c.active_scan_recommended_scan_angle_deg = 30.0;
    c.active_scan_min_useful_scan_angle_deg = 10.0;
    c.active_scan_complete_scan_angle_deg = 20.0;
    c.active_scan_min_tof_valid_ratio_for_scan = 0.20;
    c.active_scan_min_valid_tof_routes_for_scan = 1;
    c.active_scan_max_linear_speed_for_scan_mps = 0.05;
    c.active_scan_min_yaw_rate_for_observed_scan_dps = 5.0;
    c.active_scan_max_yaw_rate_for_observed_scan_dps = 90.0;
    c.active_scan_observe_natural_rotation = true;
    c.active_scan_require_supervisor_recommendation = true;
    return c;
}

robot_slamd::MappingSupervisorSnapshot supervisor_snapshot(const std::string &state, const std::string &reason,
                                                            bool recommended, int routes, double valid_ratio) {
    robot_slamd::MappingSupervisorSnapshot s;
    s.timestamp_s = 0.0;
    s.state = state;
    s.reason = reason;
    s.quality_score = state == "LOST" ? 0.05 : (state == "DEGRADED" ? 0.20 : 0.35);
    s.quality_level = recommended ? "LOW" : "GOOD";
    s.mapping_allowed = true;
    s.active_scan_recommended = recommended;
    s.degraded = state == "DEGRADED";
    s.lost = state == "LOST";
    s.tof_valid_ratio = valid_ratio;
    s.tof_reject_ratio = 1.0 - valid_ratio;
    s.map_update_rate_hz = reason.find("low_map_update") != std::string::npos ? 0.1 : 2.0;
    s.occupied_cells = 20;
    s.known_cells = 100;
    s.valid_tof_routes = routes;
    return s;
}

robot_slamd::ActiveScanInput input(double t, const robot_slamd::MappingSupervisorSnapshot &sup) {
    robot_slamd::ActiveScanInput in;
    in.timestamp_s = t;
    in.supervisor = sup;
    in.map_quality.quality_score = sup.quality_score;
    in.map_quality.quality_level = sup.quality_level;
    in.map_quality.tof_valid_ratio = sup.tof_valid_ratio;
    in.map_quality.map_update_rate_hz = sup.map_update_rate_hz;
    in.linear_speed_mps = 0.0;
    in.yaw_rate_radps = 0.0;
    in.localization_valid = true;
    return in;
}

robot_slamd::ActiveScanInput active_input(double t) {
    return input(t, supervisor_snapshot("ACTIVE_SCAN_RECOMMENDED", "low_quality_active_scan", true, 2, 0.60));
}

void drive_to_pending(robot_slamd::ActiveScanManager &mgr) {
    mgr.update(active_input(0.0));
    mgr.update(active_input(0.6));
}

void drive_to_would_scan(robot_slamd::ActiveScanManager &mgr) {
    drive_to_pending(mgr);
    mgr.update(active_input(0.7));
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg = test_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid active_scan config should pass");

    ActiveScanManager mgr(cfg);
    expect(mgr.snapshot().state == "IDLE", "default active scan state should be IDLE");
    mgr.update(active_input(0.0));
    expect(mgr.snapshot().state == "ARMED", "ACTIVE_SCAN_RECOMMENDED should arm active scan");
    expect(mgr.snapshot().request_type == "LOW_QUALITY_SCAN", "low quality request type should be selected");
    mgr.update(active_input(0.4));
    expect(mgr.snapshot().state == "ARMED", "request hold should filter short requests");
    mgr.update(active_input(0.6));
    expect(mgr.snapshot().state == "PENDING", "request hold should advance ARMED to PENDING");
    mgr.update(active_input(0.7));
    expect(mgr.snapshot().state == "WOULD_SCAN", "valid pending scan should become WOULD_SCAN");
    expect(mgr.snapshot().would_start_scan, "WOULD_SCAN should set would_start_scan only as a field");

    ActiveScanManager blocked_linear(cfg);
    drive_to_pending(blocked_linear);
    auto fast = active_input(0.7);
    fast.linear_speed_mps = 0.20;
    blocked_linear.update(fast);
    expect(blocked_linear.snapshot().state == "BLOCKED", "high linear speed should block scan");
    expect(blocked_linear.snapshot().reason == "blocked_linear_speed", "linear speed block reason should be logged");
    blocked_linear.update(active_input(0.8));
    expect(blocked_linear.snapshot().state == "PENDING", "blocked scan should return to pending when speed recovers");

    ActiveScanManager blocked_tof(cfg);
    drive_to_pending(blocked_tof);
    auto low_tof = active_input(0.7);
    low_tof.supervisor.tof_valid_ratio = 0.10;
    blocked_tof.update(low_tof);
    expect(blocked_tof.snapshot().state == "BLOCKED", "low ToF valid ratio should block scan");
    expect(blocked_tof.snapshot().reason == "blocked_low_tof_valid_ratio", "low ToF ratio block reason should be logged");

    ActiveScanManager observe(cfg);
    drive_to_would_scan(observe);
    auto rot_start = active_input(0.8);
    rot_start.yaw_rate_radps = deg2rad(20.0);
    rot_start.yaw_rad = deg2rad(179.0);
    observe.update(rot_start);
    expect(observe.snapshot().state == "OBSERVING_ROTATION", "natural yaw rate should start observing rotation");
    auto wrap = active_input(0.9);
    wrap.yaw_rate_radps = deg2rad(20.0);
    wrap.yaw_rad = deg2rad(-179.0);
    observe.update(wrap);
    expect(observe.snapshot().observed_yaw_delta_deg > 1.5 && observe.snapshot().observed_yaw_delta_deg < 2.5, "yaw wrap-around 179 to -179 should count about 2 degrees");
    auto useful = active_input(1.0);
    useful.yaw_rate_radps = deg2rad(20.0);
    useful.yaw_rad = deg2rad(-168.0);
    observe.update(useful);
    expect(observe.snapshot().useful_scan_observed, "observed angle above min useful should set useful_scan_observed");
    auto complete = active_input(1.1);
    complete.yaw_rate_radps = deg2rad(20.0);
    complete.yaw_rad = deg2rad(-158.0);
    observe.update(complete);
    expect(observe.snapshot().state == "COMPLETED", "observed angle above complete threshold should complete scan session");
    observe.update(active_input(1.2));
    expect(observe.snapshot().state == "COOLDOWN", "COMPLETED should enter COOLDOWN");
    observe.update(active_input(1.4));
    expect(observe.snapshot().state == "COOLDOWN", "cooldown should prevent immediate repeated request");
    observe.update(active_input(2.9));
    expect(observe.snapshot().state == "IDLE", "cooldown expiry should return to IDLE before new request");

    ActiveScanManager cleared(cfg);
    cleared.update(active_input(0.0));
    auto no_req = input(0.2, supervisor_snapshot("SENSOR_CHECK", "sensor_check", false, 2, 0.80));
    cleared.update(no_req);
    expect(cleared.snapshot().state == "IDLE", "request disappearance should return to IDLE");

    ActiveScanManager degraded(cfg);
    degraded.update(input(0.0, supervisor_snapshot("DEGRADED", "quality_degraded", false, 1, 0.40)));
    expect(degraded.snapshot().request_type == "DEGRADED_SCAN", "DEGRADED should request degraded scan when ToF exists");

    ActiveScanManager lost(cfg);
    lost.update(input(0.0, supervisor_snapshot("LOST", "quality_lost", false, 0, 0.0)));
    expect(lost.snapshot().request_type == "LOST_RECOVERY_SCAN", "LOST should request lost recovery scan");

    ActiveScanManager single(cfg);
    single.update(input(0.0, supervisor_snapshot("ACTIVE_SCAN_RECOMMENDED", "single_tof_route_active_scan", true, 1, 0.50)));
    expect(single.snapshot().request_type == "SINGLE_TOF_ROUTE_SCAN", "single ToF route should request SINGLE_TOF_ROUTE_SCAN");

    Config disabled = cfg;
    disabled.active_scan_enabled = false;
    expect_no_throw([&] { validate_config(disabled); }, "active_scan.enabled=false should validate");
    ActiveScanManager disabled_mgr(disabled);
    disabled_mgr.update(active_input(0.0));
    expect(disabled_mgr.snapshot().state == "IDLE", "disabled active scan manager should stay IDLE");

    Config bad_log = cfg;
    bad_log.active_scan_log_hz = 0.0;
    expect_throw([&] { validate_config(bad_log); }, "active_scan.log_hz <= 0 should be invalid");
    Config bad_time = cfg;
    bad_time.active_scan_cooldown_s = -0.1;
    expect_throw([&] { validate_config(bad_time); }, "negative active_scan time should be invalid");
    Config bad_angle = cfg;
    bad_angle.active_scan_recommended_scan_angle_deg = -1.0;
    expect_throw([&] { validate_config(bad_angle); }, "negative active_scan angle should be invalid");
    Config bad_complete = cfg;
    bad_complete.active_scan_complete_scan_angle_deg = 5.0;
    bad_complete.active_scan_min_useful_scan_angle_deg = 10.0;
    expect_throw([&] { validate_config(bad_complete); }, "complete angle below useful angle should be invalid");
    Config bad_recommended = cfg;
    bad_recommended.active_scan_recommended_scan_angle_deg = 5.0;
    bad_recommended.active_scan_min_useful_scan_angle_deg = 10.0;
    expect_throw([&] { validate_config(bad_recommended); }, "recommended angle below useful angle should be invalid");
    Config bad_ratio = cfg;
    bad_ratio.active_scan_min_tof_valid_ratio_for_scan = 1.2;
    expect_throw([&] { validate_config(bad_ratio); }, "active_scan ratio above 1 should be invalid");
    Config bad_routes = cfg;
    bad_routes.active_scan_min_valid_tof_routes_for_scan = -1;
    expect_throw([&] { validate_config(bad_routes); }, "negative active_scan min routes should be invalid");
    Config bad_yaw = cfg;
    bad_yaw.active_scan_min_yaw_rate_for_observed_scan_dps = 20.0;
    bad_yaw.active_scan_max_yaw_rate_for_observed_scan_dps = 10.0;
    expect_throw([&] { validate_config(bad_yaw); }, "active_scan max yaw rate below min should be invalid");

    if (failures) {
        std::cerr << failures << " test failures\n";
        return 1;
    }
    std::cout << "test_active_scan_manager ok\n";
    return 0;
}
