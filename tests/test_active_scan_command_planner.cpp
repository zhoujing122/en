#include "robot_slamd/config/config.hpp"
#include "robot_slamd/active_scan/active_scan_command_planner.hpp"

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
    c.active_scan_execution_log_hz = 20.0;
    c.active_scan_execution_precheck_hold_s = 0.5;
    c.active_scan_execution_command_timeout_s = 1.0;
    c.active_scan_execution_cooldown_s = 1.0;
    c.active_scan_execution_target_scan_angle_deg = 30.0;
    c.active_scan_execution_complete_scan_angle_deg = 20.0;
    c.active_scan_execution_min_useful_scan_angle_deg = 10.0;
    c.active_scan_execution_target_yaw_rate_dps = 20.0;
    c.active_scan_execution_min_observed_yaw_rate_dps = 3.0;
    c.active_scan_execution_max_observed_yaw_rate_dps = 60.0;
    c.active_scan_execution_max_linear_speed_mps = 0.05;
    c.active_scan_execution_emit_zero_command_on_stop = true;
    return c;
}

robot_slamd::ActiveScanCommandInput command_input(double t) {
    robot_slamd::ActiveScanCommandInput in;
    in.timestamp_s = t;
    in.active_scan.timestamp_s = t;
    in.active_scan.state = "WOULD_SCAN";
    in.active_scan.request_type = "LOW_QUALITY_SCAN";
    in.active_scan.reason = "would_start_scan";
    in.active_scan.scan_recommended = true;
    in.active_scan.would_start_scan = true;
    in.active_scan.recommended_scan_angle_deg = 30.0;
    in.active_scan.quality_score = 0.35;
    in.active_scan.supervisor_state = "ACTIVE_SCAN_RECOMMENDED";
    in.supervisor.timestamp_s = t;
    in.supervisor.state = "ACTIVE_SCAN_RECOMMENDED";
    in.supervisor.reason = "low_quality_active_scan";
    in.supervisor.quality_score = 0.35;
    in.map_quality.timestamp_s = t;
    in.map_quality.quality_score = 0.35;
    in.yaw_rad = 0.0;
    in.linear_speed_mps = 0.0;
    in.yaw_rate_radps = 0.0;
    in.localization_valid = true;
    return in;
}

void drive_to_verifying(robot_slamd::ActiveScanCommandPlanner &planner) {
    auto in0 = command_input(0.0);
    in0.yaw_rad = robot_slamd::deg2rad(179.0);
    planner.update(in0);
    auto in1 = command_input(0.6);
    in1.yaw_rad = robot_slamd::deg2rad(179.0);
    planner.update(in1);
    auto in2 = command_input(0.7);
    in2.yaw_rad = robot_slamd::deg2rad(179.0);
    planner.update(in2);
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config cfg = test_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid active_scan_execution config should pass");

    ActiveScanCommandPlanner planner(cfg);
    expect(planner.snapshot().state == "IDLE", "default command planner state should be IDLE");
    planner.update(command_input(0.0));
    expect(planner.snapshot().state == "PRECHECK", "would_start active scan should enter PRECHECK");
    planner.update(command_input(0.4));
    expect(planner.snapshot().state == "PRECHECK", "precheck should hold before command");
    planner.update(command_input(0.6));
    auto cmd = planner.snapshot();
    expect(cmd.state == "COMMANDING_ROTATION", "precheck hold should enter COMMANDING_ROTATION");
    expect(cmd.command_active && cmd.would_emit_command, "COMMANDING_ROTATION should emit dry-run command intent");
    expect(cmd.desired_linear_speed_mps == 0.0, "dry-run command linear speed should be zero");
    expect(std::fabs(cmd.desired_yaw_rate_dps - 20.0) < 1e-9, "dry-run command yaw rate dps should match target");
    expect(std::fabs(cmd.desired_yaw_rate_radps - deg2rad(20.0)) < 1e-9, "dry-run command yaw rate radps should match target");
    planner.update(command_input(0.7));
    expect(planner.snapshot().state == "VERIFYING_ROTATION", "COMMANDING_ROTATION should advance to VERIFYING_ROTATION");

    ActiveScanCommandPlanner wrap(cfg);
    drive_to_verifying(wrap);
    auto w = command_input(0.8);
    w.yaw_rad = deg2rad(-179.0);
    w.yaw_rate_radps = deg2rad(20.0);
    wrap.update(w);
    expect(wrap.snapshot().observed_yaw_delta_deg > 1.5 && wrap.snapshot().observed_yaw_delta_deg < 2.5, "yaw wrap 179 to -179 should count about 2 degrees");
    auto done = command_input(0.9);
    done.yaw_rad = deg2rad(-158.0);
    done.yaw_rate_radps = deg2rad(20.0);
    wrap.update(done);
    auto completed = wrap.snapshot();
    expect(completed.state == "COMPLETED", "observed yaw above complete threshold should complete");
    expect(completed.zero_command, "COMPLETED should record zero command intent");
    expect(completed.completed, "completed flag should be set");
    wrap.update(command_input(1.0));
    expect(wrap.snapshot().state == "COOLDOWN", "COMPLETED should enter COOLDOWN");
    wrap.update(command_input(2.2));
    expect(wrap.snapshot().state == "IDLE", "COOLDOWN should return to IDLE after cooldown_s");

    ActiveScanCommandPlanner timeout(cfg);
    drive_to_verifying(timeout);
    timeout.update(command_input(1.8));
    expect(timeout.snapshot().state == "ABORTED", "timeout should abort verifying scan");
    expect(timeout.snapshot().reason == "aborted_timeout", "timeout abort reason should be logged");
    expect(timeout.snapshot().zero_command, "ABORTED should record zero command intent");

    ActiveScanCommandPlanner loc_invalid(cfg);
    drive_to_verifying(loc_invalid);
    auto bad_loc = command_input(0.8);
    bad_loc.localization_valid = false;
    loc_invalid.update(bad_loc);
    expect(loc_invalid.snapshot().state == "ABORTED", "localization invalid should abort active scan command");
    expect(loc_invalid.snapshot().reason == "aborted_localization_invalid", "localization invalid abort reason should be logged");

    ActiveScanCommandPlanner fast_abort(cfg);
    drive_to_verifying(fast_abort);
    auto fast = command_input(0.8);
    fast.linear_speed_mps = 0.2;
    fast_abort.update(fast);
    expect(fast_abort.snapshot().state == "ABORTED", "linear speed too high during verifying should abort");
    expect(fast_abort.snapshot().reason == "aborted_linear_speed", "linear speed abort reason should be logged");

    ActiveScanCommandPlanner blocked(cfg);
    auto no_would = command_input(0.0);
    no_would.active_scan.would_start_scan = false;
    blocked.update(no_would);
    expect(blocked.snapshot().state == "BLOCKED", "scan recommendation without would_start should be BLOCKED");
    expect(blocked.snapshot().reason == "blocked_no_would_start", "blocked no would_start reason should be logged");

    Config disabled = cfg;
    disabled.active_scan_execution_enabled = false;
    expect_no_throw([&] { validate_config(disabled); }, "active_scan_execution.enabled=false should validate");
    ActiveScanCommandPlanner disabled_planner(disabled);
    disabled_planner.update(command_input(0.0));
    expect(disabled_planner.snapshot().state == "DISABLED", "disabled planner should stay DISABLED");

    Config disabled_mode = cfg;
    disabled_mode.active_scan_execution_mode = "disabled";
    expect_no_throw([&] { validate_config(disabled_mode); }, "active_scan_execution.mode=disabled should validate");
    ActiveScanCommandPlanner disabled_mode_planner(disabled_mode);
    disabled_mode_planner.update(command_input(0.0));
    expect(disabled_mode_planner.snapshot().state == "DISABLED", "mode=disabled should stay DISABLED");

    Config hardware = cfg;
    hardware.active_scan_execution_mode = "hardware";
    expect_throw([&] { validate_config(hardware); }, "active_scan_execution.mode=hardware should be invalid in stage4");
    Config hw_write = cfg;
    hw_write.active_scan_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(hw_write); }, "hardware_write_enabled=true should be invalid in stage4");
    Config bad_log = cfg;
    bad_log.active_scan_execution_log_hz = 0.0;
    expect_throw([&] { validate_config(bad_log); }, "active_scan_execution.log_hz <= 0 should be invalid");
    Config bad_time = cfg;
    bad_time.active_scan_execution_precheck_hold_s = -0.1;
    expect_throw([&] { validate_config(bad_time); }, "negative active_scan_execution time should be invalid");
    Config bad_angle = cfg;
    bad_angle.active_scan_execution_target_scan_angle_deg = -1.0;
    expect_throw([&] { validate_config(bad_angle); }, "negative active_scan_execution angle should be invalid");
    Config bad_complete = cfg;
    bad_complete.active_scan_execution_complete_scan_angle_deg = 5.0;
    bad_complete.active_scan_execution_min_useful_scan_angle_deg = 10.0;
    expect_throw([&] { validate_config(bad_complete); }, "complete angle below useful angle should be invalid");
    Config bad_target = cfg;
    bad_target.active_scan_execution_target_scan_angle_deg = 5.0;
    bad_target.active_scan_execution_min_useful_scan_angle_deg = 10.0;
    expect_throw([&] { validate_config(bad_target); }, "target angle below useful angle should be invalid");
    Config bad_yaw_rate = cfg;
    bad_yaw_rate.active_scan_execution_target_yaw_rate_dps = 0.0;
    expect_throw([&] { validate_config(bad_yaw_rate); }, "target yaw rate <= 0 should be invalid");
    Config bad_observed = cfg;
    bad_observed.active_scan_execution_min_observed_yaw_rate_dps = 20.0;
    bad_observed.active_scan_execution_max_observed_yaw_rate_dps = 10.0;
    expect_throw([&] { validate_config(bad_observed); }, "max observed yaw rate below min should be invalid");
    Config bad_mode = cfg;
    bad_mode.active_scan_execution_mode = "closed_loop";
    expect_throw([&] { validate_config(bad_mode); }, "unsupported active_scan_execution mode should be invalid");

    if (failures) {
        std::cerr << failures << " test failures\n";
        return 1;
    }
    std::cout << "test_active_scan_command_planner ok\n";
    return 0;
}
