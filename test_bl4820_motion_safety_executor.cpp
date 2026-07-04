#include "config.hpp"
#include "bl4820_motion_safety_executor.hpp"

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

void expect_no_throw(const std::function<void()> &fn, const char *message) {
    try {
        fn();
    } catch (const std::exception &e) {
        std::cerr << "FAIL: unexpected throw: " << message << ": " << e.what() << "\n";
        failures++;
    }
}

robot_slamd::Config motion_config() {
    robot_slamd::Config c;
    c.motion_execution_enabled = true;
    c.motion_execution_mode = "dry_run";
    c.motion_execution_log_hz = 20.0;
    c.motion_execution_command_source = "active_scan_command";
    c.motion_execution_allow_recovery_scan_request_as_reason = true;
    c.motion_execution_require_active_scan_command = true;
    c.motion_execution_require_command_planner_verifying_or_commanding = true;
    c.motion_execution_wheel_base_m = 0.160;
    c.motion_execution_wheel_radius_m = 0.032;
    c.motion_execution_max_linear_speed_mps = 0.0;
    c.motion_execution_max_abs_yaw_rate_dps = 20.0;
    c.motion_execution_max_wheel_speed_rpm = 60.0;
    c.motion_execution_min_wheel_speed_rpm = 3.0;
    c.motion_execution_max_command_duration_s = 5.0;
    c.motion_execution_deadman_timeout_s = 0.3;
    c.motion_execution_command_stale_timeout_s = 0.5;
    c.motion_execution_require_localizer_initialized = true;
    c.motion_execution_require_supervisor_not_lost = true;
    c.motion_execution_require_tof_recent = true;
    c.motion_execution_max_tof_age_s = 0.5;
    c.motion_execution_obstacle_stop_enabled = true;
    c.motion_execution_min_front_distance_m = 0.25;
    c.motion_execution_min_side_distance_m = 0.15;
    c.motion_execution_require_encoder_feedback_recent = true;
    c.motion_execution_max_encoder_age_s = 0.5;
    c.motion_execution_current_safety_enabled = true;
    c.motion_execution_max_motor_current_a = 3.0;
    c.motion_execution_stall_detection_enabled = true;
    c.motion_execution_stall_speed_rpm = 2.0;
    c.motion_execution_stall_current_a = 2.0;
    c.motion_execution_stall_confirm_count = 3;
    c.motion_execution_dry_run_write_motor_commands = false;
    c.motion_execution_hardware_write_enabled = false;
    c.motion_execution_apply_log_enabled = true;
    return c;
}

robot_slamd::MotionSafetyExecutorInput valid_input(double t = 1.0, double yaw_dps = 20.0) {
    robot_slamd::MotionSafetyExecutorInput in;
    in.timestamp_s = t;
    in.active_scan_command.timestamp_s = t;
    in.active_scan_command.state = "COMMANDING_ROTATION";
    in.active_scan_command.reason = "commanding_rotation";
    in.active_scan_command.command_active = true;
    in.active_scan_command.would_emit_command = true;
    in.active_scan_command.zero_command = false;
    in.active_scan_command.desired_linear_speed_mps = 0.0;
    in.active_scan_command.desired_yaw_rate_dps = yaw_dps;
    in.active_scan_command.desired_yaw_rate_radps = robot_slamd::deg2rad(yaw_dps);
    in.supervisor.state = "MAPPING";
    in.recovery.state = "MAPPING_OK";
    in.localizer_initialized = true;
    in.current_pose.x = 0.0;
    in.current_pose.y = 0.0;
    in.current_pose.yaw = 0.0;
    in.linear_speed_mps = 0.0;
    in.yaw_rate_radps = 0.0;
    in.tof_recent = true;
    in.latest_tof_age_s = 0.1;
    in.front_distance_m = 1.0;
    in.left_distance_m = 1.0;
    in.right_distance_m = 1.0;
    in.encoder_feedback_recent = true;
    in.latest_encoder_age_s = 0.1;
    in.left_speed_rpm = 10.0;
    in.right_speed_rpm = -10.0;
    in.left_current_a = 0.2;
    in.right_current_a = 0.2;
    in.left_status = 0;
    in.right_status = 0;
    return in;
}

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void static_safety_search() {
    const std::string file = __FILE__;
    const size_t slash = file.find_last_of("/\\");
    const std::string dir = slash == std::string::npos ? std::string() : file.substr(0, slash + 1);
    const std::vector<std::string> production = {
        dir + "bl4820_motion_safety_executor.hpp",
        dir + "app.hpp",
    };
    const std::vector<std::string> forbidden = {
        "motor_write",
        "bl4820_write",
        "write_speed",
        "write_rpm",
        "set_motor",
        "set_pwm",
        "fr_output",
        "uart_write_motor",
        "send_motor_command",
        "apply_motor_command",
        "relocalization_writeback",
        "lost_recovery_writeback",
        "startup_relocalization_writeback",
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

    Config cfg = motion_config();
    expect_no_throw([&] { validate_config(cfg); }, "valid motion config should pass");

    BL4820MotionSafetyExecutor executor(cfg);
    expect(executor.snapshot().state == "IDLE", "default executor starts IDLE");

    Config disabled_cfg = cfg;
    disabled_cfg.motion_execution_enabled = false;
    BL4820MotionSafetyExecutor disabled(disabled_cfg);
    auto disabled_changed = disabled.update(valid_input());
    (void)disabled_changed;
    expect(disabled.snapshot().state == "DISABLED", "disabled executor stays DISABLED");

    Config invalid = cfg;
    invalid.motion_execution_mode = "write";
    expect_throw([&] { validate_config(invalid); }, "mode=write should be fatal");
    invalid = cfg;
    invalid.motion_execution_mode = "hardware";
    expect_throw([&] { validate_config(invalid); }, "mode=hardware should be fatal");
    invalid = cfg;
    invalid.motion_execution_mode = "apply";
    expect_throw([&] { validate_config(invalid); }, "mode=apply should be fatal");
    invalid = cfg;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "hardware_write_enabled=true should be fatal");
    invalid = cfg;
    invalid.motion_execution_dry_run_write_motor_commands = true;
    expect_throw([&] { validate_config(invalid); }, "dry_run_write_motor_commands=true should be fatal");
    invalid = cfg;
    invalid.motion_execution_log_hz = 0.0;
    expect_throw([&] { validate_config(invalid); }, "log_hz <= 0 should be fatal");
    invalid = cfg;
    invalid.motion_execution_wheel_base_m = 0.0;
    expect_throw([&] { validate_config(invalid); }, "wheel_base_m <= 0 should be fatal");
    invalid = cfg;
    invalid.motion_execution_wheel_radius_m = 0.0;
    expect_throw([&] { validate_config(invalid); }, "wheel_radius_m <= 0 should be fatal");
    invalid = cfg;
    invalid.motion_execution_min_wheel_speed_rpm = 10.0;
    invalid.motion_execution_max_wheel_speed_rpm = 5.0;
    expect_throw([&] { validate_config(invalid); }, "min wheel rpm above max should be fatal");

    BL4820MotionSafetyExecutor idle(cfg);
    MotionSafetyExecutorInput no_command = valid_input(1.0);
    no_command.active_scan_command = ActiveScanCommandSnapshot{};
    idle.update(no_command);
    expect(idle.snapshot().state == "IDLE" && idle.snapshot().reason == "idle", "no command should stay IDLE");

    BL4820MotionSafetyExecutor would(cfg);
    would.update(valid_input(1.0, 20.0));
    auto s = would.snapshot();
    expect(s.state == "WOULD_COMMAND" && s.would_command, "valid command should enter WOULD_COMMAND");
    const double expected_rpm = ((deg2rad(20.0) * cfg.motion_execution_wheel_base_m * 0.5) / (2.0 * kPi * cfg.motion_execution_wheel_radius_m)) * 60.0;
    expect_near(s.target_left_rpm, -expected_rpm, 1e-6, "positive yaw should target negative left rpm");
    expect_near(s.target_right_rpm, expected_rpm, 1e-6, "positive yaw should target positive right rpm");
    expect_near(s.target_left_wheel_mps, -deg2rad(20.0) * cfg.motion_execution_wheel_base_m * 0.5, 1e-9, "left wheel mps formula");
    expect_near(s.target_right_wheel_mps, deg2rad(20.0) * cfg.motion_execution_wheel_base_m * 0.5, 1e-9, "right wheel mps formula");

    BL4820MotionSafetyExecutor yaw_high(cfg);
    yaw_high.update(valid_input(1.0, 25.0));
    expect(yaw_high.snapshot().state == "BLOCKED" && yaw_high.snapshot().reason == "yaw_rate_too_high", "yaw rate above max should block");

    BL4820MotionSafetyExecutor lin_nonzero(cfg);
    auto lin = valid_input();
    lin.active_scan_command.desired_linear_speed_mps = 0.01;
    lin_nonzero.update(lin);
    expect(lin_nonzero.snapshot().state == "BLOCKED" && lin_nonzero.snapshot().reason == "linear_speed_not_zero", "linear speed should block");

    BL4820MotionSafetyExecutor no_loc(cfg);
    auto bad = valid_input();
    bad.localizer_initialized = false;
    no_loc.update(bad);
    expect(no_loc.snapshot().state == "BLOCKED" && no_loc.snapshot().reason == "localizer_not_initialized", "uninitialized localizer should block");

    BL4820MotionSafetyExecutor lost(cfg);
    bad = valid_input();
    bad.supervisor.state = "LOST";
    lost.update(bad);
    expect(lost.snapshot().state == "WOULD_ZERO" && lost.snapshot().would_zero && lost.snapshot().reason == "supervisor_lost", "LOST supervisor should zero");

    BL4820MotionSafetyExecutor tof(cfg);
    bad = valid_input();
    bad.tof_recent = false;
    tof.update(bad);
    expect(tof.snapshot().state == "BLOCKED" && tof.snapshot().reason == "tof_stale", "stale ToF should block");

    BL4820MotionSafetyExecutor front(cfg);
    bad = valid_input();
    bad.front_distance_m = 0.1;
    front.update(bad);
    expect(front.snapshot().state == "WOULD_ZERO" && front.snapshot().reason == "front_obstacle_too_close", "front obstacle should zero");

    BL4820MotionSafetyExecutor left(cfg);
    bad = valid_input();
    bad.left_distance_m = 0.1;
    left.update(bad);
    expect(left.snapshot().state == "WOULD_ZERO" && left.snapshot().reason == "left_obstacle_too_close", "left obstacle should zero");

    BL4820MotionSafetyExecutor right(cfg);
    bad = valid_input();
    bad.right_distance_m = 0.1;
    right.update(bad);
    expect(right.snapshot().state == "WOULD_ZERO" && right.snapshot().reason == "right_obstacle_too_close", "right obstacle should zero");

    BL4820MotionSafetyExecutor enc(cfg);
    bad = valid_input();
    bad.encoder_feedback_recent = false;
    enc.update(bad);
    expect(enc.snapshot().state == "BLOCKED" && enc.snapshot().reason == "encoder_stale", "stale encoder should block");

    BL4820MotionSafetyExecutor status(cfg);
    bad = valid_input();
    bad.left_status = 7;
    status.update(bad);
    expect(status.snapshot().state == "WOULD_ZERO" && status.snapshot().reason == "motor_status_error", "motor status error should zero");

    BL4820MotionSafetyExecutor current(cfg);
    bad = valid_input();
    bad.right_current_a = 4.0;
    current.update(bad);
    expect(current.snapshot().state == "WOULD_ZERO" && current.snapshot().reason == "motor_current_high", "high motor current should zero");

    BL4820MotionSafetyExecutor stall(cfg);
    bad = valid_input();
    bad.left_speed_rpm = 0.0;
    bad.right_speed_rpm = 0.0;
    bad.left_current_a = 2.5;
    bad.right_current_a = 2.5;
    stall.update(bad);
    bad.timestamp_s += 0.05;
    bad.active_scan_command.timestamp_s = bad.timestamp_s;
    stall.update(bad);
    bad.timestamp_s += 0.05;
    bad.active_scan_command.timestamp_s = bad.timestamp_s;
    stall.update(bad);
    expect(stall.snapshot().state == "WOULD_ZERO" && stall.snapshot().reason == "stall_detected", "stall should zero after confirm count");

    BL4820MotionSafetyExecutor zero_plan(cfg);
    bad = valid_input();
    bad.active_scan_command.zero_command = true;
    zero_plan.update(bad);
    expect(zero_plan.snapshot().state == "WOULD_ZERO" && zero_plan.snapshot().reason == "zero_from_planner", "planner zero should produce WOULD_ZERO");

    BL4820MotionSafetyExecutor stale(cfg);
    bad = valid_input(2.0);
    bad.active_scan_command.timestamp_s = 1.0;
    stale.update(bad);
    expect(stale.snapshot().state == "WOULD_ZERO" && stale.snapshot().reason == "command_stale", "stale command should zero");

    BL4820MotionSafetyExecutor deadman(cfg);
    deadman.update(valid_input(1.0));
    no_command = valid_input(1.5);
    no_command.active_scan_command = ActiveScanCommandSnapshot{};
    deadman.update(no_command);
    expect(deadman.snapshot().state == "WOULD_ZERO" && deadman.snapshot().reason == "deadman_timeout", "deadman timeout should zero after command disappears");

    Config limited_cfg = cfg;
    limited_cfg.motion_execution_max_wheel_speed_rpm = 5.0;
    BL4820MotionSafetyExecutor limited(limited_cfg);
    limited.update(valid_input(1.0, 20.0));
    expect(limited.snapshot().state == "WOULD_COMMAND" && limited.snapshot().reason == "wheel_speed_limited", "wheel speed limit should keep dry-run command but record reason");
    expect_near(std::fabs(limited.snapshot().target_left_rpm), 5.0, 1e-9, "left rpm should be limited");
    expect_near(std::fabs(limited.snapshot().target_right_rpm), 5.0, 1e-9, "right rpm should be limited");

    std::ostringstream header;
    write_motion_safety_executor_header(header);
    expect(header.str().find("timestamp_s,state,previous_state,reason,command_seen,would_command,would_zero") == 0, "motion CSV header should start with stable fields");
    expect(header.str().find("active_scan_command_reason,supervisor_state,recovery_state") != std::string::npos, "motion CSV header should include tail fields");

    auto stats = would.run_stats(3.0);
    expect(stats.would_command_count >= 1, "metrics should count would_command");
    expect(stats.command_seen_count >= 1, "metrics should count command seen");
    expect_near(stats.last_target_left_rpm, -expected_rpm, 1e-6, "metrics should keep last left rpm");
    expect(stats.last_reason == "would_command", "metrics should keep last reason");

    Config disabled_log = cfg;
    disabled_log.motion_execution_enabled = false;
    BL4820MotionSafetyExecutor disabled_metrics(disabled_log);
    disabled_metrics.update(valid_input());
    expect(disabled_metrics.run_stats(1.0).state_last == "DISABLED", "disabled metrics should retain default disabled state");

    static_safety_search();

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
