#include "config.hpp"
#include "bl4820_motion_safety_executor.hpp"

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
    c.motion_execution_write_mode_acknowledgement = "";
    c.motion_execution_required_write_mode_acknowledgement = "I_UNDERSTAND_LOW_SPEED_MOTOR_WRITE_RISK";
    c.motion_execution_max_allowed_write_yaw_rate_dps = 10.0;
    c.motion_execution_max_allowed_write_duration_s = 3.0;
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
        "pwm",
        "uart_write_motor",
        "send_motor_command",
        "apply_motor_command",
        "apply_pose_correction",
        "set_pose",
        "reset_pose",
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
    disabled.update(valid_input());
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
    invalid.motion_execution_max_allowed_write_yaw_rate_dps = 0.0;
    expect_throw([&] { validate_config(invalid); }, "max_allowed_write_yaw_rate_dps <= 0 should be fatal");
    invalid = cfg;
    invalid.motion_execution_max_allowed_write_duration_s = 0.0;
    expect_throw([&] { validate_config(invalid); }, "max_allowed_write_duration_s <= 0 should be fatal");
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

    BL4820MotionSafetyExecutor precheck_blocked(cfg);
    auto pre = valid_input(1.0);
    pre.active_scan_command.state = "PRECHECK";
    precheck_blocked.update(pre);
    expect(precheck_blocked.snapshot().state == "BLOCKED" && precheck_blocked.snapshot().reason == "command_state_not_allowed", "require verifying/commanding should block PRECHECK");

    Config allow_pre_cfg = cfg;
    allow_pre_cfg.motion_execution_require_command_planner_verifying_or_commanding = false;
    BL4820MotionSafetyExecutor precheck_allowed(allow_pre_cfg);
    precheck_allowed.update(pre);
    expect(precheck_allowed.snapshot().state == "WOULD_COMMAND", "PRECHECK may pass when strict command state gate disabled");

    BL4820MotionSafetyExecutor commanding(cfg);
    commanding.update(valid_input(1.0));
    expect(commanding.snapshot().state == "WOULD_COMMAND", "COMMANDING_ROTATION should pass state gate");
    BL4820MotionSafetyExecutor verifying(cfg);
    auto verify = valid_input(1.0);
    verify.active_scan_command.state = "VERIFYING_ROTATION";
    verifying.update(verify);
    expect(verifying.snapshot().state == "WOULD_COMMAND", "VERIFYING_ROTATION should pass state gate");

    BL4820MotionSafetyExecutor would(cfg);
    would.update(valid_input(1.0, 20.0));
    auto s = would.snapshot();
    expect(s.state == "WOULD_COMMAND" && s.would_command, "valid command should enter WOULD_COMMAND");
    const double expected_rpm = ((deg2rad(20.0) * cfg.motion_execution_wheel_base_m * 0.5) / (2.0 * kPi * cfg.motion_execution_wheel_radius_m)) * 60.0;
    expect_near(s.target_left_rpm, -expected_rpm, 1e-6, "positive yaw should target negative left rpm");
    expect_near(s.target_right_rpm, expected_rpm, 1e-6, "positive yaw should target positive right rpm");
    expect(s.command_session_id == 1, "first command should start session id 1");
    expect(s.command_duration_ok, "initial session duration should be ok");

    auto cont = valid_input(1.2, 20.0);
    would.update(cont);
    expect(would.snapshot().state == "WOULD_COMMAND", "duration under max should continue would_command");
    expect(would.snapshot().command_session_id == 1, "same continuous command should keep session id");
    expect_near(would.snapshot().command_session_duration_s, 0.2, 1e-9, "session duration should accumulate");

    Config dur_cfg = cfg;
    dur_cfg.motion_execution_max_command_duration_s = 0.5;
    dur_cfg.motion_execution_deadman_timeout_s = 2.0;
    BL4820MotionSafetyExecutor duration(dur_cfg);
    duration.update(valid_input(1.0));
    duration.update(valid_input(1.7));
    expect(duration.snapshot().state == "WOULD_ZERO" && duration.snapshot().reason == "command_duration_exceeded", "duration over max should force zero");
    expect(!duration.snapshot().command_duration_ok, "duration exceeded snapshot should mark duration not ok");
    uint64_t first_session = duration.snapshot().command_session_id;
    duration.update(valid_input(2.0));
    expect(duration.snapshot().command_session_id == first_session + 1, "new command after timeout should get new session id");

    BL4820MotionSafetyExecutor lost_command(cfg);
    lost_command.update(valid_input(1.0));
    no_command = valid_input(1.1);
    no_command.active_scan_command = ActiveScanCommandSnapshot{};
    lost_command.update(no_command);
    expect(lost_command.snapshot().state == "WOULD_ZERO" && lost_command.snapshot().reason == "command_lost_zero", "command disappearance after would_command should emit zero first");
    expect_near(lost_command.snapshot().target_left_rpm, 0.0, 1e-12, "lost command zero left rpm");
    lost_command.update(no_command);
    expect(lost_command.snapshot().state == "IDLE", "after zero frame no command can return to IDLE");

    for (const std::string state : {"COMPLETED", "ABORTED", "BLOCKED", "FAULT"}) {
        BL4820MotionSafetyExecutor stop(cfg);
        stop.update(valid_input(1.0));
        auto stop_in = valid_input(1.1);
        stop_in.active_scan_command.state = state;
        stop_in.active_scan_command.command_active = false;
        stop_in.active_scan_command.would_emit_command = false;
        stop.update(stop_in);
        expect(stop.snapshot().state == "WOULD_ZERO" && stop.snapshot().reason == "command_completed_zero", "stop state should zero after command");
    }

    BL4820MotionSafetyExecutor stale_bad_ts(cfg);
    auto bad = valid_input();
    bad.active_scan_command.timestamp_s = std::numeric_limits<double>::quiet_NaN();
    stale_bad_ts.update(bad);
    expect(stale_bad_ts.snapshot().state == "WOULD_ZERO" && stale_bad_ts.snapshot().reason == "command_stale", "non-finite command timestamp should be stale");

    BL4820MotionSafetyExecutor stale(cfg);
    bad = valid_input(2.0);
    bad.active_scan_command.timestamp_s = 1.0;
    stale.update(bad);
    expect(stale.snapshot().state == "WOULD_ZERO" && stale.snapshot().reason == "command_stale", "stale command should zero");
    expect(stale.run_stats(3.0).command_stale_count >= 1, "metrics should count command stale");

    BL4820MotionSafetyExecutor deadman(cfg);
    deadman.update(valid_input(1.0));
    auto late = valid_input(1.4);
    deadman.update(late);
    expect(deadman.snapshot().state == "WOULD_ZERO" && deadman.snapshot().reason == "deadman_timeout", "deadman timeout should zero when command gap exceeds timeout");
    expect(deadman.run_stats(2.0).deadman_timeout_count >= 1, "metrics should count deadman timeout");

    BL4820MotionSafetyExecutor yaw_high(cfg);
    yaw_high.update(valid_input(1.0, 25.0));
    expect(yaw_high.snapshot().state == "BLOCKED" && yaw_high.snapshot().reason == "yaw_rate_too_high", "yaw rate above max should block");

    BL4820MotionSafetyExecutor lin_nonzero(cfg);
    auto lin = valid_input();
    lin.active_scan_command.desired_linear_speed_mps = 0.01;
    lin_nonzero.update(lin);
    expect(lin_nonzero.snapshot().state == "BLOCKED" && lin_nonzero.snapshot().reason == "linear_speed_not_zero", "linear speed should block");

    BL4820MotionSafetyExecutor no_loc(cfg);
    bad = valid_input();
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

    BL4820MotionSafetyExecutor enc(cfg);
    bad = valid_input();
    bad.encoder_feedback_recent = false;
    enc.update(bad);
    expect(enc.snapshot().state == "BLOCKED" && enc.snapshot().reason == "encoder_stale", "stale encoder should block");

    BL4820MotionSafetyExecutor feedback_nan(cfg);
    bad = valid_input();
    bad.left_speed_rpm = std::numeric_limits<double>::quiet_NaN();
    feedback_nan.update(bad);
    expect(feedback_nan.snapshot().state == "BLOCKED" && feedback_nan.snapshot().reason == "feedback_not_finite", "NaN speed should block as feedback_not_finite");

    BL4820MotionSafetyExecutor current_nan(cfg);
    bad = valid_input();
    bad.right_current_a = std::numeric_limits<double>::quiet_NaN();
    current_nan.update(bad);
    expect(current_nan.snapshot().state == "BLOCKED" && current_nan.snapshot().reason == "feedback_not_finite", "NaN current should block as feedback_not_finite");

    BL4820MotionSafetyExecutor negative_current_ok(cfg);
    bad = valid_input();
    bad.left_current_a = -2.5;
    negative_current_ok.update(bad);
    expect(negative_current_ok.snapshot().state == "WOULD_COMMAND", "negative current within abs limit should be ok");

    BL4820MotionSafetyExecutor negative_current_high(cfg);
    bad = valid_input();
    bad.left_current_a = -4.0;
    negative_current_high.update(bad);
    expect(negative_current_high.snapshot().state == "WOULD_ZERO" && negative_current_high.snapshot().reason == "motor_current_high", "negative current over abs limit should zero");

    BL4820MotionSafetyExecutor status(cfg);
    bad = valid_input();
    bad.left_status = 7;
    status.update(bad);
    expect(status.snapshot().state == "WOULD_ZERO" && status.snapshot().reason == "motor_status_error", "motor status error should zero");

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

    Config bad_direction_cfg = cfg;
    bad_direction_cfg.motion_execution_wheel_base_m = -0.160;
    BL4820MotionSafetyExecutor bad_direction(bad_direction_cfg);
    bad_direction.update(valid_input());
    expect(bad_direction.snapshot().state == "BLOCKED" && bad_direction.snapshot().reason == "wheel_direction_invalid", "invalid target wheel direction should block");

    BL4820MotionSafetyExecutor zero_plan(cfg);
    bad = valid_input();
    bad.active_scan_command.zero_command = true;
    zero_plan.update(bad);
    expect(zero_plan.snapshot().state == "WOULD_ZERO" && zero_plan.snapshot().reason == "zero_from_planner", "planner zero should produce WOULD_ZERO");

    Config limited_cfg = cfg;
    limited_cfg.motion_execution_max_wheel_speed_rpm = 5.0;
    BL4820MotionSafetyExecutor limited(limited_cfg);
    limited.update(valid_input(1.0, 20.0));
    expect(limited.snapshot().state == "WOULD_COMMAND" && limited.snapshot().reason == "wheel_speed_limited", "wheel speed limit should keep dry-run command but record reason");
    expect_near(std::fabs(limited.snapshot().target_left_rpm), 5.0, 1e-9, "left rpm should be limited");

    BL4820MotionSafetyExecutor no_ack(cfg);
    no_ack.update(valid_input());
    expect(!no_ack.snapshot().write_authorization_present && !no_ack.snapshot().write_authorization_valid, "empty write ack should not be valid");
    Config ack_cfg = cfg;
    ack_cfg.motion_execution_write_mode_acknowledgement = ack_cfg.motion_execution_required_write_mode_acknowledgement;
    BL4820MotionSafetyExecutor ack(ack_cfg);
    ack.update(valid_input());
    expect(ack.snapshot().write_authorization_present && ack.snapshot().write_authorization_valid, "correct write ack should be reflected but remain dry-run");
    expect(ack.snapshot().would_command && ack.snapshot().state == "WOULD_COMMAND", "write ack must not change dry-run behavior");

    std::ostringstream header;
    write_motion_safety_executor_header(header);
    expect(header.str().find("timestamp_s,state,previous_state,reason,command_seen,would_command,would_zero") == 0, "motion CSV header should start with stable fields");
    expect(header.str().find("command_session_id,command_session_duration_s,command_duration_ok,command_age_s,deadman_age_s,write_authorization_present,write_authorization_valid,feedback_finite_ok,wheel_direction_ok") != std::string::npos, "motion CSV header should append M1+ fields");

    auto stats = would.run_stats(3.0);
    expect(stats.would_command_count >= 2, "metrics should count would_command");
    expect(stats.command_seen_count >= 2, "metrics should count command seen");
    expect_near(stats.last_target_left_rpm, -expected_rpm, 1e-6, "metrics should keep last left rpm");
    expect(stats.last_reason == "would_command", "metrics should keep last reason");
    expect(stats.last_command_session_id == 1, "metrics should keep command session id");
    expect_near(stats.last_command_session_duration_s, 0.2, 1e-9, "metrics should keep command session duration");
    expect(duration.run_stats(3.0).command_duration_exceeded_count >= 1, "metrics should count duration exceeded");
    expect(feedback_nan.run_stats(1.0).feedback_not_finite_block_count >= 1, "metrics should count feedback finite reject");
    expect(bad_direction.run_stats(1.0).wheel_direction_invalid_count >= 1, "metrics should count wheel direction invalid");
    expect(ack.run_stats(1.0).write_authorization_valid_last, "metrics should keep write authorization validity");

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
