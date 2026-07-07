#include "robot_slamd/config/config.hpp"
#include "robot_slamd/motion/motion_write_controller.hpp"

#include <cmath>
#include <fstream>
#include <functional>
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

robot_slamd::Config motion_config() {
    robot_slamd::Config c;
    c.motion_execution_enabled = true;
    c.motion_execution_mode = "dry_run";
    c.motion_execution_log_hz = 5.0;
    c.motion_execution_dry_run_write_motor_commands = false;
    c.motion_execution_hardware_write_enabled = false;
    c.motion_execution_allow_writer_dispatch = false;
    return c;
}

robot_slamd::MotionSafetyExecutorSnapshot would_command(double t = 1.0) {
    robot_slamd::MotionSafetyExecutorSnapshot s;
    s.timestamp_s = t;
    s.state = "WOULD_COMMAND";
    s.reason = "would_command";
    s.would_command = true;
    s.target_left_rpm = -12.0;
    s.target_right_rpm = 12.0;
    return s;
}

robot_slamd::MotionSafetyExecutorSnapshot would_zero(double t = 1.0) {
    robot_slamd::MotionSafetyExecutorSnapshot s;
    s.timestamp_s = t;
    s.state = "WOULD_ZERO";
    s.reason = "would_zero";
    s.would_zero = true;
    return s;
}

robot_slamd::MotionSafetyExecutorSnapshot blocked(double t = 1.0) {
    robot_slamd::MotionSafetyExecutorSnapshot s;
    s.timestamp_s = t;
    s.state = "BLOCKED";
    s.reason = "front_obstacle_too_close";
    s.blocked = true;
    return s;
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
    const std::string dir = slash == std::string::npos ? std::string("../") : file.substr(0, slash + 1) + "../";
    const std::vector<std::string> production = {
        dir + "include/robot_slamd/motion/motion_command_writer.hpp",
        dir + "include/robot_slamd/motion/motion_write_controller.hpp",
        dir + "include/robot_slamd/motion/bl4820_motion_safety_executor.hpp",
        dir + "include/robot_slamd/app/app.hpp",
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
    validate_config(cfg);
    Config invalid = cfg;
    invalid.motion_execution_mode = "hardware";
    expect_throw([&] { validate_config(invalid); }, "mode=hardware should remain fatal");
    invalid = cfg;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "hardware_write_enabled=true should remain fatal");
    invalid = cfg;
    invalid.motion_execution_allow_writer_dispatch = true;
    expect_throw([&] { validate_config(invalid); }, "allow_writer_dispatch=true should be fatal in production config");

    NullMotionCommandWriter null_writer;
    expect(null_writer.write_zero(1.0).ok, "null writer zero should succeed without hardware");
    expect(null_writer.write_wheel_rpm(1.0, -1.0, 1.0).ok, "null writer rpm should succeed without hardware");

    FakeMotionCommandWriter fake;
    expect(fake.write_zero(1.0).ok, "fake zero should succeed");
    expect(fake.zero_write_count == 1, "fake zero count should increment");
    expect(fake.write_wheel_rpm(2.0, -4.0, 5.0).ok, "fake rpm should succeed");
    expect(fake.rpm_write_count == 1, "fake rpm count should increment");
    expect_near(fake.last_left_rpm, -4.0, 1e-12, "fake last left rpm");
    expect_near(fake.last_right_rpm, 5.0, 1e-12, "fake last right rpm");

    MotionWriteController gated;
    FakeMotionCommandWriter gated_writer;
    auto snap = gated.update(would_command(3.0), gated_writer, false);
    expect(!snap.attempted && gated_writer.rpm_write_count == 0, "dispatch disabled should not write rpm");
    expect(gated.run_stats(3.0).dispatch_enabled_last == false, "dispatch disabled metric should be false");

    MotionWriteController controller;
    FakeMotionCommandWriter writer;
    snap = controller.update(would_command(4.0), writer, true);
    expect(snap.wrote_rpm && writer.rpm_write_count == 1, "enabled would_command should write rpm");
    expect_near(writer.last_left_rpm, -12.0, 1e-12, "writer left target");
    expect_near(writer.last_right_rpm, 12.0, 1e-12, "writer right target");
    snap = controller.update(would_zero(4.1), writer, true);
    expect(snap.wrote_zero && writer.zero_write_count == 1, "enabled would_zero should write zero");

    MotionWriteController blocked_controller;
    FakeMotionCommandWriter blocked_writer;
    blocked_controller.update(would_command(5.0), blocked_writer, true);
    snap = blocked_controller.update(blocked(5.1), blocked_writer, true);
    expect(snap.wrote_zero && blocked_writer.zero_write_count == 1, "blocked after nonzero should write zero");
    expect(blocked_writer.rpm_write_count == 1, "blocked should not write additional rpm");
    snap = blocked_controller.update(blocked(5.2), blocked_writer, true);
    expect(!snap.wrote_zero && blocked_writer.zero_write_count == 1, "blocked without active nonzero should not repeat zero");

    MotionWriteController fault_controller;
    FakeMotionCommandWriter fault_writer;
    fault_controller.update(would_command(6.0), fault_writer, true);
    auto fault = blocked(6.1);
    fault.state = "FAULT";
    fault.fault = true;
    snap = fault_controller.update(fault, fault_writer, true);
    expect(snap.wrote_zero && fault_writer.zero_write_count == 1, "fault after nonzero should write zero");

    MotionWriteController latch_controller;
    FakeMotionCommandWriter latch_writer;
    auto latched = would_command(7.0);
    latched.command_duration_latched = true;
    latched.command_duration_latch_session_id = 9;
    snap = latch_controller.update(latched, latch_writer, true);
    expect(snap.wrote_zero && latch_writer.zero_write_count == 1, "duration latch should only write zero");
    expect(latch_writer.rpm_write_count == 0, "duration latch must not write rpm");

    MotionWriteController fail_controller;
    FakeMotionCommandWriter fail_writer;
    fail_writer.fail_next_write = true;
    snap = fail_controller.update(would_command(8.0), fail_writer, true);
    expect(snap.writer_error && snap.reason == "writer_error", "writer failure should be recorded");
    snap = fail_controller.update(would_command(8.1), fail_writer, true);
    expect(snap.wrote_zero && fail_writer.zero_write_count == 1, "writer fault should force a zero before any later command");
    snap = fail_controller.update(would_command(8.2), fail_writer, true);
    expect(!snap.wrote_rpm && fail_writer.rpm_write_count == 0, "writer fault should prevent later nonzero");
    expect(fail_controller.run_stats(8.2).error_count >= 1, "writer error metric should increment");

    auto stats = controller.run_stats(10.0);
    expect(stats.dispatch_enabled_last, "writer dispatch metric should remember enabled state");
    expect(stats.zero_write_count == 1, "writer zero metric should count writes");
    expect(stats.rpm_write_count == 1, "writer rpm metric should count writes");
    expect_near(stats.last_left_rpm, 0.0, 1e-12, "writer stats last zero left rpm");
    expect_near(stats.last_right_rpm, 0.0, 1e-12, "writer stats last zero right rpm");

    static_safety_search();

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
