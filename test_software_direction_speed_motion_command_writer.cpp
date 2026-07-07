#include "config.hpp"
#include "motion_write_controller.hpp"
#include "software_direction_speed_motion_command_writer.hpp"
#include "test_software_motion_transport_fakes.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
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

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

robot_slamd::SoftwareDirectionSpeedMotionCommandWriter::Options options(bool rotation = true) {
    robot_slamd::SoftwareDirectionSpeedMotionCommandWriter::Options o;
    o.max_internal_rpm_for_normalization = 30.0;
    o.max_speed_normalized = 0.10;
    o.command_ttl_s = 0.30;
    o.allow_rotation_commands = rotation;
    o.allow_translation_commands = false;
    return o;
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

void static_safety_search() {
    const std::string file = __FILE__;
    const size_t slash = file.find_last_of("/\\");
    const std::string dir = slash == std::string::npos ? std::string() : file.substr(0, slash + 1);
    const std::vector<std::string> production = {
        dir + "software_motion_command.hpp",
        dir + "software_motion_transport.hpp",
        dir + "loopback_software_motion_transport.hpp",
        dir + "software_direction_speed_motion_command_writer.hpp",
        dir + "motion_write_controller.hpp",
        dir + "app.hpp",
    };
    const std::vector<std::string> forbidden = {
        "bl4820_write_register",
        "uart_write_speed",
        "write_speed_register",
        "set_pwm",
        "fr_output",
        "real_motor_write",
        "send_motor_command_to_hardware",
        "apply_motor_command_to_hardware",
        "apply_pose_correction",
        "startup_relocalization_writeback",
        "lost_recovery_writeback",
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

    auto transport = std::make_shared<FakeSoftwareMotionCommandTransport>();
    SoftwareDirectionSpeedMotionCommandWriter writer(transport, options());

    auto r = writer.write_zero(1.0);
    expect(r.ok, "write_zero succeeds");
    expect(transport->sent_commands.size() == 1, "zero command sent");
    expect(transport->sent_commands.back().direction == SoftwareMotionDirection::Stop, "zero sends stop");
    expect_near(transport->sent_commands.back().speed_normalized, 0.0, 1e-12, "zero speed");
    expect_near(transport->sent_commands.back().ttl_s, 0.30, 1e-12, "zero ttl");

    transport->fail_next_send = true;
    r = writer.write_zero(2.0);
    expect(!r.ok && r.error == "software_motion_send_failed", "zero send failure");

    transport->reject_next_send = true;
    r = writer.write_zero(3.0);
    expect(!r.ok && r.error == "software_motion_rejected", "zero rejected");

    r = writer.write_zero(std::numeric_limits<double>::quiet_NaN());
    expect(!r.ok && r.error == "non_finite_timestamp", "zero nonfinite timestamp");

    transport->clear();
    r = writer.write_wheel_rpm(4.0, -15.0, 15.0);
    expect(r.ok, "opposite wheel signs should rotate");
    expect(transport->sent_commands.back().direction == SoftwareMotionDirection::TurnLeft, "left negative/right positive is TurnLeft");
    expect_near(transport->sent_commands.back().speed_normalized, 0.10, 1e-12, "speed capped");

    r = writer.write_wheel_rpm(4.1, 12.0, -12.0);
    expect(r.ok, "reverse opposite signs should rotate");
    expect(transport->sent_commands.back().direction == SoftwareMotionDirection::TurnRight, "left positive/right negative is TurnRight");

    auto no_rotation_transport = std::make_shared<FakeSoftwareMotionCommandTransport>();
    SoftwareDirectionSpeedMotionCommandWriter no_rotation(no_rotation_transport, options(false));
    r = no_rotation.write_wheel_rpm(5.0, -10.0, 10.0);
    expect(!r.ok && r.error == "software_motion_command_invalid", "rotation disabled rejects rpm command");

    r = writer.write_wheel_rpm(6.0, 10.0, 10.0);
    expect(!r.ok && r.error == "translation_motion_disabled", "same sign wheels rejected");

    transport->clear();
    r = writer.write_wheel_rpm(7.0, 0.0, 0.0);
    expect(r.ok && transport->sent_commands.back().direction == SoftwareMotionDirection::Stop, "zero rpm maps to stop");

    r = writer.write_wheel_rpm(8.0, std::numeric_limits<double>::quiet_NaN(), 1.0);
    expect(!r.ok && r.error == "non_finite_rpm", "rpm finite check");

    auto bad_options = options();
    bad_options.max_internal_rpm_for_normalization = 0.0;
    SoftwareDirectionSpeedMotionCommandWriter bad_norm(transport, bad_options);
    r = bad_norm.write_wheel_rpm(9.0, -1.0, 1.0);
    expect(!r.ok && r.error == "invalid_normalization_rpm", "normalization rpm check");

    transport->fail_next_send = true;
    r = writer.write_wheel_rpm(10.0, -5.0, 5.0);
    expect(!r.ok && r.error == "software_motion_send_failed", "rpm send failure");

    transport->reject_next_send = true;
    r = writer.write_wheel_rpm(11.0, -5.0, 5.0);
    expect(!r.ok && r.error == "software_motion_rejected", "rpm rejected");

    MotionWriteController controller;
    auto ctrl_transport = std::make_shared<FakeSoftwareMotionCommandTransport>();
    SoftwareDirectionSpeedMotionCommandWriter ctrl_writer(ctrl_transport, options());
    auto snap = controller.update(would_command(12.0), ctrl_writer, true);
    expect(snap.wrote_rpm && ctrl_transport->sent_commands.size() == 1, "controller dispatches software rpm");
    ctrl_transport->fail_next_send = true;
    snap = controller.update(would_command(12.1), ctrl_writer, true);
    expect(snap.writer_error, "controller sees writer failure");
    snap = controller.update(would_command(12.2), ctrl_writer, true);
    expect(snap.wrote_zero, "writer fault forces zero");

    MotionWriteController latch_controller;
    auto latch_transport = std::make_shared<FakeSoftwareMotionCommandTransport>();
    SoftwareDirectionSpeedMotionCommandWriter latch_writer(latch_transport, options());
    auto latched = would_command(13.0);
    latched.command_duration_latched = true;
    snap = latch_controller.update(latched, latch_writer, true);
    expect(snap.wrote_zero, "latched command only writes zero");
    expect(latch_transport->sent_commands.back().direction == SoftwareMotionDirection::Stop, "latched command sends stop");

    Config cfg;
    validate_config(cfg);
    expect(cfg.motion_execution_writer_backend == "null", "default backend null");
    Config invalid = cfg;
    invalid.motion_execution_writer_backend = "software_direction_speed_test_only";
    expect_throw([&] { validate_config(invalid); }, "test-only backend fatal in production config");
    invalid = cfg;
    invalid.motion_execution_software_motion_production_interface_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "production interface fatal");
    invalid = cfg;
    invalid.motion_execution_allow_writer_dispatch = true;
    expect_throw([&] { validate_config(invalid); }, "allow_writer_dispatch fatal");
    invalid = cfg;
    invalid.motion_execution_hardware_write_enabled = true;
    expect_throw([&] { validate_config(invalid); }, "hardware_write_enabled fatal");
    invalid = cfg;
    invalid.motion_execution_mode = "hardware";
    expect_throw([&] { validate_config(invalid); }, "mode hardware fatal");

    static_safety_search();

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
