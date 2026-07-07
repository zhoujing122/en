#include "motion_hardware_preflight.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool has_error(const robot_slamd::MotionHardwarePreflightResult &r, const std::string &e) {
    return std::find(r.errors.begin(), r.errors.end(), e) != r.errors.end();
}

bool has_warning(const robot_slamd::MotionHardwarePreflightResult &r, const std::string &w) {
    return std::find(r.warnings.begin(), r.warnings.end(), w) != r.warnings.end();
}

robot_slamd::MotionHardwarePreflightInput valid_confirmed_input() {
    robot_slamd::MotionHardwarePreflightInput in;
    in.mode = robot_slamd::MotionHardwarePreflightMode::ConfirmedLiftedLive;
    in.operator_present = true;
    in.robot_lifted_or_wheels_free = true;
    in.emergency_stop_available = true;
    in.software_transport_ready = true;
    in.software_transport_shadow_mode_verified = true;
    in.stop_command_verified = true;
    in.ttl_stop_verified = true;
    in.ack_semantics_verified = true;
    in.left_right_direction_convention_verified = true;
    in.encoder_feedback_healthy = true;
    in.motor_status_zero = true;
    in.current_within_limit = true;
    in.obstacle_clear = true;
    in.requested_max_speed_normalized = 0.05;
    in.requested_max_duration_s = 0.5;
    return in;
}

robot_slamd::MotionHardwarePreflightInput valid_probe_input() {
    auto in = valid_confirmed_input();
    in.mode = robot_slamd::MotionHardwarePreflightMode::LiftedDirectionProbe;
    in.left_right_direction_convention_verified = false;
    in.requested_max_speed_normalized = 0.03;
    in.requested_max_duration_s = 0.30;
    return in;
}
} // namespace

int main() {
    using namespace robot_slamd;
    auto confirmed = valid_confirmed_input();
    auto r = run_motion_hardware_preflight(confirmed);
    expect(r.ok, "confirmed live valid preflight should pass");

    auto probe = valid_probe_input();
    r = run_motion_hardware_preflight(probe);
    expect(r.ok, "direction probe can pass with direction pending");
    expect(r.direction_convention_pending, "direction probe marks pending convention");
    expect(has_warning(r, "direction_convention_pending"), "direction probe warning present");
    expect(!has_error(r, "direction_convention_not_verified"), "direction probe does not fail on pending direction");

    auto check_confirmed = [&](auto mutate, const std::string &error) {
        auto x = valid_confirmed_input();
        mutate(x);
        auto out = run_motion_hardware_preflight(x);
        expect(!out.ok, error.c_str());
        expect(has_error(out, error), error.c_str());
    };

    auto check_probe = [&](auto mutate, const std::string &error) {
        auto x = valid_probe_input();
        mutate(x);
        auto out = run_motion_hardware_preflight(x);
        expect(!out.ok, error.c_str());
        expect(has_error(out, error), error.c_str());
    };

    check_probe([](auto &x){ x.requested_max_speed_normalized = 0.04; }, "direction_probe_speed_too_high");
    check_probe([](auto &x){ x.requested_max_duration_s = 0.4; }, "direction_probe_duration_too_long");
    check_probe([](auto &x){ x.operator_present = false; }, "operator_not_present");
    check_probe([](auto &x){ x.robot_lifted_or_wheels_free = false; }, "robot_not_lifted_or_wheels_not_free");
    check_probe([](auto &x){ x.emergency_stop_available = false; }, "emergency_stop_missing");
    check_probe([](auto &x){ x.stop_command_verified = false; }, "stop_command_not_verified");
    check_probe([](auto &x){ x.ttl_stop_verified = false; }, "ttl_stop_not_verified");
    check_probe([](auto &x){ x.ack_semantics_verified = false; }, "ack_semantics_not_verified");
    check_probe([](auto &x){ x.encoder_feedback_healthy = false; }, "encoder_feedback_unhealthy");
    check_probe([](auto &x){ x.motor_status_zero = false; }, "motor_status_nonzero");
    check_probe([](auto &x){ x.current_within_limit = false; }, "current_limit_not_ok");
    check_probe([](auto &x){ x.obstacle_clear = false; }, "obstacle_not_clear");

    check_confirmed([](auto &x){ x.operator_present = false; }, "operator_not_present");
    check_confirmed([](auto &x){ x.left_right_direction_convention_verified = false; }, "direction_convention_not_verified");
    check_confirmed([](auto &x){ x.requested_max_speed_normalized = std::numeric_limits<double>::quiet_NaN(); }, "invalid_requested_speed");
    check_confirmed([](auto &x){ x.requested_max_speed_normalized = 0.051; }, "requested_speed_too_high");
    check_confirmed([](auto &x){ x.requested_max_duration_s = std::numeric_limits<double>::quiet_NaN(); }, "invalid_requested_duration");
    check_confirmed([](auto &x){ x.requested_max_duration_s = 0.501; }, "requested_duration_too_long");

    auto confirmed_ok = valid_confirmed_input();
    confirmed_ok.left_right_direction_convention_verified = true;
    confirmed_ok.requested_max_speed_normalized = 0.05;
    confirmed_ok.requested_max_duration_s = 0.5;
    expect(run_motion_hardware_preflight(confirmed_ok).ok, "confirmed live passes at limits");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
