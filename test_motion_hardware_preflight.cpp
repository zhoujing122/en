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

robot_slamd::MotionHardwarePreflightInput valid_input() {
    robot_slamd::MotionHardwarePreflightInput in;
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
} // namespace

int main() {
    using namespace robot_slamd;
    auto in = valid_input();
    auto r = run_motion_hardware_preflight(in);
    expect(r.ok, "valid preflight should pass");

    auto check = [&](auto mutate, const std::string &error) {
        auto x = valid_input();
        mutate(x);
        auto out = run_motion_hardware_preflight(x);
        expect(!out.ok, error.c_str());
        expect(has_error(out, error), error.c_str());
    };

    check([](auto &x){ x.operator_present = false; }, "operator_not_present");
    check([](auto &x){ x.robot_lifted_or_wheels_free = false; }, "robot_not_lifted_or_wheels_not_free");
    check([](auto &x){ x.emergency_stop_available = false; }, "emergency_stop_missing");
    check([](auto &x){ x.software_transport_ready = false; }, "software_transport_not_ready");
    check([](auto &x){ x.software_transport_shadow_mode_verified = false; }, "shadow_mode_not_verified");
    check([](auto &x){ x.stop_command_verified = false; }, "stop_command_not_verified");
    check([](auto &x){ x.ttl_stop_verified = false; }, "ttl_stop_not_verified");
    check([](auto &x){ x.ack_semantics_verified = false; }, "ack_semantics_not_verified");
    check([](auto &x){ x.left_right_direction_convention_verified = false; }, "direction_convention_not_verified");
    check([](auto &x){ x.encoder_feedback_healthy = false; }, "encoder_feedback_unhealthy");
    check([](auto &x){ x.motor_status_zero = false; }, "motor_status_nonzero");
    check([](auto &x){ x.current_within_limit = false; }, "current_limit_not_ok");
    check([](auto &x){ x.obstacle_clear = false; }, "obstacle_not_clear");
    check([](auto &x){ x.requested_max_speed_normalized = std::numeric_limits<double>::quiet_NaN(); }, "invalid_requested_speed");
    check([](auto &x){ x.requested_max_speed_normalized = 0.051; }, "requested_speed_too_high");
    check([](auto &x){ x.requested_max_duration_s = std::numeric_limits<double>::quiet_NaN(); }, "invalid_requested_duration");
    check([](auto &x){ x.requested_max_duration_s = 0.501; }, "requested_duration_too_long");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
