#include "manual_motion_arm_gate.hpp"

#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::ManualMotionArmRequest valid_request() {
    robot_slamd::ManualMotionArmRequest r;
    r.enable_live_motion = true;
    r.confirm_robot_lifted = true;
    r.confirm_emergency_stop = true;
    r.confirm_low_speed = true;
    r.confirm_short_duration = true;
    r.confirm_operator_present = true;
    r.confirmation_phrase = "I_UNDERSTAND_M2B1_LIFTED_LOW_SPEED_ONLY";
    return r;
}
} // namespace

int main() {
    using namespace robot_slamd;
    ManualMotionArmGate gate;
    auto r = gate.evaluate(valid_request());
    expect(r.armed, "valid arm request should arm");

    auto check = [&](auto mutate, const std::string &reason) {
        auto req = valid_request();
        mutate(req);
        auto out = gate.evaluate(req);
        expect(!out.armed, reason.c_str());
        expect(out.reason == reason, reason.c_str());
    };
    check([](auto &x){ x.enable_live_motion = false; }, "live_motion_not_enabled");
    check([](auto &x){ x.confirm_robot_lifted = false; }, "robot_lift_confirmation_missing");
    check([](auto &x){ x.confirm_emergency_stop = false; }, "emergency_stop_confirmation_missing");
    check([](auto &x){ x.confirm_low_speed = false; }, "low_speed_confirmation_missing");
    check([](auto &x){ x.confirm_short_duration = false; }, "short_duration_confirmation_missing");
    check([](auto &x){ x.confirm_operator_present = false; }, "operator_confirmation_missing");
    check([](auto &x){ x.confirmation_phrase = "wrong"; }, "confirmation_phrase_mismatch");

    if (failures) {
        std::cerr << failures << " failures\n";
        return 1;
    }
    return 0;
}
