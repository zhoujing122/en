#pragma once
#include "common.hpp"

namespace robot_slamd {

struct ManualMotionArmRequest {
    bool enable_live_motion = false;
    bool confirm_robot_lifted = false;
    bool confirm_emergency_stop = false;
    bool confirm_low_speed = false;
    bool confirm_short_duration = false;
    bool confirm_operator_present = false;
    std::string confirmation_phrase;
};

struct ManualMotionArmResult {
    bool armed = false;
    std::string reason;
};

class ManualMotionArmGate {
public:
    ManualMotionArmResult evaluate(const ManualMotionArmRequest &request) const {
        if (!request.enable_live_motion) return {false, "live_motion_not_enabled"};
        if (!request.confirm_robot_lifted) return {false, "robot_lift_confirmation_missing"};
        if (!request.confirm_emergency_stop) return {false, "emergency_stop_confirmation_missing"};
        if (!request.confirm_low_speed) return {false, "low_speed_confirmation_missing"};
        if (!request.confirm_short_duration) return {false, "short_duration_confirmation_missing"};
        if (!request.confirm_operator_present) return {false, "operator_confirmation_missing"};
        if (request.confirmation_phrase != "I_UNDERSTAND_M2B1_LIFTED_LOW_SPEED_ONLY") return {false, "confirmation_phrase_mismatch"};
        return {true, "armed"};
    }
};

} // namespace robot_slamd
