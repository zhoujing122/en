#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <cmath>
#include <cstdint>
#include <string>

namespace robot_slamd {

// This is the neutral robot_slamd contract.  It intentionally does not
// mention UART, motor registers, or the motore repository.
enum class RelativeMotionStepAction : std::uint8_t {
    Stop = 0,
    Right = 1,
    Left = 2,
    Forward = 3,
    Back = 4,
    Estop = 5
};
static_assert(static_cast<int>(RelativeMotionStepAction::Right) == 1);
static_assert(static_cast<int>(RelativeMotionStepAction::Left) == 2);

enum class RelativeMotionStepState : std::uint8_t {
    Accepted,
    Running,
    Stopping,
    Settling,
    Done,
    Cancelled,
    Timeout,
    Fault,
    EstopLatched
};

struct RelativeMotionStepCommand {
    std::uint64_t command_id = 0;
    RelativeMotionStepAction action = RelativeMotionStepAction::Stop;
    double target_amount = 0.0;
    double max_rpm = 0.0;
    std::uint64_t timeout_ms = 0;
};

struct RelativeMotionStepResult {
    std::uint64_t command_id = 0;
    RelativeMotionStepAction action = RelativeMotionStepAction::Stop;
    RelativeMotionStepState state = RelativeMotionStepState::Fault;
    double requested_amount = 0.0;
    double actual_distance_mm = 0.0;
    double actual_angle_deg = 0.0;
    RobotSlamSensorSnapshot final_feedback;
    int error_code = 0;
    std::string reason = "not_run";
    bool motor_settled = false;
};

inline bool relative_motion_step_is_translation(RelativeMotionStepAction a) {
    return a == RelativeMotionStepAction::Forward ||
           a == RelativeMotionStepAction::Back;
}

inline bool relative_motion_step_is_rotation(RelativeMotionStepAction a) {
    return a == RelativeMotionStepAction::Right ||
           a == RelativeMotionStepAction::Left;
}

inline const char *to_string(RelativeMotionStepAction action) {
    switch (action) {
    case RelativeMotionStepAction::Stop: return "STOP";
    case RelativeMotionStepAction::Right: return "RIGHT";
    case RelativeMotionStepAction::Left: return "LEFT";
    case RelativeMotionStepAction::Forward: return "FORWARD";
    case RelativeMotionStepAction::Back: return "BACK";
    case RelativeMotionStepAction::Estop: return "ESTOP";
    }
    return "INVALID";
}

inline const char *to_string(RelativeMotionStepState state) {
    switch (state) {
    case RelativeMotionStepState::Accepted: return "ACCEPTED";
    case RelativeMotionStepState::Running: return "RUNNING";
    case RelativeMotionStepState::Stopping: return "STOPPING";
    case RelativeMotionStepState::Settling: return "SETTLING";
    case RelativeMotionStepState::Done: return "DONE";
    case RelativeMotionStepState::Cancelled: return "CANCELLED";
    case RelativeMotionStepState::Timeout: return "TIMEOUT";
    case RelativeMotionStepState::Fault: return "FAULT";
    case RelativeMotionStepState::EstopLatched: return "ESTOP_LATCHED";
    }
    return "UNKNOWN";
}

class RelativeMotionStepPort {
public:
    virtual ~RelativeMotionStepPort() = default;
    virtual bool ready() const = 0;
    virtual RelativeMotionStepResult submit(
        const RelativeMotionStepCommand &command, double now_s) = 0;
    virtual RelativeMotionStepResult poll(double now_s) = 0;
    virtual RelativeMotionStepResult cancel(double now_s) = 0;
    virtual RelativeMotionStepResult stop(double now_s) = 0;
    virtual RelativeMotionStepResult emergency_stop(double now_s) = 0;
    virtual RelativeMotionStepResult clear_estop(double now_s) = 0;
    virtual std::string status() const = 0;
};

} // namespace robot_slamd
