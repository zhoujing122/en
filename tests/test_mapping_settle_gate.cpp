#include "robot_slamd/runtime/mapping_settle_gate.hpp"

#include <cmath>

static robot_slamd::RobotSlamSensorSnapshot snapshot(double t) {
    robot_slamd::RobotSlamSensorSnapshot s;
    s.has_wheel = true;
    s.has_imu = true;
    s.wheel.valid = true;
    s.wheel.timestamp_s = t;
    s.wheel.left_rpm = 0.2;
    s.wheel.right_rpm = -0.2;
    s.wheel.pair_skew_s = 0.001;
    s.wheel.pair_skew_valid = true;
    s.imu.timestamp_s = t;
    s.imu.yaw_rate_rad_s = 0.01;
    return s;
}

int main() {
    using namespace robot_slamd;
    MappingSettleGate gate;
    RelativeMotionStepResult done;
    done.command_id = 7;
    done.state = RelativeMotionStepState::Done;
    if (gate.update(done, snapshot(1.0), 1000000).stable) return 1;
    if (gate.update(done, snapshot(1.1), 1100000).stable) return 1;
    if (!gate.update(done, snapshot(1.2), 1200000).stable) return 1;
    auto moving = snapshot(1.3);
    moving.wheel.left_rpm = 2.0;
    if (gate.update(done, moving, 1300000).stable) return 1;
    auto skewed = snapshot(1.4);
    skewed.wheel.pair_skew_s = 0.011;
    if (gate.update(done, skewed, 1400000).reason != "wheel_pair_skew_exceeded") return 1;
    RelativeMotionStepResult running = done;
    running.state = RelativeMotionStepState::Running;
    if (gate.update(running, snapshot(1.5), 1500000).reason != "motion_not_done") return 1;
    return 0;
}
