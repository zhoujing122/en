#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/simulation/core/sim_math.hpp"
#include "robot_slamd/simulation/world/fake_world_2d.hpp"

#include <cmath>

namespace robot_slamd {

struct SimRobotState {
    RobotPose2D pose;
    double linear_velocity_mps = 0.0;
    double angular_velocity_rad_s = 0.0;
    double left_wheel_position_rad = 0.0;
    double right_wheel_position_rad = 0.0;
    double left_wheel_speed_rad_s = 0.0;
    double right_wheel_speed_rad_s = 0.0;
    double timestamp_s = 0.0;
    bool collision = false;
};

struct SimRobotPlantConfig {
    double wheel_radius_m = 0.032;
    double wheel_base_m = 0.160;
    double max_linear_speed_mps = 0.20;
    double max_angular_speed_rad_s = 1.20;
    double max_linear_accel_mps2 = 0.30;
    double max_angular_accel_rad_s2 = 1.80;
    double robot_radius_m = 0.080;
    bool collision_check_enabled = false;
};

class SimRobotPlant {
public:
    explicit SimRobotPlant(SimRobotPlantConfig config = {})
        : config_(config) {}

    bool valid() const {
        return sim_finite(config_.wheel_radius_m) && config_.wheel_radius_m > 0.0 &&
               sim_finite(config_.wheel_base_m) && config_.wheel_base_m > 0.0 &&
               sim_finite(config_.max_linear_speed_mps) && config_.max_linear_speed_mps >= 0.0 &&
               sim_finite(config_.max_angular_speed_rad_s) && config_.max_angular_speed_rad_s >= 0.0 &&
               sim_finite(config_.max_linear_accel_mps2) && config_.max_linear_accel_mps2 >= 0.0 &&
               sim_finite(config_.max_angular_accel_rad_s2) && config_.max_angular_accel_rad_s2 >= 0.0 &&
               sim_finite(config_.robot_radius_m) && config_.robot_radius_m >= 0.0;
    }

    bool reset(const RobotPose2D &pose, double now_s = 0.0) {
        if (!valid_pose(pose) || !sim_finite(now_s)) {
            return false;
        }
        state_ = SimRobotState{};
        state_.pose = pose;
        state_.pose.yaw_rad = sim_normalize_yaw(pose.yaw_rad);
        state_.timestamp_s = now_s;
        target_linear_velocity_mps_ = 0.0;
        target_angular_velocity_rad_s_ = 0.0;
        return true;
    }

    bool set_target_velocity(double linear_mps, double angular_rad_s) {
        if (!sim_finite(linear_mps) || !sim_finite(angular_rad_s) || !valid()) {
            return false;
        }
        target_linear_velocity_mps_ =
            sim_clamp(linear_mps, -config_.max_linear_speed_mps, config_.max_linear_speed_mps);
        target_angular_velocity_rad_s_ =
            sim_clamp(angular_rad_s, -config_.max_angular_speed_rad_s, config_.max_angular_speed_rad_s);
        return true;
    }

    void stop_target() {
        target_linear_velocity_mps_ = 0.0;
        target_angular_velocity_rad_s_ = 0.0;
    }

    bool step(double dt_s, double now_s, const FakeWorld2D *world = nullptr) {
        if (!valid() || !sim_finite(dt_s) || dt_s <= 0.0 || !sim_finite(now_s)) {
            return false;
        }
        const double next_linear = sim_step_toward(
            state_.linear_velocity_mps,
            target_linear_velocity_mps_,
            config_.max_linear_accel_mps2 * dt_s);
        const double next_angular = sim_step_toward(
            state_.angular_velocity_rad_s,
            target_angular_velocity_rad_s_,
            config_.max_angular_accel_rad_s2 * dt_s);

        RobotPose2D candidate = state_.pose;
        const double yaw_mid = state_.pose.yaw_rad + 0.5 * next_angular * dt_s;
        candidate.x_m += next_linear * std::cos(yaw_mid) * dt_s;
        candidate.y_m += next_linear * std::sin(yaw_mid) * dt_s;
        candidate.yaw_rad = sim_normalize_yaw(state_.pose.yaw_rad + next_angular * dt_s);

        state_.linear_velocity_mps = next_linear;
        state_.angular_velocity_rad_s = next_angular;
        if (config_.collision_check_enabled && world &&
            world->circle_collides(candidate.x_m, candidate.y_m, config_.robot_radius_m)) {
            state_.collision = true;
            candidate.x_m = state_.pose.x_m;
            candidate.y_m = state_.pose.y_m;
            target_linear_velocity_mps_ = 0.0;
            state_.linear_velocity_mps = 0.0;
        }

        if (!valid_pose(candidate)) {
            return false;
        }
        state_.pose = candidate;
        state_.left_wheel_speed_rad_s =
            (state_.linear_velocity_mps - 0.5 * state_.angular_velocity_rad_s * config_.wheel_base_m) /
            config_.wheel_radius_m;
        state_.right_wheel_speed_rad_s =
            (state_.linear_velocity_mps + 0.5 * state_.angular_velocity_rad_s * config_.wheel_base_m) /
            config_.wheel_radius_m;
        state_.left_wheel_position_rad += state_.left_wheel_speed_rad_s * dt_s;
        state_.right_wheel_position_rad += state_.right_wheel_speed_rad_s * dt_s;
        state_.timestamp_s = now_s;
        return sim_finite(state_.left_wheel_position_rad) &&
               sim_finite(state_.right_wheel_position_rad);
    }

    const SimRobotState &state() const {
        return state_;
    }

    const SimRobotPlantConfig &config() const {
        return config_;
    }

    double target_linear_velocity_mps() const {
        return target_linear_velocity_mps_;
    }

    double target_angular_velocity_rad_s() const {
        return target_angular_velocity_rad_s_;
    }

private:
    static bool valid_pose(const RobotPose2D &pose) {
        return sim_finite(pose.x_m) && sim_finite(pose.y_m) && sim_finite(pose.yaw_rad);
    }

    SimRobotPlantConfig config_;
    SimRobotState state_;
    double target_linear_velocity_mps_ = 0.0;
    double target_angular_velocity_rad_s_ = 0.0;
};

} // namespace robot_slamd
