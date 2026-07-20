#pragma once

#include "robot_slamd/autonomy/ports/relative_motion_step_port.hpp"
#include "robot_slamd/simulation/robot/sim_robot_plant.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <vector>

namespace robot_slamd {

class SimRelativeMotionStepPort final : public RelativeMotionStepPort {
public:
    explicit SimRelativeMotionStepPort(std::shared_ptr<SimRobotPlant> plant,
                                       double corner_turn_rate_scale = 1.0)
        : plant_(std::move(plant)), corner_turn_rate_scale_(corner_turn_rate_scale) {}

    SimRelativeMotionStepPort(std::shared_ptr<SimRobotPlant> plant,
                              std::vector<double> corner_turn_rate_scales)
        : plant_(std::move(plant)),
          corner_turn_rate_scales_(std::move(corner_turn_rate_scales)) {}

    bool ready() const override { return plant_ && plant_->valid() && !estop_latched_; }

    std::size_t corner_turn_count() const { return corner_turn_count_; }

    RelativeMotionStepResult submit(const RelativeMotionStepCommand &command,
                                    double now_s) override {
        if (command.command_id == 0 || !std::isfinite(command.target_amount) ||
            command.target_amount <= 0.0 || !std::isfinite(command.max_rpm) ||
            command.max_rpm <= 0.0 || command.timeout_ms == 0) {
            return reject(command, "invalid_sim_motion_command");
        }
        if (!ready()) return reject(command, estop_latched_ ? "estop_latched" : "sim_motion_not_ready");
        if (active_) return reject(command, "motion_command_already_active");
        if (!relative_motion_step_is_translation(command.action) &&
            !relative_motion_step_is_rotation(command.action)) {
            return reject(command, "sim_port_accepts_motion_actions_only");
        }
        command_ = command;
        active_ = true;
        stop_requested_ = false;
        settle_frames_ = 0;
        start_s_ = now_s;
        start_left_rad_ = plant_->state().left_wheel_position_rad;
        start_right_rad_ = plant_->state().right_wheel_position_rad;
        const double wheel_speed = command.max_rpm * kSimTwoPi / 60.0;
        if (relative_motion_step_is_translation(command.action)) {
            const double sign = command.action == RelativeMotionStepAction::Back ? -1.0 : 1.0;
            target_linear_mps_ = sign * wheel_speed * plant_->config().wheel_radius_m;
            target_angular_rad_s_ = 0.0;
            duration_s_ = command.target_amount / (std::fabs(target_linear_mps_) * 1000.0);
        } else {
            const double sign = command.action == RelativeMotionStepAction::Left ? 1.0 : -1.0;
            target_linear_mps_ = 0.0;
            target_angular_rad_s_ = sign *
                (2.0 * wheel_speed * plant_->config().wheel_radius_m) /
                plant_->config().wheel_base_m;
            duration_s_ = command.target_amount * 3.14159265358979323846 / 180.0 /
                          std::fabs(target_angular_rad_s_);
            // P5 formal cases model a bounded, sensor-visible motor/traction
            // error on the single main 90 degree turn.  Duration remains the
            // nominal command duration so the controller must validate the
            // result from Core's odom yaw rather than this requested amount.
            if (std::fabs(command.target_amount - 90.0) < 1e-9) {
                double scale = corner_turn_rate_scale_;
                if (corner_turn_count_ < corner_turn_rate_scales_.size()) {
                    scale = corner_turn_rate_scales_[corner_turn_count_];
                }
                ++corner_turn_count_;
                if (std::isfinite(scale) && scale > 0.0) {
                    target_angular_rad_s_ *= scale;
                }
            }
        }
        if (!std::isfinite(duration_s_) || duration_s_ <= 0.0 ||
            !plant_->set_target_velocity(target_linear_mps_, target_angular_rad_s_)) {
            active_ = false;
            return reject(command, "sim_motion_target_failed");
        }
        return result(RelativeMotionStepState::Accepted, "motion_accepted", now_s);
    }

    // The runner owns the simulation clock and calls this before poll().
    bool advance(double dt_s, double now_s, const FakeWorld2D *world = nullptr) {
        return plant_ && plant_->step(dt_s, now_s, world);
    }

    RelativeMotionStepResult poll(double now_s) override {
        if (!active_) return last_result_;
        if (!std::isfinite(now_s) || now_s - start_s_ > command_.timeout_ms / 1000.0) {
            plant_->stop_target();
            active_ = false;
            last_result_ = result(RelativeMotionStepState::Timeout, "motion_timeout", now_s);
            return last_result_;
        }
        if (!stop_requested_ && now_s - start_s_ + 1e-9 >= duration_s_) {
            plant_->stop_target();
            stop_requested_ = true;
        }
        const bool quiet = std::fabs(plant_->state().left_wheel_speed_rad_s) < 0.20 &&
                           std::fabs(plant_->state().right_wheel_speed_rad_s) < 0.20;
        if (stop_requested_ && quiet) settle_frames_++;
        const auto state = stop_requested_
            ? (settle_frames_ >= 3 ? RelativeMotionStepState::Done
                                    : RelativeMotionStepState::Settling)
            : RelativeMotionStepState::Running;
        last_result_ = result(state, state == RelativeMotionStepState::Done
                                      ? "motor_settled" : "motion_running", now_s);
        if (state == RelativeMotionStepState::Done) {
            active_ = false;
            last_result_.motor_settled = true;
        }
        return last_result_;
    }

    RelativeMotionStepResult cancel(double now_s) override {
        if (!active_) return last_result_;
        plant_->stop_target();
        active_ = false;
        last_result_ = result(RelativeMotionStepState::Cancelled, "motion_cancelled", now_s);
        return last_result_;
    }

    RelativeMotionStepResult stop(double now_s) override {
        if (active_) {
            plant_->stop_target();
            active_ = false;
            last_result_ = result(RelativeMotionStepState::Cancelled, "stop_requested", now_s);
        } else {
            plant_->stop_target();
            last_result_ = result(RelativeMotionStepState::Done, "stop_idempotent", now_s);
        }
        return last_result_;
    }

    RelativeMotionStepResult emergency_stop(double now_s) override {
        if (plant_) plant_->stop_target();
        active_ = false;
        estop_latched_ = true;
        last_result_ = result(RelativeMotionStepState::EstopLatched, "estop_latched", now_s);
        return last_result_;
    }

    RelativeMotionStepResult clear_estop(double now_s) override {
        if (!estop_latched_) return result(RelativeMotionStepState::Done, "estop_not_latched", now_s);
        if (!plant_ || std::fabs(plant_->state().left_wheel_speed_rad_s) >= 0.20 ||
            std::fabs(plant_->state().right_wheel_speed_rad_s) >= 0.20) {
            return result(RelativeMotionStepState::Fault, "wheels_not_stopped", now_s);
        }
        estop_latched_ = false;
        last_result_ = result(RelativeMotionStepState::Done, "estop_cleared_explicitly", now_s);
        return last_result_;
    }

    std::string status() const override {
        std::ostringstream out;
        out << "sim_relative_motion ready=" << (ready() ? 1 : 0)
            << " active=" << (active_ ? 1 : 0)
            << " estop=" << (estop_latched_ ? 1 : 0);
        return out.str();
    }

private:
    RelativeMotionStepResult reject(const RelativeMotionStepCommand &command,
                                    const char *reason) {
        RelativeMotionStepResult r;
        r.command_id = command.command_id;
        r.action = command.action;
        r.requested_amount = command.target_amount;
        r.state = RelativeMotionStepState::Fault;
        r.error_code = 1;
        r.reason = reason;
        return r;
    }

    RelativeMotionStepResult result(RelativeMotionStepState state,
                                    const char *reason, double now_s) const {
        RelativeMotionStepResult r;
        r.command_id = command_.command_id;
        r.action = command_.action;
        r.state = state;
        r.requested_amount = command_.target_amount;
        r.reason = reason;
        r.final_feedback.has_wheel = true;
        r.final_feedback.has_imu = true;
        r.final_feedback.wheel.valid = true;
        r.final_feedback.wheel.timestamp_s = now_s;
        r.final_feedback.wheel.left_ticks = plant_ ? plant_->state().left_wheel_position_rad : 0.0;
        r.final_feedback.wheel.right_ticks = plant_ ? plant_->state().right_wheel_position_rad : 0.0;
        if (plant_) {
            r.final_feedback.wheel.left_rpm = plant_->state().left_wheel_speed_rad_s * 60.0 / kSimTwoPi;
            r.final_feedback.wheel.right_rpm = plant_->state().right_wheel_speed_rad_s * 60.0 / kSimTwoPi;
            const double dl = plant_->state().left_wheel_position_rad - start_left_rad_;
            const double dr = plant_->state().right_wheel_position_rad - start_right_rad_;
            r.actual_distance_mm = 0.5 * (dl + dr) * plant_->config().wheel_radius_m * 1000.0;
            r.actual_angle_deg = (dr - dl) / plant_->config().wheel_base_m * 180.0 /
                                 3.14159265358979323846;
        }
        r.final_feedback.imu.timestamp_s = now_s;
        r.final_feedback.imu.yaw_rate_rad_s = plant_ ? plant_->state().angular_velocity_rad_s : 0.0;
        return r;
    }

    std::shared_ptr<SimRobotPlant> plant_;
    RelativeMotionStepCommand command_;
    RelativeMotionStepResult last_result_;
    bool active_ = false;
    bool stop_requested_ = false;
    bool estop_latched_ = false;
    int settle_frames_ = 0;
    double start_s_ = 0.0;
    double duration_s_ = 0.0;
    double target_linear_mps_ = 0.0;
    double target_angular_rad_s_ = 0.0;
    double start_left_rad_ = 0.0;
    double start_right_rad_ = 0.0;
    double corner_turn_rate_scale_ = 1.0;
    std::vector<double> corner_turn_rate_scales_;
    std::size_t corner_turn_count_ = 0;
};

} // namespace robot_slamd
