#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/motion/algorithm_motion_command_adapter.hpp"
#include "robot_slamd/simulation/robot/sim_robot_plant.hpp"

#include <cmath>
#include <memory>
#include <sstream>

namespace robot_slamd {

struct SimMotionPortConfig {
    bool allow_translation = false;
    double max_linear_speed_mps = 0.18;
    double max_angular_speed_rad_s = 0.90;
    double settle_linear_speed_mps = 0.005;
    double settle_angular_speed_rad_s = 0.01;
    int settle_required_ticks = 3;
    double maximum_algorithm_speed_normalized = 0.40;
    double maximum_algorithm_duration_s = 0.50;
};

class SimMotionPort final : public RobotSlamMotionPort {
public:
    SimMotionPort(std::shared_ptr<SimRobotPlant> plant,
                  SimMotionPortConfig config = {})
        : plant_(std::move(plant)), config_(config) {
        AlgorithmToSoftwareMotionOptions options;
        options.allow_translation_commands = config_.allow_translation;
        options.allow_rotation_commands = true;
        options.allow_manual_test_commands = true;
        options.max_manual_test_speed = config_.maximum_algorithm_speed_normalized;
        options.max_manual_test_duration_s = config_.maximum_algorithm_duration_s;
        adapter_ = AlgorithmMotionCommandAdapter(options);
    }

    bool ready() const override {
        return plant_ && plant_->valid() &&
               sim_finite(config_.max_linear_speed_mps) &&
               sim_finite(config_.max_angular_speed_rad_s) &&
               config_.max_linear_speed_mps >= 0.0 &&
               config_.max_angular_speed_rad_s >= 0.0 &&
               sim_finite(config_.settle_linear_speed_mps) &&
               sim_finite(config_.settle_angular_speed_rad_s) &&
               config_.settle_linear_speed_mps >= 0.0 &&
               config_.settle_angular_speed_rad_s >= 0.0 &&
               config_.settle_required_ticks > 0 &&
               sim_finite(config_.maximum_algorithm_speed_normalized) &&
               config_.maximum_algorithm_speed_normalized > 0.0 &&
               config_.maximum_algorithm_speed_normalized <= 1.0 &&
               sim_finite(config_.maximum_algorithm_duration_s) &&
               config_.maximum_algorithm_duration_s > 0.0;
    }

    AlgorithmMotionFacadeResult send_algorithm_command(
        const AlgorithmMotionCommand &command) override {
        AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;
        if (!ready()) {
            result.error = "sim_motion_not_ready";
            reject_count_++;
            return result;
        }
        if (command.timestamp_s + command.ttl_s < now_s_) {
            result.error = "sim_motion_command_stale";
            reject_count_++;
            return result;
        }
        auto converted = adapter_.to_software_command(command);
        if (!converted.ok) {
            result.error = converted.error;
            reject_count_++;
            return result;
        }
        result.software_command = converted.command;
        result.ok = true;
        result.accepted = true;
        accepted_count_++;
        last_command_ = command;
        last_software_command_ = converted.command;

        if (command.kind == AlgorithmMotionKind::EmergencyStop) {
            emergency_stop_active_ = true;
            active_ = false;
            plant_->stop_target();
            settle_ticks_ = 0;
            emergency_stop_count_++;
            return result;
        }
        if (emergency_stop_active_ && command.kind != AlgorithmMotionKind::Stop) {
            result.ok = false;
            result.accepted = false;
            result.error = "sim_motion_emergency_stop_latched";
            reject_count_++;
            accepted_count_--;
            return result;
        }
        if (command.kind == AlgorithmMotionKind::Stop) {
            active_ = false;
            plant_->stop_target();
            settle_ticks_ = 0;
            stop_count_++;
            return result;
        }

        active_ = true;
        command_start_s_ = now_s_;
        command_duration_s_ = command.duration_s;
        settle_ticks_ = 0;
        const double speed = std::max(0.0, command.speed_normalized);
        switch (converted.command.direction) {
        case SoftwareMotionDirection::TurnLeft:
            plant_->set_target_velocity(0.0, speed * config_.max_angular_speed_rad_s);
            turn_left_count_++;
            break;
        case SoftwareMotionDirection::TurnRight:
            plant_->set_target_velocity(0.0, -speed * config_.max_angular_speed_rad_s);
            turn_right_count_++;
            break;
        case SoftwareMotionDirection::Forward:
            plant_->set_target_velocity(speed * config_.max_linear_speed_mps, 0.0);
            forward_count_++;
            break;
        case SoftwareMotionDirection::Backward:
            plant_->set_target_velocity(-speed * config_.max_linear_speed_mps, 0.0);
            backward_count_++;
            break;
        case SoftwareMotionDirection::Stop:
        case SoftwareMotionDirection::EmergencyStop:
            plant_->stop_target();
            active_ = false;
            break;
        }
        return result;
    }

    RobotSlamMotionFeedback latest_feedback(double now_s) override {
        update(now_s);
        RobotSlamMotionFeedback feedback;
        feedback.command_active = active_;
        feedback.command_settled = command_settled_;
        feedback.emergency_stop_active = emergency_stop_active_;
        feedback.fault = false;
        feedback.timestamp_s = now_s_;
        return feedback;
    }

    std::string status() const override {
        std::ostringstream out;
        out << "sim_motion ready=" << (ready() ? 1 : 0)
            << " active=" << (active_ ? 1 : 0)
            << " settled=" << (command_settled_ ? 1 : 0)
            << " accepted=" << accepted_count_
            << " rejected=" << reject_count_;
        return out.str();
    }

    void update(double now_s) {
        if (!sim_finite(now_s) || now_s < now_s_) {
            return;
        }
        now_s_ = now_s;
        if (active_ && now_s_ - command_start_s_ >= command_duration_s_) {
            active_ = false;
            if (plant_) {
                plant_->stop_target();
            }
        }
        const auto &state = plant_->state();
        const bool speed_settled =
            std::fabs(state.linear_velocity_mps) <= config_.settle_linear_speed_mps &&
            std::fabs(state.angular_velocity_rad_s) <= config_.settle_angular_speed_rad_s;
        if (!active_ && speed_settled) {
            settle_ticks_++;
        } else {
            settle_ticks_ = 0;
        }
        command_settled_ = !active_ && settle_ticks_ >= config_.settle_required_ticks;
    }

    bool command_active() const { return active_; }
    bool command_settled() const { return command_settled_; }
    bool emergency_stop_active() const { return emergency_stop_active_; }
    int accepted_count() const { return accepted_count_; }
    int reject_count() const { return reject_count_; }
    int turn_left_count() const { return turn_left_count_; }
    int turn_right_count() const { return turn_right_count_; }
    int forward_count() const { return forward_count_; }
    int backward_count() const { return backward_count_; }
    int stop_count() const { return stop_count_; }
    int emergency_stop_count() const { return emergency_stop_count_; }
    bool allow_translation() const { return config_.allow_translation; }

private:
    std::shared_ptr<SimRobotPlant> plant_;
    SimMotionPortConfig config_;
    AlgorithmMotionCommandAdapter adapter_;
    AlgorithmMotionCommand last_command_;
    SoftwareMotionCommand last_software_command_;
    double now_s_ = 0.0;
    double command_start_s_ = 0.0;
    double command_duration_s_ = 0.0;
    bool active_ = false;
    bool command_settled_ = true;
    bool emergency_stop_active_ = false;
    int settle_ticks_ = 0;
    int accepted_count_ = 0;
    int reject_count_ = 0;
    int turn_left_count_ = 0;
    int turn_right_count_ = 0;
    int forward_count_ = 0;
    int backward_count_ = 0;
    int stop_count_ = 0;
    int emergency_stop_count_ = 0;
};

} // namespace robot_slamd
