#pragma once

#include "robot_slamd/autonomy/autonomous_slam_policy.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace robot_slamd {

struct AutonomousSlamCoordinatorOptions {
    bool enabled = false;
    int max_iterations = 100;
    double sensor_timeout_s = 0.50;
    double motion_settle_timeout_s = 1.00;
    double active_scan_speed = 0.05;
    double active_scan_duration_s = 0.50;
    int max_active_scan_commands = 24;
    bool prefer_left_turn = true;
};

class AutonomousSlamCoordinator {
public:
    AutonomousSlamCoordinator(std::shared_ptr<RobotSlamSensorPort> sensor_port,
                              std::shared_ptr<RobotSlamMotionPort> motion_port,
                              std::shared_ptr<RobotSlamMapPort> map_port,
                              AutonomousSlamCoordinatorOptions options = {})
        : sensor_port_(std::move(sensor_port)),
          motion_port_(std::move(motion_port)),
          map_port_(std::move(map_port)),
          options_(options),
          policy_(make_policy_options(options)) {}

    AutonomousSlamStepOutput step(const AutonomousSlamStepInput &input) {
        AutonomousSlamStepOutput output;
        output.state = state_;
        output.fault = fault_;

        if (!options_.enabled) {
            state_ = AutonomousSlamState::Idle;
            output.state = state_;
            output.message = "autonomous_slam_disabled";
            return output;
        }

        if (input.stop_requested) {
            send_stop(input.now_s, "stop_requested", output);
            state_ = AutonomousSlamState::Idle;
            fault_ = AutonomousSlamFault::None;
            output.state = state_;
            output.fault = fault_;
            return output;
        }

        if (iteration_count_ >= options_.max_iterations &&
            state_ != AutonomousSlamState::Completed &&
            state_ != AutonomousSlamState::Fault) {
            enter_fault(AutonomousSlamFault::MapQualityStuck,
                        "max_iterations_reached",
                        input.now_s,
                        output);
            return output;
        }

        switch (state_) {
        case AutonomousSlamState::Idle:
            if (input.start_requested) {
                state_ = AutonomousSlamState::WaitingForSensors;
                output.message = "start_requested";
            }
            break;
        case AutonomousSlamState::WaitingForSensors:
            handle_waiting_for_sensors(input, output);
            break;
        case AutonomousSlamState::Initializing:
            handle_initializing(input, output);
            break;
        case AutonomousSlamState::Mapping:
            handle_mapping(input, output);
            break;
        case AutonomousSlamState::NeedActiveScan:
            handle_need_active_scan(input, output);
            break;
        case AutonomousSlamState::SendingMotionCommand:
            handle_sending_motion_command(input, output);
            break;
        case AutonomousSlamState::WaitingMotionSettle:
            handle_waiting_motion_settle(input, output);
            break;
        case AutonomousSlamState::IntegratingScan:
            state_ = AutonomousSlamState::Mapping;
            output.message = "integrating_scan_complete";
            break;
        case AutonomousSlamState::Completed:
            output.completed = true;
            output.message = "completed";
            break;
        case AutonomousSlamState::Fault:
            send_stop(input.now_s, "fault_stop", output);
            output.message = "fault";
            break;
        }

        if (state_ != AutonomousSlamState::Idle &&
            state_ != AutonomousSlamState::Completed &&
            state_ != AutonomousSlamState::Fault) {
            iteration_count_++;
        }

        output.state = state_;
        output.fault = fault_;
        output.completed = state_ == AutonomousSlamState::Completed;
        return output;
    }

    AutonomousSlamState state() const {
        return state_;
    }

    AutonomousSlamFault fault() const {
        return fault_;
    }

    int iteration_count() const {
        return iteration_count_;
    }

private:
    static AutonomousSlamPolicyOptions make_policy_options(
        const AutonomousSlamCoordinatorOptions &options) {
        AutonomousSlamPolicyOptions policy_options;
        policy_options.active_scan_speed = options.active_scan_speed;
        policy_options.active_scan_duration_s = options.active_scan_duration_s;
        policy_options.max_active_scan_commands = options.max_active_scan_commands;
        policy_options.prefer_left_turn = options.prefer_left_turn;
        return policy_options;
    }

    RobotSlamSensorSnapshot current_snapshot(const AutonomousSlamStepInput &input) {
        if (input.sensors.has_tof || input.sensors.has_imu || input.sensors.has_wheel) {
            return input.sensors;
        }
        return sensor_port_ ? sensor_port_->latest_snapshot(input.now_s)
                            : RobotSlamSensorSnapshot{};
    }

    bool initialization_ready(const RobotSlamSensorSnapshot &snapshot) const {
        const bool has_finite_tof_time = !snapshot.has_tof || std::isfinite(snapshot.tof.timestamp_s);
        const bool has_finite_imu_time = !snapshot.has_imu || std::isfinite(snapshot.imu.timestamp_s);
        const bool has_finite_wheel_time = !snapshot.has_wheel || std::isfinite(snapshot.wheel.timestamp_s);
        return snapshot.has_tof &&
               (snapshot.has_imu || snapshot.has_wheel) &&
               has_finite_tof_time &&
               has_finite_imu_time &&
               has_finite_wheel_time;
    }

    void handle_waiting_for_sensors(const AutonomousSlamStepInput &input,
                                    AutonomousSlamStepOutput &output) {
        if (!sensor_port_ || !sensor_port_->ready()) {
            sensor_not_ready_count_++;
            output.message = "sensors_not_ready";
            return;
        }
        if (input.now_s - last_sensor_ready_time_s_ > options_.sensor_timeout_s &&
            last_sensor_ready_time_s_ > 0.0) {
            enter_fault(AutonomousSlamFault::SensorTimeout,
                        "sensor_timeout",
                        input.now_s,
                        output);
            return;
        }
        last_sensor_ready_time_s_ = input.now_s;
        state_ = AutonomousSlamState::Initializing;
        output.message = "sensors_ready";
    }

    void handle_initializing(const AutonomousSlamStepInput &input,
                             AutonomousSlamStepOutput &output) {
        const auto snapshot = current_snapshot(input);
        if (!initialization_ready(snapshot)) {
            enter_fault(AutonomousSlamFault::InitializationFailed,
                        "initialization_failed",
                        input.now_s,
                        output);
            return;
        }
        state_ = AutonomousSlamState::Mapping;
        output.message = "initialization_ok";
    }

    void handle_mapping(const AutonomousSlamStepInput &input,
                        AutonomousSlamStepOutput &output) {
        if (!map_port_ || !map_port_->ready()) {
            enter_fault(AutonomousSlamFault::InvalidStateTransition,
                        "map_port_not_ready",
                        input.now_s,
                        output);
            return;
        }

        const auto snapshot = current_snapshot(input);
        map_port_->integrate_sensor_snapshot(snapshot, input.now_s);
        const auto quality = map_port_->latest_quality(input.now_s);
        if (quality.good_enough) {
            map_quality_good_count_++;
            send_stop(input.now_s, "map_quality_good", output);
            state_ = AutonomousSlamState::Completed;
            output.message = "map_quality_good";
            return;
        }

        map_quality_poor_count_++;
        state_ = AutonomousSlamState::NeedActiveScan;
        output.message = "map_quality_poor";
    }

    void handle_need_active_scan(const AutonomousSlamStepInput &input,
                                 AutonomousSlamStepOutput &output) {
        RobotSlamMapQuality quality;
        quality.good_enough = false;
        quality.reason = "map_quality_poor";
        auto decision = policy_.decide(quality, builder_, input.now_s);
        if (decision.should_stop) {
            send_stop(input.now_s, decision.reason, output);
            if (decision.completed) {
                state_ = AutonomousSlamState::Completed;
            } else {
                enter_fault(AutonomousSlamFault::MapQualityStuck,
                            decision.reason,
                            input.now_s,
                            output);
            }
            return;
        }
        if (decision.should_send_motion) {
            pending_command_ = decision.command;
            active_scan_command_count_++;
            state_ = AutonomousSlamState::SendingMotionCommand;
            output.algorithm_command = pending_command_;
            output.message = decision.reason;
        }
    }

    void handle_sending_motion_command(const AutonomousSlamStepInput &input,
                                       AutonomousSlamStepOutput &output) {
        if (!motion_port_ || !motion_port_->ready()) {
            enter_fault(AutonomousSlamFault::MotionTransportFailed,
                        "motion_port_not_ready",
                        input.now_s,
                        output);
            return;
        }
        auto result = motion_port_->send_algorithm_command(pending_command_);
        output.algorithm_command = result.algorithm_command;
        output.software_command = result.software_command;
        if (!result.ok) {
            const auto fault = result.error == "software_motion_send_failed"
                                   ? AutonomousSlamFault::MotionTransportFailed
                                   : AutonomousSlamFault::MotionRejected;
            enter_fault(fault, result.error, input.now_s, output);
            motion_reject_count_++;
            return;
        }
        if (!result.accepted) {
            enter_fault(AutonomousSlamFault::MotionRejected,
                        result.error,
                        input.now_s,
                        output);
            motion_reject_count_++;
            return;
        }
        command_sent_count_++;
        output.command_sent = true;
        last_motion_command_time_s_ = input.now_s;
        state_ = AutonomousSlamState::WaitingMotionSettle;
        output.message = "motion_command_accepted";
    }

    void handle_waiting_motion_settle(const AutonomousSlamStepInput &input,
                                      AutonomousSlamStepOutput &output) {
        auto feedback = input.motion_feedback.timestamp_s > 0.0
                            ? input.motion_feedback
                            : (motion_port_ ? motion_port_->latest_feedback(input.now_s)
                                            : RobotSlamMotionFeedback{});
        if (feedback.fault) {
            enter_fault(AutonomousSlamFault::SafetyGateBlocked,
                        feedback.fault_reason,
                        input.now_s,
                        output);
            return;
        }
        if (feedback.command_settled) {
            state_ = AutonomousSlamState::IntegratingScan;
            output.message = "motion_settled";
            return;
        }
        if (input.now_s - last_motion_command_time_s_ > options_.motion_settle_timeout_s) {
            enter_fault(AutonomousSlamFault::SafetyGateBlocked,
                        "motion_settle_timeout",
                        input.now_s,
                        output);
        }
    }

    void send_stop(double now_s,
                   const std::string &reason,
                   AutonomousSlamStepOutput &output) {
        auto stop = builder_.stop(now_s, reason);
        output.algorithm_command = stop;
        if (motion_port_ && motion_port_->ready()) {
            auto result = motion_port_->send_algorithm_command(stop);
            output.software_command = result.software_command;
            output.command_sent = result.ok && result.accepted;
        }
        stop_sent_count_++;
    }

    void enter_fault(AutonomousSlamFault fault,
                     const std::string &message,
                     double now_s,
                     AutonomousSlamStepOutput &output) {
        state_ = AutonomousSlamState::Fault;
        fault_ = fault;
        fault_count_++;
        output.state = state_;
        output.fault = fault_;
        output.message = message;
        send_stop(now_s, "fault_stop", output);
    }

    std::shared_ptr<RobotSlamSensorPort> sensor_port_;
    std::shared_ptr<RobotSlamMotionPort> motion_port_;
    std::shared_ptr<RobotSlamMapPort> map_port_;
    AutonomousSlamCoordinatorOptions options_;
    AlgorithmMotionCommandBuilder builder_;
    AutonomousSlamPolicy policy_;
    AlgorithmMotionCommand pending_command_;
    AutonomousSlamState state_ = AutonomousSlamState::Idle;
    AutonomousSlamFault fault_ = AutonomousSlamFault::None;
    int iteration_count_ = 0;
    double last_sensor_ready_time_s_ = 0.0;
    double last_motion_command_time_s_ = 0.0;
    uint64_t command_sent_count_ = 0;
    uint64_t stop_sent_count_ = 0;
    uint64_t active_scan_command_count_ = 0;
    uint64_t sensor_not_ready_count_ = 0;
    uint64_t map_quality_good_count_ = 0;
    uint64_t map_quality_poor_count_ = 0;
    uint64_t motion_reject_count_ = 0;
    uint64_t fault_count_ = 0;
};

} // namespace robot_slamd
