#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/exploration/autonomous_exploration_controller.hpp"
#include "robot_slamd/runtime/application_mode.hpp"
#include "robot_slamd/runtime/sparse_slam_runtime_core.hpp"

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

namespace robot_slamd {

struct RobotSlamApplicationInitializationResult {
    bool ok = false;
    bool core_initialization_started = false;
    SparseSlamInitializationStatus core_status =
        SparseSlamInitializationStatus::InvalidConfiguration;
    std::string reason = "not_initialized";
};

struct RobotSlamApplicationStepResult {
    bool ok = false;
    bool terminal = false;
    bool core_step_executed = false;
    bool exploration_step_executed = false;
    std::string reason = "not_run";
};

struct RobotSlamApplicationReport {
    SensorSource sensor_source = SensorSource::Simulation;
    OperationMode operation = OperationMode::Mapping;
    bool initialized = false;
    std::size_t sensor_read_count = 0;
    std::size_t canonical_snapshot_count = 0;
    std::size_t core_step_count = 0;
    std::size_t exploration_step_count = 0;
    std::size_t rejected_step_count = 0;
    bool real_source_rejected = false;
    bool mapping_writes_enabled = true;
    std::string last_reason = "not_initialized";
};

class RobotSlamApplication final {
public:
    RobotSlamApplication(
        Config config,
        SensorSource sensor_source,
        OperationMode operation,
        std::shared_ptr<RobotSlamSensorPort> sensor,
        std::shared_ptr<RobotSlamMotionPort> motion,
        AutonomousExplorationConfig exploration_config = {})
        : config_(std::move(config)),
          sensor_source_(sensor_source),
          operation_(operation),
          sensor_(std::move(sensor)),
          motion_(std::move(motion)),
          core_(config_) {
        report_.sensor_source = sensor_source_;
        report_.operation = operation_;
        report_.mapping_writes_enabled =
            operation_ != OperationMode::Localization;
        if (operation_ == OperationMode::Exploration) {
            exploration_ = std::make_unique<AutonomousExplorationController>(
                std::move(exploration_config));
        }
    }

    RobotSlamApplicationInitializationResult initialize() {
        if (initialized_) {
            return {true, true,
                    core_.report().initialization_complete
                        ? SparseSlamInitializationStatus::Ready
                        : SparseSlamInitializationStatus::WaitingForStationarySamples,
                    "application_already_initialized"};
        }
        if (sensor_source_ == SensorSource::Real) {
            report_.real_source_rejected = true;
            report_.last_reason =
                "real_sensor_source_unavailable_fail_closed";
            return {false, false,
                    SparseSlamInitializationStatus::SensorNotReady,
                    report_.last_reason};
        }
        if (!sensor_) {
            report_.last_reason = "sensor_adapter_missing";
            return {false, false,
                    SparseSlamInitializationStatus::SensorNotReady,
                    report_.last_reason};
        }
        if (!sensor_->ready()) {
            report_.last_reason =
                "sensor_adapter_not_ready:" + sensor_->status();
            return {false, false,
                    SparseSlamInitializationStatus::SensorNotReady,
                    report_.last_reason};
        }
        if (operation_ == OperationMode::Exploration &&
            (!motion_ || !motion_->ready())) {
            report_.last_reason = motion_
                ? "motion_adapter_not_ready:" + motion_->status()
                : "motion_adapter_missing";
            return {false, false,
                    SparseSlamInitializationStatus::SensorNotReady,
                    report_.last_reason};
        }

        const auto initialized = core_.initialize(
            sparse_slam_initialization_request_from_config(config_));
        const bool started =
            initialized.status ==
                SparseSlamInitializationStatus::WaitingForStationarySamples ||
            initialized.status == SparseSlamInitializationStatus::Relocalizing ||
            initialized.status == SparseSlamInitializationStatus::Ready;
        if (!started) {
            report_.last_reason = initialized.message;
            return {false, false, initialized.status, initialized.message};
        }
        initialized_ = true;
        report_.initialized = true;
        report_.last_reason = initialized.message;
        return {true, true, initialized.status, initialized.message};
    }

    RobotSlamApplicationStepResult step(double now_s) {
        return step(now_s, ActiveObservationControl::None);
    }

    RobotSlamApplicationStepResult step(
        double now_s,
        ActiveObservationControl observation_control) {
        if (!initialized_) {
            return reject_step("application_not_initialized");
        }
        if (!std::isfinite(now_s)) {
            return reject_step("application_step_time_invalid");
        }
        if (sensor_source_ == SensorSource::Real) {
            report_.real_source_rejected = true;
            return reject_step("real_sensor_source_unavailable_fail_closed");
        }
        if (!sensor_ || !sensor_->ready()) {
            return reject_step(sensor_
                ? "sensor_adapter_not_ready:" + sensor_->status()
                : "sensor_adapter_missing");
        }

        report_.sensor_read_count++;
        const auto snapshot = sensor_->latest_snapshot(now_s);
        if (!snapshot_has_any_payload(snapshot)) {
            return reject_step("canonical_sensor_snapshot_empty");
        }
        report_.canonical_snapshot_count++;
        RobotSlamMotionFeedback feedback;
        feedback.command_settled = true;
        feedback.timestamp_s = now_s;
        if (motion_) {
            feedback = motion_->latest_feedback(now_s);
        }
        return step(snapshot, now_s, feedback, observation_control);
    }

    RobotSlamApplicationStepResult step(
        const RobotSlamSensorSnapshot &snapshot,
        double now_s,
        const RobotSlamMotionFeedback &motion_feedback = {},
        ActiveObservationControl observation_control =
            ActiveObservationControl::None) {
        if (!initialized_) {
            return reject_step("application_not_initialized");
        }
        if (!std::isfinite(now_s) || !snapshot_has_any_payload(snapshot)) {
            return reject_step("canonical_sensor_snapshot_invalid");
        }
        if (sensor_source_ == SensorSource::Real) {
            report_.real_source_rejected = true;
            return reject_step("real_sensor_source_unavailable_fail_closed");
        }

        if (operation_ == OperationMode::Exploration) {
            if (!exploration_ || !motion_) {
                return reject_step("exploration_composition_incomplete");
            }
            const auto result = exploration_->step(
                snapshot, now_s, motion_feedback, core_, *motion_);
            report_.exploration_step_count++;
            report_.core_step_count++;
            report_.last_reason = result.reason;
            return {result.ok, result.terminal, true, true, result.reason};
        }

        SparseSlamStepRequest request;
        request.observation_control = observation_control;
        request.snapshot = snapshot;
        request.now_s = now_s;
        request.mapping_write_enabled =
            operation_ == OperationMode::Mapping;
        const auto result = core_.step(request);
        report_.core_step_count++;
        report_.last_reason = result.message;
        return {result.ok, false, true, false,
                result.status + ":" + result.message};
    }

    SensorSource sensor_source() const { return sensor_source_; }
    OperationMode operation() const { return operation_; }
    const RobotSlamApplicationReport &report() const { return report_; }
    SparseSlamRuntimeCore &core() { return core_; }
    const SparseSlamRuntimeCore &core() const { return core_; }
    AutonomousExplorationController *exploration_controller() {
        return exploration_.get();
    }
    const AutonomousExplorationController *exploration_controller() const {
        return exploration_.get();
    }
    RobotSlamSensorPort *sensor_adapter() { return sensor_.get(); }
    RobotSlamMotionPort *motion_adapter() { return motion_.get(); }

private:
    RobotSlamApplicationStepResult reject_step(const std::string &reason) {
        report_.rejected_step_count++;
        report_.last_reason = reason;
        return {false, false, false, false, reason};
    }

    Config config_;
    SensorSource sensor_source_;
    OperationMode operation_;
    std::shared_ptr<RobotSlamSensorPort> sensor_;
    std::shared_ptr<RobotSlamMotionPort> motion_;
    SparseSlamRuntimeCore core_;
    std::unique_ptr<AutonomousExplorationController> exploration_;
    RobotSlamApplicationReport report_;
    bool initialized_ = false;
};

} // namespace robot_slamd
