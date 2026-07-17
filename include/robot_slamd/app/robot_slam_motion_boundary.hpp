#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/motion/algorithm_motion_command_adapter.hpp"
#include "robot_slamd/motion/motion_write_controller.hpp"
#include "robot_slamd/runtime/application_mode.hpp"
#include "robot_slamd/runtime/sparse_slam_runtime_core.hpp"
#include "robot_slamd/software_motion/software_motion_transport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace robot_slamd {

struct RobotSlamMotionBoundaryReport {
    bool unified_boundary = true;
    bool motion_safety_constructed = true;
    bool motion_write_controller_constructed = true;
    bool motion_transport_constructed = false;
    bool replay_no_motion = false;
    bool real_fail_closed = false;
    bool command_speed_used_as_odometry = false;
    std::size_t command_count = 0;
    std::size_t accepted_command_count = 0;
    std::size_t rejected_command_count = 0;
    std::size_t safety_stop_count = 0;
    std::size_t transport_reject_count = 0;
    std::size_t stale_command_count = 0;
    std::size_t deadman_reject_count = 0;
    std::size_t duration_reject_count = 0;
    std::size_t encoder_reject_count = 0;
    std::size_t localization_stop_count = 0;
    std::size_t obstacle_stop_count = 0;
    std::size_t stop_command_count = 0;
    std::size_t turn_left_command_count = 0;
    std::size_t turn_right_command_count = 0;
    std::size_t forward_command_count = 0;
    std::size_t backward_command_count = 0;
    std::string last_safety_reason = "idle";
    std::string last_transport_status = "idle";
    std::string last_state = "IDLE";
};

struct RobotSlamMotionSafetyDecision {
    MotionSafetyExecutorSnapshot snapshot;
    bool allow = false;
    bool requires_stop = false;
};

class RobotSlamMotionSafetyExecutor final {
public:
    explicit RobotSlamMotionSafetyExecutor(const Config &config)
        : config_(config) {}

    RobotSlamMotionSafetyDecision evaluate(
        const AlgorithmMotionCommand &command,
        double now_s,
        const RobotSlamSensorSnapshot &sensor_snapshot,
        bool have_sensor_snapshot,
        const SparseSlamRuntimeCore *core,
        SensorSource source,
        OperationMode operation) {
        RobotSlamMotionSafetyDecision decision;
        auto &s = decision.snapshot;
        s.timestamp_s = now_s;
        s.command_seen = true;
        s.command_source = to_string(command.profile);
        s.command_age_s = now_s - command.timestamp_s;
        s.deadman_age_s = s.command_age_s;
        s.command_fresh = std::isfinite(s.command_age_s) &&
                          s.command_age_s >= 0.0 &&
                          s.command_age_s <= command.ttl_s &&
                          s.command_age_s <=
                              config_.motion_execution_command_stale_timeout_s;
        s.deadman_ok = s.command_fresh &&
                       s.command_age_s <=
                           config_.motion_execution_deadman_timeout_s;
        s.command_duration_ok = std::isfinite(command.duration_s) &&
                                command.duration_s >= 0.0 &&
                                command.duration_s <=
                                    config_.motion_execution_max_command_duration_s;
        s.command_duration_latched = !s.command_duration_ok;
        populate_target(s, command);

        if (source == SensorSource::Real) {
            return block(decision, "real_motion_unavailable_fail_closed", false);
        }
        if (source == SensorSource::Replay && !is_algorithm_stop_kind(command.kind)) {
            return block(decision, "replay_motion_disabled", false);
        }
        if (source == SensorSource::Simulation && operation == OperationMode::Localization) {
            return block(decision, "motion_disabled_for_localization", false);
        }

        if (!s.command_fresh) {
            return block(decision, "command_stale", !is_algorithm_stop_kind(command.kind));
        }
        if (!s.deadman_ok) {
            return block(decision, "deadman_timeout", !is_algorithm_stop_kind(command.kind));
        }
        if (!s.command_duration_ok) {
            return block(decision, "command_duration_latched", !is_algorithm_stop_kind(command.kind));
        }

        if (is_algorithm_stop_kind(command.kind)) {
            s.localizer_ok = true;
            s.tof_ok = true;
            s.obstacle_ok = true;
            s.encoder_ok = true;
            s.encoder_read_ok = true;
            s.current_ok = true;
            s.status_ok = true;
            s.state = command.kind == AlgorithmMotionKind::EmergencyStop
                ? "WOULD_ZERO" : "WOULD_ZERO";
            s.would_zero = true;
            s.reason = "stop_requested";
            decision.allow = true;
            return decision;
        }

        // The order here is intentional: encoder freshness/status/current
        // precede localization, then obstacle stop, before MotionWriteController.
        if (!encoder_ok(sensor_snapshot, have_sensor_snapshot, now_s)) {
            s.encoder_ok = false;
            return block(decision, "encoder_feedback_not_recent", true);
        }
        s.encoder_ok = true;
        s.encoder_read_ok = true;
        s.status_ok = true;
        s.current_ok = true;

        const auto *core_report = core ? &core->report() : nullptr;
        const bool localization_lost = core_report &&
            (core->relocalization_required() ||
             core_report->relocalization_required ||
             core_report->localization_stop_required ||
             core_report->localization_health_state == "lost" ||
             core_report->localization_health_state == "degraded_lost");
        const bool recovery_motion =
            command.reason.find("relocalization") != std::string::npos ||
            command.reason.find("recovery") != std::string::npos ||
            command.reason.find("bootstrap") != std::string::npos;
        s.localizer_ok = !localization_lost || recovery_motion;
        if (localization_lost && !recovery_motion) {
            return block(decision, "localization_lost_recovery_stop", true);
        }

        const bool tof_recent = tof_ok(sensor_snapshot, have_sensor_snapshot, now_s);
        s.tof_ok = tof_recent;
        if (config_.motion_execution_require_tof_recent && !tof_recent) {
            return block(decision, "tof_stale", true);
        }

        const bool obstacle_safe = obstacle_ok(sensor_snapshot, command);
        s.obstacle_ok = obstacle_safe;
        if (!obstacle_safe) {
            return block(decision, "obstacle_stop", true);
        }

        s.state = "WOULD_COMMAND";
        s.would_command = true;
        s.reason = "would_command";
        decision.allow = true;
        return decision;
    }

    MotionSafetyExecutorSnapshot snapshot() const { return latest_; }

private:
    static RobotSlamMotionSafetyDecision block(
        RobotSlamMotionSafetyDecision decision,
        const std::string &reason,
        bool requires_stop) {
        decision.snapshot.state = requires_stop ? "BLOCKED" : "FAULT";
        decision.snapshot.blocked = requires_stop;
        decision.snapshot.fault = !requires_stop;
        decision.snapshot.reason = reason;
        decision.requires_stop = requires_stop;
        return decision;
    }

    void populate_target(MotionSafetyExecutorSnapshot &s,
                         const AlgorithmMotionCommand &command) const {
        double linear_mps = 0.0;
        double yaw_rate_radps = 0.0;
        const double speed = std::max(0.0, command.speed_normalized);
        if (command.kind == AlgorithmMotionKind::Forward ||
            command.kind == AlgorithmMotionKind::RecoveryForward) {
            linear_mps = speed * 0.18;
        } else if (command.kind == AlgorithmMotionKind::Backward ||
                   command.kind == AlgorithmMotionKind::RecoveryBackward) {
            linear_mps = -speed * 0.18;
        } else if (is_algorithm_rotation_kind(command.kind)) {
            yaw_rate_radps = speed * 0.90;
            if (command.kind == AlgorithmMotionKind::TurnRight ||
                command.kind == AlgorithmMotionKind::DirectionProbeRight ||
                command.kind == AlgorithmMotionKind::ActiveScanTurnRight ||
                command.kind == AlgorithmMotionKind::RecoveryTurnRight) {
                yaw_rate_radps = -yaw_rate_radps;
            }
        }
        s.desired_linear_speed_mps = linear_mps;
        s.desired_yaw_rate_radps = yaw_rate_radps;
        s.desired_yaw_rate_dps = yaw_rate_radps * 180.0 / 3.14159265358979323846;
        s.target_left_wheel_mps = linear_mps -
            0.5 * config_.motion_execution_wheel_base_m * yaw_rate_radps;
        s.target_right_wheel_mps = linear_mps +
            0.5 * config_.motion_execution_wheel_base_m * yaw_rate_radps;
        const double left_radius = config_.wheel_radius_left_m > 0.0
            ? config_.wheel_radius_left_m : 0.032;
        const double right_radius = config_.wheel_radius_right_m > 0.0
            ? config_.wheel_radius_right_m : 0.032;
        s.target_left_rpm = s.target_left_wheel_mps / (2.0 * 3.14159265358979323846 * left_radius) * 60.0;
        s.target_right_rpm = s.target_right_wheel_mps / (2.0 * 3.14159265358979323846 * right_radius) * 60.0;
    }

    bool encoder_ok(const RobotSlamSensorSnapshot &snapshot,
                    bool have_snapshot,
                    double now_s) const {
        if (!config_.motion_execution_require_encoder_feedback_recent) return true;
        if (!have_snapshot || !snapshot.has_wheel || !snapshot.wheel.valid ||
            !std::isfinite(snapshot.wheel.timestamp_s)) return false;
        const double age = now_s - snapshot.wheel.timestamp_s;
        return std::isfinite(age) && age >= 0.0 &&
               age <= config_.motion_execution_max_encoder_age_s;
    }

    bool tof_ok(const RobotSlamSensorSnapshot &snapshot,
                bool have_snapshot,
                double now_s) const {
        if (!config_.motion_execution_require_tof_recent) return true;
        if (!have_snapshot || !snapshot.has_multi_tof ||
            !snapshot.multi_tof.has_front ||
            !snapshot.multi_tof.front.protocol_valid) return false;
        const double age = now_s - snapshot.multi_tof.front.effective_timestamp_s;
        return std::isfinite(age) && age >= 0.0 &&
               age <= config_.motion_execution_max_tof_age_s;
    }

    bool obstacle_ok(const RobotSlamSensorSnapshot &snapshot,
                     const AlgorithmMotionCommand &command) const {
        if (!config_.motion_execution_obstacle_stop_enabled ||
            !is_algorithm_translation_kind(command.kind)) return true;
        if (!snapshot.has_multi_tof || !snapshot.multi_tof.has_front ||
            !snapshot.multi_tof.front.protocol_valid ||
            !snapshot.multi_tof.front.usable_for_mapping) return true;
        if (snapshot.multi_tof.front.source.find("explicit_no_return") !=
            std::string::npos) return true;
        return snapshot.multi_tof.front.distance_m >=
               config_.motion_execution_min_front_distance_m;
    }

    Config config_;
    MotionSafetyExecutorSnapshot latest_;
};

class RobotSlamMotionPortSoftwareTransport final
    : public SoftwareMotionCommandTransport {
public:
    explicit RobotSlamMotionPortSoftwareTransport(
        std::shared_ptr<RobotSlamMotionPort> downstream)
        : downstream_(std::move(downstream)) {}

    SoftwareMotionSendResult send_command(
        const SoftwareMotionCommand &command) override {
        SoftwareMotionSendResult result;
        result.timestamp_s = command.timestamp_s;
        result.transport_sequence = command.sequence;
        if (!downstream_ || !downstream_->ready()) {
            result.ok = false;
            result.error = "motion_endpoint_not_ready";
            return result;
        }
        AlgorithmMotionCommand algorithm;
        algorithm.kind = map_kind(command.direction);
        algorithm.profile = map_profile(command.source);
        algorithm.speed_normalized = command.speed_normalized;
        algorithm.duration_s = command.duration_s;
        algorithm.timestamp_s = command.timestamp_s;
        algorithm.ttl_s = command.ttl_s;
        algorithm.reason = command.reason.empty()
            ? "application_motion_transport" : command.reason;
        algorithm.sequence = command.sequence;
        const auto sent = downstream_->send_algorithm_command(algorithm);
        if (!sent.ok || !sent.accepted) {
            result.ok = true;
            result.accepted = false;
            result.error = sent.error.empty()
                ? "motion_endpoint_rejected" : sent.error;
            return result;
        }
        result.ok = true;
        result.accepted = true;
        return result;
    }

private:
    static AlgorithmMotionKind map_kind(SoftwareMotionDirection direction) {
        switch (direction) {
        case SoftwareMotionDirection::TurnLeft: return AlgorithmMotionKind::TurnLeft;
        case SoftwareMotionDirection::TurnRight: return AlgorithmMotionKind::TurnRight;
        case SoftwareMotionDirection::Forward: return AlgorithmMotionKind::Forward;
        case SoftwareMotionDirection::Backward: return AlgorithmMotionKind::Backward;
        case SoftwareMotionDirection::EmergencyStop: return AlgorithmMotionKind::EmergencyStop;
        case SoftwareMotionDirection::Stop: return AlgorithmMotionKind::Stop;
        }
        return AlgorithmMotionKind::Stop;
    }

    static AlgorithmMotionProfile map_profile(SoftwareMotionCommandSource source) {
        switch (source) {
        case SoftwareMotionCommandSource::ActiveScan: return AlgorithmMotionProfile::ActiveScan;
        case SoftwareMotionCommandSource::Recovery: return AlgorithmMotionProfile::Recovery;
        case SoftwareMotionCommandSource::ManualTest: return AlgorithmMotionProfile::ManualTest;
        case SoftwareMotionCommandSource::SafetyStop:
        case SoftwareMotionCommandSource::Unknown: return AlgorithmMotionProfile::Safety;
        }
        return AlgorithmMotionProfile::Safety;
    }

    std::shared_ptr<RobotSlamMotionPort> downstream_;
};

class NoMotionSoftwareTransport final : public SoftwareMotionCommandTransport {
public:
    SoftwareMotionSendResult send_command(
        const SoftwareMotionCommand &command) override {
        SoftwareMotionSendResult result;
        result.ok = true;
        result.timestamp_s = command.timestamp_s;
        result.transport_sequence = command.sequence;
        if (command.direction == SoftwareMotionDirection::Stop ||
            command.direction == SoftwareMotionDirection::EmergencyStop) {
            result.accepted = true;
            result.error = "no_motion_stop";
        } else {
            result.accepted = false;
            result.error = "replay_motion_disabled";
        }
        return result;
    }
};

class RobotSlamMotionCommandWriter final : public MotionCommandWriter {
public:
    explicit RobotSlamMotionCommandWriter(
        std::shared_ptr<SoftwareMotionCommandTransport> transport)
        : transport_(std::move(transport)) {}

    void set_pending(const SoftwareMotionCommand &command) {
        pending_ = command;
        has_pending_ = true;
    }

    MotionWriteResult write_zero(double timestamp_s) override {
        if (!transport_) return {false, "motion_transport_missing"};
        SoftwareMotionCommand stop;
        stop.direction = SoftwareMotionDirection::Stop;
        stop.timestamp_s = timestamp_s;
        stop.ttl_s = 0.30;
        stop.source = SoftwareMotionCommandSource::SafetyStop;
        stop.reason = "motion_write_controller_stop";
        stop.sequence = ++stop_sequence_;
        const auto sent = transport_->send_command(stop);
        return sent.ok && sent.accepted
            ? MotionWriteResult{true, ""}
            : MotionWriteResult{false, sent.error.empty() ? "motion_stop_rejected" : sent.error};
    }

    MotionWriteResult write_wheel_rpm(double timestamp_s,
                                      double left_rpm,
                                      double right_rpm) override {
        (void)timestamp_s;
        (void)left_rpm;
        (void)right_rpm;
        if (!transport_) return {false, "motion_transport_missing"};
        if (!has_pending_) return {false, "motion_pending_command_missing"};
        const auto sent = transport_->send_command(pending_);
        return sent.ok && sent.accepted
            ? MotionWriteResult{true, ""}
            : MotionWriteResult{false, sent.error.empty() ? "motion_command_rejected" : sent.error};
    }

private:
    std::shared_ptr<SoftwareMotionCommandTransport> transport_;
    SoftwareMotionCommand pending_;
    bool has_pending_ = false;
    uint64_t stop_sequence_ = 0;
};

class RobotSlamMotionBoundary final : public RobotSlamMotionPort {
public:
    RobotSlamMotionBoundary(
        Config config,
        SensorSource source,
        OperationMode operation,
        std::shared_ptr<RobotSlamMotionPort> downstream)
        : config_(std::move(config)),
          source_(source),
          operation_(operation),
          downstream_(std::move(downstream)),
          transport_(make_transport(source_, downstream_)),
          adapter_(make_adapter(source_, operation_)),
          safety_(config_),
          writer_(transport_) {
        report_.replay_no_motion = source_ == SensorSource::Replay;
        report_.real_fail_closed = source_ == SensorSource::Real;
        report_.motion_transport_constructed = static_cast<bool>(transport_);
    }

    bool ready() const override {
        if (source_ == SensorSource::Real) return false;
        if (source_ == SensorSource::Replay) return true;
        return downstream_ && downstream_->ready();
    }

    void update_context(const RobotSlamSensorSnapshot &snapshot,
                        double now_s,
                        const RobotSlamMotionFeedback &feedback,
                        const SparseSlamRuntimeCore &core) {
        context_snapshot_ = snapshot;
        context_now_s_ = now_s;
        context_feedback_ = feedback;
        context_core_ = &core;
        have_context_ = true;
    }

    AlgorithmMotionFacadeResult send_algorithm_command(
        const AlgorithmMotionCommand &command) override {
        report_.command_count++;
        report_.last_state = "PRECHECK";
        AlgorithmMotionFacadeResult result;
        result.algorithm_command = command;
        const double now_s = have_context_ && std::isfinite(context_now_s_)
            ? std::max(context_now_s_, command.timestamp_s)
            : command.timestamp_s;
        if (source_ == SensorSource::Real) {
            return reject(result, "real_motion_unavailable_fail_closed", false, now_s);
        }
        if (source_ == SensorSource::Replay &&
            !is_algorithm_stop_kind(command.kind)) {
            return reject(result, "replay_motion_disabled", false, now_s);
        }
        const auto converted = adapter_.to_software_command(command);
        if (!converted.ok) {
            return reject(result, converted.error, false, now_s);
        }
        result.software_command = converted.command;
        const auto decision = safety_.evaluate(
            command, now_s, context_snapshot_, have_context_,
            context_core_, source_, operation_);
        report_.last_safety_reason = decision.snapshot.reason;
        report_.last_state = decision.snapshot.state;
        if (!decision.allow) {
            const bool should_stop = decision.requires_stop &&
                                     !is_algorithm_stop_kind(command.kind);
            auto rejected = reject(result, decision.snapshot.reason,
                                   should_stop, now_s);
            if (decision.snapshot.reason == "command_stale") report_.stale_command_count++;
            if (decision.snapshot.reason == "deadman_timeout") report_.deadman_reject_count++;
            if (decision.snapshot.reason == "command_duration_latched") report_.duration_reject_count++;
            if (decision.snapshot.reason == "encoder_feedback_not_recent") report_.encoder_reject_count++;
            if (decision.snapshot.reason == "localization_lost_recovery_stop") report_.localization_stop_count++;
            if (decision.snapshot.reason == "obstacle_stop") report_.obstacle_stop_count++;
            return rejected;
        }

        writer_.set_pending(converted.command);
        const auto write = write_controller_.update(
            decision.snapshot, writer_, dispatch_allowed());
        if ((!write.wrote_rpm && !write.wrote_zero) || write.writer_error) {
            report_.transport_reject_count++;
            auto rejected = reject(result,
                                   write.error.empty() ? write.reason : write.error,
                                   true, now_s);
            report_.last_transport_status = write.reason;
            return rejected;
        }

        report_.accepted_command_count++;
        report_.last_transport_status = write.reason;
        result.send_result.ok = true;
        result.send_result.accepted = true;
        result.send_result.timestamp_s = command.timestamp_s;
        result.send_result.transport_sequence = command.sequence;
        result.ok = true;
        result.accepted = true;
        count_command(command);
        return result;
    }

    RobotSlamMotionFeedback latest_feedback(double now_s) override {
        if (downstream_) {
            return downstream_->latest_feedback(now_s);
        }
        RobotSlamMotionFeedback feedback;
        feedback.command_settled = true;
        feedback.fault = source_ == SensorSource::Real;
        feedback.fault_reason = feedback.fault ? "real_motion_unavailable_fail_closed" : "";
        feedback.timestamp_s = now_s;
        return feedback;
    }

    std::string status() const override {
        std::ostringstream out;
        out << "application_motion_boundary ready=" << (ready() ? 1 : 0)
            << " safety=1 write_controller=1 transport="
            << (transport_ ? 1 : 0)
            << " accepted=" << report_.accepted_command_count
            << " rejected=" << report_.rejected_command_count
            << " last=" << report_.last_safety_reason;
        return out.str();
    }

    const RobotSlamMotionBoundaryReport &report() const { return report_; }
    const MotionWriteController &write_controller() const { return write_controller_; }

private:
    static std::shared_ptr<SoftwareMotionCommandTransport> make_transport(
        SensorSource source,
        const std::shared_ptr<RobotSlamMotionPort> &downstream) {
        if (source == SensorSource::Replay) {
            return std::make_shared<NoMotionSoftwareTransport>();
        }
        if (source == SensorSource::Simulation && downstream) {
            return std::make_shared<RobotSlamMotionPortSoftwareTransport>(downstream);
        }
        return nullptr;
    }

    static AlgorithmMotionCommandAdapter make_adapter(
        SensorSource source,
        OperationMode operation) {
        AlgorithmToSoftwareMotionOptions options;
        options.allow_translation_commands =
            source == SensorSource::Simulation && operation == OperationMode::Exploration;
        options.allow_rotation_commands = source == SensorSource::Simulation;
        options.allow_recovery_commands =
            source == SensorSource::Simulation && operation == OperationMode::Exploration;
        options.allow_manual_test_commands = source == SensorSource::Simulation;
        options.max_direction_probe_speed = 0.40;
        options.max_active_scan_speed = 0.40;
        options.max_recovery_speed = 0.40;
        options.max_manual_test_speed = 0.40;
        options.max_direction_probe_duration_s = 0.50;
        options.max_active_scan_duration_s = 0.50;
        options.max_recovery_duration_s = 0.50;
        options.max_manual_test_duration_s = 0.50;
        return AlgorithmMotionCommandAdapter(options);
    }

    bool dispatch_allowed() const {
        return source_ != SensorSource::Real && transport_ && ready();
    }

    AlgorithmMotionFacadeResult reject(AlgorithmMotionFacadeResult result,
                                       const std::string &reason,
                                       bool send_stop,
                                       double now_s) {
        report_.rejected_command_count++;
        report_.last_safety_reason = reason;
        result.error = reason;
        result.send_result.ok = false;
        result.send_result.accepted = false;
        if (send_stop) {
            report_.safety_stop_count++;
            AlgorithmMotionCommand stop;
            stop.kind = AlgorithmMotionKind::Stop;
            stop.profile = AlgorithmMotionProfile::Safety;
            stop.speed_normalized = 0.0;
            stop.duration_s = 0.0;
            stop.timestamp_s = std::isfinite(now_s) ? now_s : 0.0;
            stop.ttl_s = std::max(0.30, config_.motion_execution_deadman_timeout_s);
            stop.reason = "application_safety_stop:" + reason;
            stop.sequence = ++safety_stop_sequence_;
            writer_.set_pending(SoftwareMotionCommand{});
            MotionSafetyExecutorSnapshot stop_snapshot;
            stop_snapshot.timestamp_s = stop.timestamp_s;
            stop_snapshot.state = "WOULD_ZERO";
            stop_snapshot.would_zero = true;
            stop_snapshot.reason = stop.reason;
            const auto write = write_controller_.update(
                stop_snapshot, writer_, dispatch_allowed());
            if (!write.wrote_zero && dispatch_allowed()) {
                report_.last_transport_status = write.error.empty()
                    ? write.reason : write.error;
            }
        }
        return result;
    }

    void count_command(const AlgorithmMotionCommand &command) {
        switch (command.kind) {
        case AlgorithmMotionKind::Stop:
        case AlgorithmMotionKind::EmergencyStop:
            report_.stop_command_count++;
            break;
        case AlgorithmMotionKind::TurnLeft:
        case AlgorithmMotionKind::DirectionProbeLeft:
        case AlgorithmMotionKind::ActiveScanTurnLeft:
        case AlgorithmMotionKind::RecoveryTurnLeft:
            report_.turn_left_command_count++;
            break;
        case AlgorithmMotionKind::TurnRight:
        case AlgorithmMotionKind::DirectionProbeRight:
        case AlgorithmMotionKind::ActiveScanTurnRight:
        case AlgorithmMotionKind::RecoveryTurnRight:
            report_.turn_right_command_count++;
            break;
        case AlgorithmMotionKind::Forward:
        case AlgorithmMotionKind::RecoveryForward:
            report_.forward_command_count++;
            break;
        case AlgorithmMotionKind::Backward:
        case AlgorithmMotionKind::RecoveryBackward:
            report_.backward_command_count++;
            break;
        }
    }

    Config config_;
    SensorSource source_;
    OperationMode operation_;
    std::shared_ptr<RobotSlamMotionPort> downstream_;
    std::shared_ptr<SoftwareMotionCommandTransport> transport_;
    AlgorithmMotionCommandAdapter adapter_;
    RobotSlamMotionSafetyExecutor safety_;
    MotionWriteController write_controller_;
    RobotSlamMotionCommandWriter writer_;
    RobotSlamSensorSnapshot context_snapshot_;
    RobotSlamMotionFeedback context_feedback_;
    const SparseSlamRuntimeCore *context_core_ = nullptr;
    double context_now_s_ = 0.0;
    bool have_context_ = false;
    uint64_t safety_stop_sequence_ = 0;
    RobotSlamMotionBoundaryReport report_;
};

} // namespace robot_slamd
