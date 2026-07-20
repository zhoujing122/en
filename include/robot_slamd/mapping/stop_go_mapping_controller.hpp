#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/mapping/left_wall_line_estimator.hpp"
#include "robot_slamd/autonomy/ports/relative_motion_step_port.hpp"
#include "robot_slamd/runtime/mapping_settle_gate.hpp"
#include "robot_slamd/sensors/stable_three_tof_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace robot_slamd {

enum class StopGoMappingControllerState {
    Idle,
    WaitingForCoreReady,
    InitialSettle,
    InitialSample,
    WallAcquisition,
    IssueForwardStep,
    WaitMotionDone,
    WaitMappingSettle,
    AcquireThreeTof,
    CommitMappingSample,
    IssueTurnCorrection,
    WaitTurnDone,
    WaitTurnSettle,
    VerifyAfterTurn,
    CheckLimit,
    Completed,
    WallLost,
    FrontBlocked,
    Fault,
    Estop
};

inline const char *to_string(StopGoMappingControllerState state) {
    switch (state) {
    case StopGoMappingControllerState::Idle: return "IDLE";
    case StopGoMappingControllerState::WaitingForCoreReady: return "WAITING_FOR_CORE_READY";
    case StopGoMappingControllerState::InitialSettle: return "INITIAL_SETTLE";
    case StopGoMappingControllerState::InitialSample: return "INITIAL_SAMPLE";
    case StopGoMappingControllerState::WallAcquisition: return "WALL_ACQUISITION";
    case StopGoMappingControllerState::IssueForwardStep: return "ISSUE_FORWARD_STEP";
    case StopGoMappingControllerState::WaitMotionDone: return "WAIT_MOTION_DONE";
    case StopGoMappingControllerState::WaitMappingSettle: return "WAIT_MAPPING_SETTLE";
    case StopGoMappingControllerState::AcquireThreeTof: return "ACQUIRE_THREE_TOF";
    case StopGoMappingControllerState::CommitMappingSample: return "COMMIT_MAPPING_SAMPLE";
    case StopGoMappingControllerState::IssueTurnCorrection: return "ISSUE_TURN_CORRECTION";
    case StopGoMappingControllerState::WaitTurnDone: return "WAIT_TURN_DONE";
    case StopGoMappingControllerState::WaitTurnSettle: return "WAIT_TURN_SETTLE";
    case StopGoMappingControllerState::VerifyAfterTurn: return "VERIFY_AFTER_TURN";
    case StopGoMappingControllerState::CheckLimit: return "CHECK_LIMIT";
    case StopGoMappingControllerState::Completed: return "COMPLETED";
    case StopGoMappingControllerState::WallLost: return "WALL_LOST";
    case StopGoMappingControllerState::FrontBlocked: return "FRONT_BLOCKED";
    case StopGoMappingControllerState::Fault: return "FAULT";
    case StopGoMappingControllerState::Estop: return "ESTOP";
    }
    return "UNKNOWN";
}

struct StopGoMappingControllerConfig {
    std::string mode = "straight";
    double forward_step_mm = 20.0;
    int max_steps = 20;
    double max_total_distance_mm = 400.0;
    double motion_max_rpm = 12.0;
    std::uint64_t motion_timeout_ms = 3000;
    MappingSettleGateConfig settle;
    StableThreeTofSamplerConfig tof;
    std::string log_path;
    std::string desired_distance_mode = "configured";
    double desired_base_to_wall_distance_m = 0.15;
    LeftWallLineEstimatorConfig wall_estimator;
    std::size_t wall_acquisition_max_forward_steps = 6;
    std::size_t max_stationary_resample_attempts = 3;
    std::size_t max_consecutive_wall_misses = 2;
    double heading_deadband_rad = 1.0 * 3.14159265358979323846 / 180.0;
    double distance_deadband_m = 0.010;
    double distance_to_heading_gain_rad_per_m = 20.0 * 3.14159265358979323846 / 180.0;
    double max_distance_bias_rad = 3.0 * 3.14159265358979323846 / 180.0;
    double min_correction_deg = 1.0;
    double max_correction_deg = 3.0;
    double min_allowed_base_to_wall_distance_m = 0.05;
    double max_allowed_base_to_wall_distance_m = 0.50;
    double front_stop_threshold_m = 0.20;
    std::size_t front_max_invalid_samples = 2;
    RobotPose2D base_T_left_tof;
};

inline StopGoMappingControllerConfig stop_go_mapping_config_from(
    const Config &config) {
    StopGoMappingControllerConfig result;
    result.forward_step_mm = config.stop_go_forward_step_mm;
    result.max_steps = config.stop_go_max_steps;
    result.max_total_distance_mm = config.stop_go_max_total_distance_mm;
    result.motion_max_rpm = config.stop_go_motion_max_rpm;
    result.motion_timeout_ms = static_cast<std::uint64_t>(config.stop_go_motion_timeout_ms);
    result.settle.wheel_rpm_threshold = config.stop_go_settle_wheel_rpm_threshold;
    result.settle.imu_gyro_threshold_deg_s = config.stop_go_settle_imu_gyro_threshold_deg_s;
    result.settle.consecutive_frames = static_cast<std::size_t>(config.stop_go_settle_consecutive_frames);
    result.settle.hold_ms = static_cast<std::uint64_t>(config.stop_go_settle_hold_ms);
    result.tof.samples_per_sensor = static_cast<std::size_t>(config.stop_go_tof_samples_per_sensor);
    result.tof.min_valid_samples = static_cast<std::size_t>(config.stop_go_tof_min_valid_samples);
    result.tof.protocol_min_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_protocol_min_distance_mm);
    result.tof.protocol_max_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_protocol_max_distance_mm);
    result.tof.mapping_min_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_mapping_min_distance_mm);
    result.tof.mapping_max_distance_mm = static_cast<std::uint16_t>(config.stop_go_tof_mapping_max_distance_mm);
    result.tof.mapping_min_confidence = static_cast<std::uint8_t>(config.stop_go_tof_mapping_min_confidence);
    result.log_path = config.stop_go_log_path;
    result.mode = config.stop_go_mapping_mode;
    result.desired_distance_mode = config.stop_go_desired_distance_mode;
    result.desired_base_to_wall_distance_m =
        config.stop_go_desired_base_to_wall_distance_mm / 1000.0;
    result.wall_estimator.window_max_points =
        static_cast<std::size_t>(config.stop_go_wall_window_max_points);
    result.wall_estimator.min_fit_points =
        static_cast<std::size_t>(config.stop_go_wall_min_fit_points);
    result.wall_estimator.min_baseline_m =
        config.stop_go_wall_min_baseline_mm / 1000.0;
    result.wall_estimator.max_fit_rms_m =
        config.stop_go_wall_max_fit_rms_mm / 1000.0;
    result.wall_estimator.outlier_min_threshold_m =
        config.stop_go_wall_outlier_min_threshold_mm / 1000.0;
    result.wall_estimator.outlier_mad_scale = config.stop_go_wall_outlier_mad_scale;
    result.wall_acquisition_max_forward_steps = static_cast<std::size_t>(
        config.stop_go_wall_acquisition_max_forward_steps);
    result.max_stationary_resample_attempts = static_cast<std::size_t>(
        config.stop_go_wall_max_stationary_resample_attempts);
    result.max_consecutive_wall_misses = static_cast<std::size_t>(
        config.stop_go_wall_max_consecutive_misses);
    result.heading_deadband_rad = config.stop_go_wall_heading_deadband_deg *
        3.14159265358979323846 / 180.0;
    result.distance_deadband_m = config.stop_go_wall_distance_deadband_mm / 1000.0;
    result.distance_to_heading_gain_rad_per_m =
        config.stop_go_wall_distance_to_heading_gain_deg_per_m *
        3.14159265358979323846 / 180.0;
    result.max_distance_bias_rad = config.stop_go_wall_max_distance_bias_deg *
        3.14159265358979323846 / 180.0;
    result.min_correction_deg = config.stop_go_wall_min_correction_deg;
    result.max_correction_deg = config.stop_go_wall_max_correction_deg;
    result.min_allowed_base_to_wall_distance_m =
        config.stop_go_wall_min_allowed_distance_mm / 1000.0;
    result.max_allowed_base_to_wall_distance_m =
        config.stop_go_wall_max_allowed_distance_mm / 1000.0;
    result.front_stop_threshold_m = config.stop_go_front_stop_threshold_mm / 1000.0;
    result.front_max_invalid_samples = static_cast<std::size_t>(
        config.stop_go_front_max_invalid_samples);
    result.base_T_left_tof = RobotPose2D{
        config.sparse_slam_left_tof_x_m,
        config.sparse_slam_left_tof_y_m,
        config.sparse_slam_left_tof_yaw_rad};
    return result;
}

enum class LeftWallControlAction { NoTurn, Left, Right };

struct LeftWallControlDecision {
    LeftWallControlAction action = LeftWallControlAction::NoTurn;
    double heading_error_rad = 0.0;
    double distance_error_m = 0.0;
    double distance_bias_rad = 0.0;
    double turn_error_rad = 0.0;
    double correction_amount_deg = 0.0;
    std::string reason = "not_decided";
};

struct StopGoReplayOdomSample {
    WheelOdomFrame wheel;
    ImuFrame imu;
};

struct StopGoReplayRecord {
    std::size_t cycle_index = 0;
    std::uint64_t command_id = 0;
    RelativeMotionStepResult motion;
    StableThreeTofSample stable_sample;
    RobotSlamSensorSnapshot snapshot;
    std::uint64_t map_revision_before = 0;
    std::uint64_t map_revision_after = 0;
    std::uint64_t motor_settled_timestamp_us = 0;
    std::uint64_t mapping_ready_timestamp_us = 0;
    std::size_t settle_frame_count = 0;
    double settle_duration_ms = 0.0;
    // Read-only audit values copied from the canonical Core report.  The
    // controller never writes either state back into SLAM.
    RobotPose2D odom_pose;
    RobotPose2D map_from_odom;
    StopGoMappingControllerState controller_state = StopGoMappingControllerState::Idle;
    bool left_wall_point_accepted = false;
    LeftWallPoint left_wall_point;
    LeftWallModel wall_model;
    LeftWallControlDecision control_decision;
    bool post_turn_verified = false;
    std::uint64_t frame_transform_epoch = 0;
    std::vector<StopGoReplayOdomSample> odom_samples;
};

inline LeftWallControlDecision decide_left_wall_control(
    const LeftWallModel &wall,
    const RobotPose2D &robot_pose,
    double desired_distance_m,
    const StopGoMappingControllerConfig &config) {
    LeftWallControlDecision result;
    if (!wall.valid || !sparse_slam_pose_finite(robot_pose) ||
        !std::isfinite(desired_distance_m) || desired_distance_m <= 0.0) {
        result.reason = "invalid_wall_or_pose";
        return result;
    }
    result.heading_error_rad = sparse_slam_shortest_yaw_delta(
        robot_pose.yaw_rad, wall.wall_heading_rad);
    result.distance_error_m =
        wall.signed_base_to_wall_distance_m - desired_distance_m;
    if (std::fabs(result.distance_error_m) <= config.distance_deadband_m) {
        result.distance_bias_rad = 0.0;
    } else {
        result.distance_bias_rad = std::clamp(
            config.distance_to_heading_gain_rad_per_m * result.distance_error_m,
            -config.max_distance_bias_rad, config.max_distance_bias_rad);
    }
    const double desired_heading = sparse_slam_normalize_yaw(
        wall.wall_heading_rad + result.distance_bias_rad);
    result.turn_error_rad = sparse_slam_shortest_yaw_delta(
        robot_pose.yaw_rad, desired_heading);
    const double turn_deg = std::fabs(result.turn_error_rad) *
        180.0 / 3.14159265358979323846;
    if (result.turn_error_rad > config.heading_deadband_rad) {
        result.action = LeftWallControlAction::Left;
        result.reason = "left_correction_required";
    } else if (result.turn_error_rad < -config.heading_deadband_rad) {
        result.action = LeftWallControlAction::Right;
        result.reason = "right_correction_required";
    } else {
        result.reason = "heading_and_distance_within_deadband";
    }
    if (result.action != LeftWallControlAction::NoTurn) {
        result.correction_amount_deg = std::clamp(
            turn_deg, config.min_correction_deg, config.max_correction_deg);
    }
    return result;
}

inline const char *to_string(LeftWallControlAction action) {
    switch (action) {
    case LeftWallControlAction::NoTurn: return "NO_TURN";
    case LeftWallControlAction::Left: return "LEFT";
    case LeftWallControlAction::Right: return "RIGHT";
    }
    return "UNKNOWN";
}

struct StopGoMappingRunReport {
    bool ok = false;
    std::size_t completed_steps = 0;
    std::size_t commands_submitted = 0;
    std::size_t commands_completed = 0;
    std::size_t forward_command_count = 0;
    std::size_t correction_command_count = 0;
    std::size_t left_correction_count = 0;
    std::size_t right_correction_count = 0;
    std::size_t wall_acquisition_steps = 0;
    std::size_t wall_model_valid_count = 0;
    std::size_t wall_model_reset_count = 0;
    std::size_t wall_model_reset_due_to_frame_change_count = 0;
    std::size_t post_turn_verification_count = 0;
    std::size_t post_turn_verification_failure_count = 0;
    std::size_t stable_samples = 0;
    std::size_t map_commits = 0;
    std::uint64_t map_revision = 0;
    std::size_t map_cells = 0;
    std::size_t left_wall_observed_cells = 0;
    std::size_t collision_count = 0;
    std::size_t map_writes_while_moving = 0;
    std::size_t map_writes_during_turn_or_verify = 0;
    std::size_t core_wait_ticks = 0;
    bool core_ready_observed = false;
    bool command_speed_used_as_odometry = false;
    bool ground_truth_used_by_algorithm = false;
    bool command_target_used_as_odometry = false;
    std::size_t wall_fit_input_points = 0;
    std::size_t wall_fit_inliers = 0;
    double wall_fit_baseline_m = 0.0;
    double wall_fit_rms_m = 0.0;
    double initial_heading_error_deg = 0.0;
    double final_heading_error_deg = 0.0;
    double initial_distance_error_mm = 0.0;
    double final_distance_error_mm = 0.0;
    double maximum_abs_heading_error_deg = 0.0;
    double maximum_abs_distance_error_mm = 0.0;
    RobotPose2D final_estimated_pose;
    std::uint64_t frame_transform_epoch = 0;
    std::uint64_t wall_point_sequence_hash = 1469598103934665603ULL;
    std::uint64_t wall_model_sequence_hash = 1469598103934665603ULL;
    std::uint64_t final_map_checksum = 0;
    std::string termination_reason = "not_started";
    std::string log_error;
    std::vector<std::string> state_trace;
    std::vector<std::uint64_t> command_ids;
    std::vector<StopGoReplayRecord> replay_records;
};

class StopGoMappingController final {
public:
    using SnapshotReader = std::function<RobotSlamSensorSnapshot(double)>;

    StopGoMappingController(RobotSlamApplication &application,
                            RelativeMotionStepPort &motion,
                            SnapshotReader snapshot_reader,
                            StableThreeTofSampler sampler,
                            StopGoMappingControllerConfig config,
                            std::string log_path = {})
        : application_(application), motion_(motion),
          snapshot_reader_(std::move(snapshot_reader)), sampler_(std::move(sampler)),
          config_(std::move(config)), gate_(config_.settle),
          wall_estimator_(config_.wall_estimator) {
        if (!log_path.empty()) config_.log_path = std::move(log_path);
        desired_distance_m_ = config_.desired_base_to_wall_distance_m;
        if (!config_.log_path.empty()) log_.open(config_.log_path, std::ios::out | std::ios::trunc);
        // Core startup owns gyro calibration, wheel baseline, odom_T_base,
        // map_T_odom and the pose buffer.  Stop-go cannot enter its formal
        // state machine until Core explicitly reports localization_ready().
        transition(StopGoMappingControllerState::WaitingForCoreReady);
    }

    bool tick(double now_s) {
        if (left_wall_mode()) return tick_left_wall(now_s);
        if (terminal()) return report_.ok;
        if (!snapshot_reader_ || !motion_.ready()) return fail("controller_port_or_sensor_not_ready", now_s);
        const auto snapshot = snapshot_reader_(now_s);

        if (state_ == StopGoMappingControllerState::WaitingForCoreReady) {
            report_.core_wait_ticks++;
            // A missing/invalid startup sample is a reason to keep waiting for
            // Core's stationary calibration, not a stop-go motion fault.
            if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
                return true;
            }

            const auto odom_only = make_odom_only(snapshot);
            const auto app_step = application_.step(odom_only, now_s);
            if (application_.core().localization_ready()) {
                report_.core_ready_observed = true;
                transition(StopGoMappingControllerState::InitialSettle);
                return true;
            }

            const auto &core_report = application_.core().report();
            if (core_report.initialization_status == "fault" ||
                core_report.initialization_status == "gyro_calibration_failed" ||
                core_report.initialization_status == "wheel_baseline_failed" ||
                core_report.last_fault != "none") {
                return fail("core_initialization_failed:" +
                                (app_step.reason.empty()
                                     ? core_report.last_message
                                     : app_step.reason),
                            now_s);
            }
            // During normal startup Core returns ok=false while collecting
            // stationary samples.  That is an expected waiting state.
            return true;
        }

        if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
            return fail("canonical_wheel_or_imu_feedback_invalid", now_s);
        }
        const auto odom_only = make_odom_only(snapshot);
        // A stable ToF commit is itself the canonical sample for this stop.
        // Do not first consume the same wheel timestamp as an odometry-only
        // sample; the core deliberately rejects non-monotonic feedback.
        const auto revision_before_odom_step =
            application_.core().report().current_map_revision;
        if (state_ != StopGoMappingControllerState::InitialSample &&
            state_ != StopGoMappingControllerState::AcquireThreeTof) {
            const auto app_step = application_.step(odom_only, now_s);
            if (!app_step.ok || !app_step.core_step_executed) {
                return fail("application_core_step_failed:" + app_step.reason, now_s);
            }
            const auto revision_after_odom_step =
                application_.core().report().current_map_revision;
            if ((state_ == StopGoMappingControllerState::WaitMotionDone ||
                 state_ == StopGoMappingControllerState::WaitMappingSettle) &&
                revision_after_odom_step != revision_before_odom_step) {
                report_.map_writes_while_moving++;
            }
        }

        if (state_ == StopGoMappingControllerState::InitialSettle) {
            RelativeMotionStepResult initial;
            initial.state = RelativeMotionStepState::Done;
            const auto settled = gate_.update(initial, odom_only,
                                              monotonic_us(now_s), true);
            if (settled.stable) transition(StopGoMappingControllerState::InitialSample);
            return true;
        }
        if (state_ == StopGoMappingControllerState::InitialSample) {
            transition(StopGoMappingControllerState::AcquireThreeTof);
            return acquire_and_commit(snapshot, now_s, 0);
        }
        if (state_ == StopGoMappingControllerState::IssueForwardStep) {
            if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
                total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
                return complete("max_steps_or_distance_reached", now_s);
            }
            command_ = {};
            command_.command_id = next_command_id_++;
            command_.action = RelativeMotionStepAction::Forward;
            command_.target_amount = config_.forward_step_mm;
            command_.max_rpm = config_.motion_max_rpm;
            command_.timeout_ms = config_.motion_timeout_ms;
            const auto accepted = motion_.submit(command_, now_s);
            if (accepted.state != RelativeMotionStepState::Accepted) {
                return fail("motion_submit_failed:" + accepted.reason, now_s);
            }
            report_.commands_submitted++;
            report_.command_ids.push_back(command_.command_id);
            transition(StopGoMappingControllerState::WaitMotionDone);
            return true;
        }
        if (state_ == StopGoMappingControllerState::WaitMotionDone) {
            const auto result = motion_.poll(now_s);
            last_motion_ = result;
            if (result.state == RelativeMotionStepState::Fault ||
                result.state == RelativeMotionStepState::Timeout ||
                result.state == RelativeMotionStepState::Cancelled ||
                result.state == RelativeMotionStepState::EstopLatched) {
                return fail("motion_failed:" + result.reason, now_s);
            }
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitMappingSettle);
            }
            return true;
        }
        if (state_ == StopGoMappingControllerState::WaitMappingSettle) {
            last_motion_ = motion_.poll(now_s);
            const auto settled = gate_.update(last_motion_, odom_only,
                                              monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::AcquireThreeTof);
            return true;
        }
        if (state_ == StopGoMappingControllerState::AcquireThreeTof) {
            return acquire_and_commit(snapshot, now_s, report_.completed_steps + 1);
        }
        return true;
    }

    bool request_stop(double now_s) {
        const auto result = motion_.stop(now_s);
        if (state_ == StopGoMappingControllerState::WaitMotionDone ||
            state_ == StopGoMappingControllerState::WaitMappingSettle ||
            state_ == StopGoMappingControllerState::WaitTurnDone ||
            state_ == StopGoMappingControllerState::WaitTurnSettle ||
            state_ == StopGoMappingControllerState::VerifyAfterTurn) {
            last_motion_ = result;
            transition(StopGoMappingControllerState::Fault);
            report_.termination_reason = "cancelled_by_stop";
        }
        return result.state == RelativeMotionStepState::Done ||
               result.state == RelativeMotionStepState::Cancelled;
    }

    bool request_estop(double now_s) {
        last_motion_ = motion_.emergency_stop(now_s);
        transition(StopGoMappingControllerState::Estop);
        report_.termination_reason = "estop_latched";
        return true;
    }

    bool terminal() const {
        return state_ == StopGoMappingControllerState::Completed ||
               state_ == StopGoMappingControllerState::WallLost ||
               state_ == StopGoMappingControllerState::FrontBlocked ||
               state_ == StopGoMappingControllerState::Fault ||
               state_ == StopGoMappingControllerState::Estop;
    }
    StopGoMappingControllerState state() const { return state_; }
    const StopGoMappingRunReport &report() const { return report_; }

private:
    bool left_wall_mode() const { return config_.mode == "left_wall_follow"; }

    bool tick_left_wall(double now_s) {
        if (terminal()) return report_.ok;
        if (!snapshot_reader_ || !motion_.ready()) {
            return fail("controller_port_or_sensor_not_ready", now_s);
        }
        const auto snapshot = snapshot_reader_(now_s);
        const auto &core = application_.core();

        if (state_ == StopGoMappingControllerState::WaitingForCoreReady) {
            report_.core_wait_ticks++;
            if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
                return true;
            }
            const auto app_step = application_.step(
                make_odom_only(snapshot), now_s);
            if (core.localization_ready()) {
                report_.core_ready_observed = true;
                gate_.reset();
                transition(StopGoMappingControllerState::InitialSettle);
                return true;
            }
            const auto &core_report = core.report();
            const bool recovery_wait = core.relocalization_required() ||
                core_report.relocalization_active ||
                core_report.localization_stop_required;
            if (!recovery_wait &&
                (core_report.initialization_status == "fault" ||
                 core_report.initialization_status == "gyro_calibration_failed" ||
                 core_report.initialization_status == "wheel_baseline_failed" ||
                 core_report.last_fault != "none")) {
                return fail("core_initialization_failed:" +
                                (app_step.reason.empty()
                                     ? core_report.last_message
                                     : app_step.reason),
                            now_s);
            }
            return true;
        }

        if (!core.localization_ready() ||
            core.localization_health_state() == LocalizationHealthState::Lost ||
            core.report().localization_stop_required ||
            core.relocalization_required()) {
            if (state_ != StopGoMappingControllerState::WaitingForCoreReady) {
                (void)motion_.stop(now_s);
                clear_wall_window("core_not_ready_or_recovering", true);
                transition(StopGoMappingControllerState::WaitingForCoreReady);
            }
            if (snapshot.has_wheel && snapshot.has_imu && snapshot.wheel.valid) {
                (void)application_.step(make_odom_only(snapshot), now_s);
            }
            return true;
        }

        if (wall_epoch_ != 0 && core.frame_transform_epoch() != wall_epoch_) {
            (void)motion_.stop(now_s);
            clear_wall_window("frame_transform_epoch_changed", true);
            transition(StopGoMappingControllerState::WallAcquisition);
            return true;
        }
        if (!snapshot.has_wheel || !snapshot.has_imu || !snapshot.wheel.valid) {
            return fail("canonical_wheel_or_imu_feedback_invalid", now_s);
        }
        const auto odom_only = make_odom_only(snapshot);
        const bool commit_state =
            state_ == StopGoMappingControllerState::InitialSample ||
            state_ == StopGoMappingControllerState::AcquireThreeTof ||
            state_ == StopGoMappingControllerState::VerifyAfterTurn;
        if (!commit_state) {
            const auto before = core.report().current_map_revision;
            const auto app_step = application_.step(odom_only, now_s);
            if (!app_step.ok || !app_step.core_step_executed) {
                return fail("application_core_step_failed:" + app_step.reason, now_s);
            }
            const auto after = core.report().current_map_revision;
            if (state_ == StopGoMappingControllerState::WaitMotionDone ||
                state_ == StopGoMappingControllerState::WaitMappingSettle) {
                if (after != before) report_.map_writes_while_moving++;
            }
            if (state_ == StopGoMappingControllerState::WaitTurnDone ||
                state_ == StopGoMappingControllerState::WaitTurnSettle) {
                if (after != before) report_.map_writes_during_turn_or_verify++;
            }
            pending_replay_odom_samples_.push_back({odom_only.wheel, odom_only.imu});
        }

        switch (state_) {
        case StopGoMappingControllerState::InitialSettle: {
            RelativeMotionStepResult initial;
            initial.state = RelativeMotionStepState::Done;
            const auto settled = gate_.update(initial, odom_only,
                                              monotonic_us(now_s), true);
            if (settled.stable) {
                transition(StopGoMappingControllerState::InitialSample);
            }
            return true;
        }
        case StopGoMappingControllerState::InitialSample:
            transition(StopGoMappingControllerState::WallAcquisition);
            return acquire_left_wall_sample(snapshot, now_s, 0);
        case StopGoMappingControllerState::WallAcquisition:
            return acquire_left_wall_sample(snapshot, now_s,
                                            report_.completed_steps + 1);
        case StopGoMappingControllerState::IssueForwardStep:
            return issue_forward_step(now_s);
        case StopGoMappingControllerState::WaitMotionDone: {
            const auto result = motion_.poll(now_s);
            if (!motion_result_matches_command(result)) {
                return fail("forward_command_id_mismatch", now_s);
            }
            last_motion_ = result;
            if (motion_failed(result)) return fail("motion_failed:" + result.reason, now_s);
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitMappingSettle);
            }
            return true;
        }
        case StopGoMappingControllerState::WaitMappingSettle: {
            last_motion_ = motion_.poll(now_s);
            if (!motion_result_matches_command(last_motion_)) {
                return fail("forward_settle_command_id_mismatch", now_s);
            }
            const auto settled = gate_.update(last_motion_, odom_only,
                                              monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::AcquireThreeTof);
            return true;
        }
        case StopGoMappingControllerState::AcquireThreeTof:
            return acquire_left_wall_sample(snapshot, now_s,
                                             report_.completed_steps + 1);
        case StopGoMappingControllerState::IssueTurnCorrection:
            return issue_turn_correction(now_s);
        case StopGoMappingControllerState::WaitTurnDone: {
            const auto result = motion_.poll(now_s);
            if (!motion_result_matches_command(result)) {
                return fail("turn_command_id_mismatch", now_s);
            }
            last_motion_ = result;
            if (motion_failed(result)) return fail("turn_motion_failed:" + result.reason, now_s);
            if (result.state == RelativeMotionStepState::Done) {
                gate_.reset();
                transition(StopGoMappingControllerState::WaitTurnSettle);
            }
            return true;
        }
        case StopGoMappingControllerState::WaitTurnSettle: {
            last_motion_ = motion_.poll(now_s);
            if (!motion_result_matches_command(last_motion_)) {
                return fail("turn_settle_command_id_mismatch", now_s);
            }
            const auto settled = gate_.update(last_motion_, odom_only,
                                              monotonic_us(now_s));
            if (settled.stable) transition(StopGoMappingControllerState::VerifyAfterTurn);
            return true;
        }
        case StopGoMappingControllerState::VerifyAfterTurn:
            return verify_after_turn(snapshot, now_s);
        case StopGoMappingControllerState::CheckLimit:
            return check_left_wall_limit(now_s);
        case StopGoMappingControllerState::Completed:
        case StopGoMappingControllerState::WallLost:
        case StopGoMappingControllerState::FrontBlocked:
        case StopGoMappingControllerState::Fault:
        case StopGoMappingControllerState::Estop:
        case StopGoMappingControllerState::Idle:
        case StopGoMappingControllerState::WaitingForCoreReady:
        case StopGoMappingControllerState::CommitMappingSample:
            return true;
        }
        return true;
    }

    static bool motion_failed(const RelativeMotionStepResult &result) {
        return result.state == RelativeMotionStepState::Fault ||
               result.state == RelativeMotionStepState::Timeout ||
               result.state == RelativeMotionStepState::Cancelled ||
               result.state == RelativeMotionStepState::EstopLatched;
    }

    bool motion_result_matches_command(const RelativeMotionStepResult &result) const {
        return result.command_id == command_.command_id &&
               result.action == command_.action;
    }

    bool issue_forward_step(double now_s) {
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return complete("max_steps_or_distance_reached", now_s);
        }
        command_ = {};
        command_.command_id = next_command_id_++;
        command_.action = RelativeMotionStepAction::Forward;
        command_.target_amount = config_.forward_step_mm;
        command_.max_rpm = config_.motion_max_rpm;
        command_.timeout_ms = config_.motion_timeout_ms;
        const auto accepted = motion_.submit(command_, now_s);
        if (accepted.state != RelativeMotionStepState::Accepted) {
            return fail("motion_submit_failed:" + accepted.reason, now_s);
        }
        report_.commands_submitted++;
        report_.forward_command_count++;
        report_.command_ids.push_back(command_.command_id);
        if (!wall_model_.valid) report_.wall_acquisition_steps++;
        transition(StopGoMappingControllerState::WaitMotionDone);
        return true;
    }

    bool issue_turn_correction(double now_s) {
        if (pending_decision_.action == LeftWallControlAction::NoTurn) {
            transition(StopGoMappingControllerState::IssueForwardStep);
            return true;
        }
        command_ = {};
        command_.command_id = next_command_id_++;
        command_.action = pending_decision_.action == LeftWallControlAction::Left
            ? RelativeMotionStepAction::Left : RelativeMotionStepAction::Right;
        command_.target_amount = pending_decision_.correction_amount_deg;
        command_.max_rpm = config_.motion_max_rpm;
        command_.timeout_ms = config_.motion_timeout_ms;
        const auto accepted = motion_.submit(command_, now_s);
        if (accepted.state != RelativeMotionStepState::Accepted) {
            return fail("turn_submit_failed:" + accepted.reason, now_s);
        }
        report_.commands_submitted++;
        report_.correction_command_count++;
        if (command_.action == RelativeMotionStepAction::Left) {
            report_.left_correction_count++;
        } else {
            report_.right_correction_count++;
        }
        report_.command_ids.push_back(command_.command_id);
        transition(StopGoMappingControllerState::WaitTurnDone);
        return true;
    }

    bool acquire_left_wall_sample(const RobotSlamSensorSnapshot &base,
                                  double now_s, std::size_t cycle) {
        transition(StopGoMappingControllerState::AcquireThreeTof);
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.front.usable_for_mapping) {
            ++front_invalid_samples_;
            if (front_invalid_samples_ <= config_.front_max_invalid_samples) {
                return true;
            }
            return front_blocked("front_sample_invalid", now_s);
        }
        front_invalid_samples_ = 0;
        if (stable.front.distance_mm <=
            config_.front_stop_threshold_m * 1000.0 + 1e-9) {
            return front_blocked("front_obstacle_below_stop_threshold", now_s);
        }
        if (!stable.left.usable_for_mapping) {
            if (wall_points_.empty()) {
                ++stationary_resample_attempts_;
                if (stationary_resample_attempts_ <=
                    config_.max_stationary_resample_attempts) {
                    return true;
                }
                return wall_lost("initial_left_wall_resample_exhausted", now_s);
            }
            ++consecutive_wall_misses_;
            if (consecutive_wall_misses_ <= config_.max_consecutive_wall_misses) {
                return true;
            }
            return wall_lost("left_wall_measurement_lost", now_s);
        }
        stationary_resample_attempts_ = 0;
        consecutive_wall_misses_ = 0;
        if (!stable.right.usable_for_mapping) {
            return fail("right_measurement_unusable_for_core_map_commit", now_s);
        }

        transition(StopGoMappingControllerState::CommitMappingSample);
        const auto full = make_stable_snapshot(base, stable);
        const std::uint64_t before = application_.core().report().current_map_revision;
        const auto committed = application_.step(full, now_s);
        const std::uint64_t after = application_.core().report().current_map_revision;
        const auto &accepted = application_.core().last_accepted_map_observation();
        if (!committed.ok || after <= before || !accepted.valid ||
            !accepted.backend_accepted ||
            accepted.map_revision_before != before ||
            accepted.map_revision_after != after ||
            !accepted.measurement_poses.left.valid ||
            accepted.frame_transform_epoch != application_.core().frame_transform_epoch()) {
            return fail("stable_snapshot_map_commit_failed:" + committed.reason, now_s);
        }
        report_.map_commits++;
        report_.completed_steps += cycle == 0 ? 0 : 1;
        if (cycle != 0) {
            total_distance_mm_ += last_motion_.actual_distance_mm;
            report_.commands_completed++;
        }
        const auto point = project_left_wall_point(
            stable.left, accepted.measurement_poses.left.base_pose_in_map,
            accepted.map_revision_after, accepted.frame_transform_epoch, cycle);
        if (!point) return fail("left_wall_point_projection_failed", now_s);
        if (wall_epoch_ == 0) wall_epoch_ = accepted.frame_transform_epoch;
        if (point->frame_transform_epoch != wall_epoch_) {
            clear_wall_window("frame_transform_epoch_changed_at_point", true);
            transition(StopGoMappingControllerState::WallAcquisition);
            return true;
        }
        wall_points_.push_back(*point);
        while (wall_points_.size() > config_.wall_estimator.window_max_points) {
            wall_points_.erase(wall_points_.begin());
        }
        hash_wall_point(*point);
        wall_model_ = wall_estimator_.fit(
            wall_points_, application_.core().current_map_pose().map_T_base,
            wall_epoch_);
        report_.frame_transform_epoch = wall_epoch_;
        report_.wall_fit_input_points = wall_model_.input_point_count;
        report_.wall_fit_inliers = wall_model_.inlier_point_count;
        report_.wall_fit_baseline_m = wall_model_.baseline_m;
        report_.wall_fit_rms_m = wall_model_.rms_residual_m;
        hash_wall_model(wall_model_);
        if (wall_model_.valid) {
            report_.wall_model_valid_count++;
            if (!desired_distance_captured_ &&
                config_.desired_distance_mode == "capture_initial") {
                desired_distance_m_ = wall_model_.signed_base_to_wall_distance_m;
                desired_distance_captured_ = true;
            }
            pending_decision_ = decide_left_wall_control(
                wall_model_, application_.core().current_map_pose().map_T_base,
                desired_distance_m_, config_);
            update_control_metrics(pending_decision_);
            if (wall_model_.signed_base_to_wall_distance_m <
                    config_.min_allowed_base_to_wall_distance_m ||
                wall_model_.signed_base_to_wall_distance_m >
                    config_.max_allowed_base_to_wall_distance_m) {
                return wall_lost("base_to_wall_distance_out_of_safe_range", now_s);
            }
        } else {
            pending_decision_ = {};
            if (report_.wall_acquisition_steps >=
                config_.wall_acquisition_max_forward_steps) {
                return wall_lost("wall_acquisition_limit_reached", now_s);
            }
        }

        StopGoReplayRecord replay;
        replay.cycle_index = cycle;
        replay.command_id = cycle == 0 ? 0 : command_.command_id;
        replay.motion = cycle == 0 ? RelativeMotionStepResult{} : last_motion_;
        replay.snapshot = full;
        replay.stable_sample = stable;
        replay.map_revision_before = before;
        replay.map_revision_after = after;
        replay.motor_settled_timestamp_us = cycle == 0
            ? 0 : monotonic_us(last_motion_.final_feedback.wheel.timestamp_s);
        replay.mapping_ready_timestamp_us = gate_.last_result().stable_timestamp_us;
        replay.settle_frame_count = gate_.last_result().consecutive_frames;
        replay.settle_duration_ms = gate_.last_result().stable_duration_ms;
        replay.odom_pose = application_.core().report().last_odom_pose;
        replay.map_from_odom = application_.core().frame_state().current_map_from_odom().map_T_odom;
        replay.controller_state = StopGoMappingControllerState::CommitMappingSample;
        replay.left_wall_point_accepted = true;
        replay.left_wall_point = *point;
        replay.wall_model = wall_model_;
        replay.control_decision = pending_decision_;
        replay.post_turn_verified = post_turn_verified_since_last_commit_;
        replay.frame_transform_epoch = wall_epoch_;
        replay.odom_samples = std::move(pending_replay_odom_samples_);
        pending_replay_odom_samples_.clear();
        post_turn_verified_since_last_commit_ = false;
        report_.replay_records.push_back(replay);
        write_log(replay, now_s);
        transition(StopGoMappingControllerState::CheckLimit);
        return check_left_wall_limit(now_s);
    }

    std::optional<LeftWallPoint> project_left_wall_point(
        const StableTofReading &reading,
        const MapPose2D &measurement_base_pose,
        std::uint64_t map_revision,
        std::uint64_t epoch,
        std::size_t cycle) const {
        if (!reading.usable_for_mapping || !sparse_slam_frame_pose_valid(measurement_base_pose) ||
            !std::isfinite(reading.sample_timestamp_us) ||
            !std::isfinite(reading.distance_mm)) {
            return std::nullopt;
        }
        const auto map_T_left_tof = compose_robot_poses(
            measurement_base_pose.map_T_base, config_.base_T_left_tof);
        const double distance_m = reading.distance_mm / 1000.0;
        LeftWallPoint point;
        point.x_m = map_T_left_tof.x_m + std::cos(map_T_left_tof.yaw_rad) * distance_m;
        point.y_m = map_T_left_tof.y_m + std::sin(map_T_left_tof.yaw_rad) * distance_m;
        point.timestamp_s = reading.sample_timestamp_us / 1e6;
        point.confidence = reading.confidence / 100.0;
        point.cycle_index = cycle;
        point.map_revision = map_revision;
        point.frame_transform_epoch = epoch;
        if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
            !std::isfinite(point.timestamp_s) || !std::isfinite(point.confidence)) {
            return std::nullopt;
        }
        return point;
    }

    static void hash_mix(std::uint64_t &hash, std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ULL;
    }

    static std::uint64_t quantized(double value) {
        if (!std::isfinite(value)) return 0;
        return static_cast<std::uint64_t>(std::llround(value * 1000000.0));
    }

    void hash_wall_point(const LeftWallPoint &point) {
        hash_mix(report_.wall_point_sequence_hash, quantized(point.x_m));
        hash_mix(report_.wall_point_sequence_hash, quantized(point.y_m));
        hash_mix(report_.wall_point_sequence_hash, point.cycle_index);
        hash_mix(report_.wall_point_sequence_hash, point.frame_transform_epoch);
    }

    void hash_wall_model(const LeftWallModel &model) {
        hash_mix(report_.wall_model_sequence_hash, model.valid ? 1U : 0U);
        hash_mix(report_.wall_model_sequence_hash, quantized(model.wall_heading_rad));
        hash_mix(report_.wall_model_sequence_hash,
                 quantized(model.signed_base_to_wall_distance_m));
        hash_mix(report_.wall_model_sequence_hash, quantized(model.baseline_m));
        hash_mix(report_.wall_model_sequence_hash, quantized(model.rms_residual_m));
        hash_mix(report_.wall_model_sequence_hash, model.inlier_point_count);
        hash_mix(report_.wall_model_sequence_hash, model.frame_transform_epoch);
    }

    void update_control_metrics(const LeftWallControlDecision &decision) {
        const double heading_deg = decision.heading_error_rad *
            180.0 / 3.14159265358979323846;
        const double distance_mm = decision.distance_error_m * 1000.0;
        if (!have_control_metrics_) {
            report_.initial_heading_error_deg = heading_deg;
            report_.initial_distance_error_mm = distance_mm;
            have_control_metrics_ = true;
        }
        report_.final_heading_error_deg = heading_deg;
        report_.final_distance_error_mm = distance_mm;
        report_.maximum_abs_heading_error_deg = std::max(
            report_.maximum_abs_heading_error_deg, std::fabs(heading_deg));
        report_.maximum_abs_distance_error_mm = std::max(
            report_.maximum_abs_distance_error_mm, std::fabs(distance_mm));
    }

    void clear_wall_window(const char *reason, bool frame_change) {
        if (!wall_points_.empty() || wall_model_.valid) {
            report_.wall_model_reset_count++;
            if (frame_change) report_.wall_model_reset_due_to_frame_change_count++;
        }
        wall_points_.clear();
        wall_model_ = {};
        pending_decision_ = {};
        consecutive_wall_misses_ = 0;
        stationary_resample_attempts_ = 0;
        front_invalid_samples_ = 0;
        desired_distance_captured_ = false;
        desired_distance_m_ = config_.desired_base_to_wall_distance_m;
        wall_epoch_ = application_.core().frame_transform_epoch();
        report_.frame_transform_epoch = wall_epoch_;
        (void)reason;
    }

    bool wall_lost(const char *reason, double now_s) {
        (void)motion_.stop(now_s);
        clear_wall_window(reason, false);
        transition(StopGoMappingControllerState::WallLost);
        report_.ok = false;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        return false;
    }

    bool front_blocked(const char *reason, double now_s) {
        (void)motion_.stop(now_s);
        transition(StopGoMappingControllerState::FrontBlocked);
        report_.ok = false;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        return false;
    }

    bool verify_after_turn(const RobotSlamSensorSnapshot &base, double now_s) {
        report_.post_turn_verification_count++;
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.front.usable_for_mapping) {
            report_.post_turn_verification_failure_count++;
            return front_blocked("post_turn_front_sample_invalid", now_s);
        }
        if (stable.front.distance_mm <=
            config_.front_stop_threshold_m * 1000.0 + 1e-9) {
            report_.post_turn_verification_failure_count++;
            return front_blocked("post_turn_front_blocked", now_s);
        }
        if (!stable.left.usable_for_mapping) {
            report_.post_turn_verification_failure_count++;
            return wall_lost("post_turn_left_wall_lost", now_s);
        }
        if (!stable.right.usable_for_mapping) {
            report_.post_turn_verification_failure_count++;
            return fail("post_turn_right_sample_invalid", now_s);
        }
        const auto full = make_stable_snapshot(base, stable);
        const auto before = application_.core().report().current_map_revision;
        const auto pose_success_before =
            application_.core().report().measurement_pose_set_success_count;
        const auto verified = application_.step(
            full, now_s, {}, ActiveObservationControl::None, false);
        const auto after = application_.core().report().current_map_revision;
        if (!verified.ok || after != before ||
            application_.core().report().measurement_pose_set_success_count <=
                pose_success_before ||
            !application_.core().localization_ready()) {
            report_.post_turn_verification_failure_count++;
            if (after != before) report_.map_writes_during_turn_or_verify++;
            return fail("post_turn_verification_core_rejected", now_s);
        }
        pending_replay_odom_samples_.push_back({full.wheel, full.imu});
        post_turn_verified_since_last_commit_ = true;
        transition(StopGoMappingControllerState::IssueForwardStep);
        return true;
    }

    bool check_left_wall_limit(double now_s) {
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return complete("max_steps_or_distance_reached", now_s);
        }
        if (wall_model_.valid && pending_decision_.action != LeftWallControlAction::NoTurn) {
            transition(StopGoMappingControllerState::IssueTurnCorrection);
        } else {
            transition(StopGoMappingControllerState::IssueForwardStep);
        }
        return true;
    }

    static std::uint64_t monotonic_us(double now_s) {
        return static_cast<std::uint64_t>(std::llround(now_s * 1e6));
    }

    static RobotSlamSensorSnapshot make_odom_only(RobotSlamSensorSnapshot snapshot) {
        snapshot.has_tof = false;
        snapshot.has_multi_tof = false;
        snapshot.tof = {};
        snapshot.multi_tof = {};
        return snapshot;
    }

    RobotSlamSensorSnapshot make_stable_snapshot(
        const RobotSlamSensorSnapshot &base,
        const StableThreeTofSample &stable) const {
        RobotSlamSensorSnapshot snapshot = base;
        snapshot.has_tof = false;
        snapshot.has_multi_tof = true;
        snapshot.multi_tof = {};
        snapshot.multi_tof.has_front = stable.front.usable_for_mapping;
        snapshot.multi_tof.has_left = stable.left.usable_for_mapping;
        snapshot.multi_tof.has_right = stable.right.usable_for_mapping;
        snapshot.multi_tof.valid_tof_count = 0;
        auto fill = [](const StableTofReading &reading,
                       ScalarTofSnapshotFrame &frame,
                       const char *id, double yaw) {
            frame.distance_mm = reading.distance_mm;
            frame.distance_m = reading.distance_mm / 1000.0;
            frame.confidence = reading.confidence;
            frame.effective_timestamp_s = reading.sample_timestamp_us / 1e6;
            frame.protocol_valid = reading.valid;
            frame.usable_for_mapping = reading.usable_for_mapping;
            frame.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
            frame.mount_yaw_rad = yaw;
            frame.frame_id = id;
            frame.source = "stop_go_stable_three_tof";
            frame.reason = reading.rejection_reason;
        };
        fill(stable.front, snapshot.multi_tof.front, "tof_front_frame", 0.0);
        fill(stable.left, snapshot.multi_tof.left, "tof_left_frame", 1.5707963267948966);
        fill(stable.right, snapshot.multi_tof.right, "tof_right_frame", -1.5707963267948966);
        snapshot.multi_tof.valid_tof_count =
            static_cast<int>(snapshot.multi_tof.has_front) +
            static_cast<int>(snapshot.multi_tof.has_left) +
            static_cast<int>(snapshot.multi_tof.has_right);
        snapshot.multi_tof.synchronized_time_s =
            std::max({stable.front.sample_timestamp_us,
                      stable.left.sample_timestamp_us,
                      stable.right.sample_timestamp_us}) / 1e6;
        snapshot.multi_tof.source = "stop_go_stable_three_tof";
        snapshot.multi_tof.multi_tof_imu_dt_s =
            snapshot.multi_tof.synchronized_time_s - snapshot.imu.timestamp_s;
        snapshot.multi_tof.multi_tof_wheel_dt_s =
            snapshot.multi_tof.synchronized_time_s - snapshot.wheel.timestamp_s;
        return snapshot;
    }

    bool acquire_and_commit(const RobotSlamSensorSnapshot &base,
                            double now_s, std::size_t cycle) {
        transition(StopGoMappingControllerState::AcquireThreeTof);
        const auto stable = sampler_.acquire();
        report_.stable_samples++;
        if (!stable.usable_for_mapping) {
            return fail("stable_three_tof_unusable", now_s);
        }
        transition(StopGoMappingControllerState::CommitMappingSample);
        const auto full = make_stable_snapshot(base, stable);
        const std::uint64_t before = application_.core().report().current_map_revision;
        const auto committed = application_.step(full, now_s);
        const std::uint64_t after = application_.core().report().current_map_revision;
        if (!committed.ok || after <= before) {
            return fail("stable_snapshot_map_commit_failed:" + committed.reason, now_s);
        }
        report_.map_commits++;
        report_.completed_steps += cycle == 0 ? 0 : 1;
        if (cycle != 0) {
            total_distance_mm_ += last_motion_.actual_distance_mm;
            report_.commands_completed++;
        }
        StopGoReplayRecord replay;
        replay.cycle_index = cycle;
        replay.command_id = cycle == 0 ? 0 : command_.command_id;
        replay.motion = cycle == 0 ? RelativeMotionStepResult{} : last_motion_;
        replay.snapshot = full;
        replay.stable_sample = stable;
        replay.map_revision_before = before;
        replay.map_revision_after = after;
        replay.motor_settled_timestamp_us = cycle == 0
            ? 0 : monotonic_us(last_motion_.final_feedback.wheel.timestamp_s);
        replay.mapping_ready_timestamp_us = gate_.last_result().stable_timestamp_us;
        replay.settle_frame_count = gate_.last_result().consecutive_frames;
        replay.settle_duration_ms = gate_.last_result().stable_duration_ms;
        replay.odom_pose = application_.core().report().last_odom_pose;
        replay.map_from_odom =
            application_.core().frame_state().current_map_from_odom().map_T_odom;
        replay.controller_state = StopGoMappingControllerState::CommitMappingSample;
        report_.replay_records.push_back(replay);
        write_log(replay, now_s);
        transition(StopGoMappingControllerState::CheckLimit);
        return check_limit(now_s);
    }

    bool check_limit(double now_s) {
        if (report_.completed_steps >= static_cast<std::size_t>(config_.max_steps) ||
            total_distance_mm_ >= config_.max_total_distance_mm - 1e-9) {
            return complete("max_steps_or_distance_reached", now_s);
        }
        transition(StopGoMappingControllerState::IssueForwardStep);
        return true;
    }

    bool complete(const char *reason, double) {
        motion_.stop(0.0);
        transition(StopGoMappingControllerState::Completed);
        report_.ok = true;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        report_.frame_transform_epoch = application_.core().frame_transform_epoch();
        return true;
    }

    bool fail(const std::string &reason, double now_s) {
        // Stop precedes any error/logging result.  The port implementation is
        // idempotent, so this is safe for a failed submit and a prior stop.
        motion_.stop(now_s);
        transition(StopGoMappingControllerState::Fault);
        report_.ok = false;
        report_.termination_reason = reason;
        report_.map_revision = application_.core().report().current_map_revision;
        report_.map_cells = application_.core().report().sparse_map_cell_count;
        report_.frame_transform_epoch = application_.core().frame_transform_epoch();
        return false;
    }

    void transition(StopGoMappingControllerState next) {
        state_ = next;
        report_.state_trace.push_back(to_string(next));
    }

    void write_log(const StopGoReplayRecord &record, double now_s) {
        if (!log_.is_open()) return;
        const auto &s = record.snapshot;
        const auto write_raw_samples = [this](const char *name,
                                               const StableTofReading &reading) {
            log_ << ",\"" << name << "\":[";
            for (std::size_t index = 0; index < reading.raw_frame_count; ++index) {
                if (index != 0) log_ << ",";
                const auto &raw = reading.raw_frames[index];
                log_ << "{\"valid\":" << (raw.frame.valid ? 1 : 0)
                     << ",\"distance_mm\":" << raw.frame.distance_mm
                     << ",\"confidence\":" << static_cast<int>(raw.frame.confidence)
                     << ",\"request_start_us\":" << raw.request_start_us
                     << ",\"response_received_us\":" << raw.response_received_us
                     << ",\"sample_timestamp_us\":" << raw.sample_timestamp_us << "}";
            }
            log_ << "]";
        };
        log_ << "{\"cycle_index\":" << record.cycle_index
             << ",\"command_id\":" << record.command_id
             << ",\"action\":\"" << to_string(record.motion.action)
             << "\",\"requested_amount\":" << record.motion.requested_amount
             << ",\"motion_state\":\"" << to_string(record.motion.state)
             << "\",\"actual_distance_mm\":" << record.motion.actual_distance_mm
             << ",\"actual_angle_deg\":" << record.motion.actual_angle_deg
             << ",\"error_code\":" << record.motion.error_code
             << ",\"reason\":\"" << record.motion.reason << "\""
             << ",\"now_s\":" << now_s
             << ",\"left_ticks\":" << s.wheel.left_ticks
             << ",\"right_ticks\":" << s.wheel.right_ticks
             << ",\"left_rpm\":" << s.wheel.left_rpm
             << ",\"right_rpm\":" << s.wheel.right_rpm
             << ",\"pair_skew_us\":" << s.wheel.pair_skew_s * 1e6
             << ",\"gyro_z\":" << s.imu.yaw_rate_rad_s
             << ",\"motor_settled_timestamp_us\":" << record.motor_settled_timestamp_us
             << ",\"mapping_ready_timestamp_us\":" << record.mapping_ready_timestamp_us
             << ",\"settle_frame_count\":" << record.settle_frame_count
             << ",\"settle_duration_ms\":" << record.settle_duration_ms
             << ",\"map_revision_before\":" << record.map_revision_before
             << ",\"map_revision_after\":" << record.map_revision_after
             << ",\"front_mm\":" << record.stable_sample.front.distance_mm
             << ",\"left_mm\":" << record.stable_sample.left.distance_mm
             << ",\"right_mm\":" << record.stable_sample.right.distance_mm
             << ",\"front_selected_us\":" << record.stable_sample.front.sample_timestamp_us
             << ",\"left_selected_us\":" << record.stable_sample.left.sample_timestamp_us
             << ",\"right_selected_us\":" << record.stable_sample.right.sample_timestamp_us;
        write_raw_samples("front_raw_samples", record.stable_sample.front);
        write_raw_samples("left_raw_samples", record.stable_sample.left);
        write_raw_samples("right_raw_samples", record.stable_sample.right);
        log_ << ",\"frame_transform_epoch\":" << record.frame_transform_epoch
             << ",\"left_wall_point_accepted\":"
             << (record.left_wall_point_accepted ? 1 : 0)
             << ",\"wall_point_x_m\":" << record.left_wall_point.x_m
             << ",\"wall_point_y_m\":" << record.left_wall_point.y_m
             << ",\"wall_model_valid\":" << (record.wall_model.valid ? 1 : 0)
             << ",\"wall_heading_rad\":" << record.wall_model.wall_heading_rad
             << ",\"signed_wall_distance_m\":"
             << record.wall_model.signed_base_to_wall_distance_m
             << ",\"wall_fit_baseline_m\":" << record.wall_model.baseline_m
             << ",\"wall_fit_rms_m\":" << record.wall_model.rms_residual_m
             << ",\"wall_fit_input_points\":" << record.wall_model.input_point_count
             << ",\"wall_fit_inliers\":" << record.wall_model.inlier_point_count
             << ",\"heading_error_rad\":" << record.control_decision.heading_error_rad
             << ",\"distance_error_m\":" << record.control_decision.distance_error_m
             << ",\"distance_bias_rad\":" << record.control_decision.distance_bias_rad
             << ",\"turn_error_rad\":" << record.control_decision.turn_error_rad
             << ",\"control_action\":\""
             << to_string(record.control_decision.action) << "\""
             << ",\"correction_amount_deg\":"
             << record.control_decision.correction_amount_deg
             << ",\"post_turn_verified\":"
             << (record.post_turn_verified ? 1 : 0);
        log_ << ",\"controller_state\":\"" << to_string(record.controller_state)
             << "\"}\n";
        if (!log_) report_.log_error = "stop_go_log_write_failed";
    }

    RobotSlamApplication &application_;
    RelativeMotionStepPort &motion_;
    SnapshotReader snapshot_reader_;
    StableThreeTofSampler sampler_;
    StopGoMappingControllerConfig config_;
    MappingSettleGate gate_;
    LeftWallLineEstimator wall_estimator_;
    StopGoMappingControllerState state_ = StopGoMappingControllerState::Idle;
    StopGoMappingRunReport report_;
    RelativeMotionStepCommand command_;
    RelativeMotionStepResult last_motion_;
    std::uint64_t next_command_id_ = 1;
    double total_distance_mm_ = 0.0;
    std::vector<LeftWallPoint> wall_points_;
    LeftWallModel wall_model_;
    LeftWallControlDecision pending_decision_;
    std::uint64_t wall_epoch_ = 0;
    std::size_t consecutive_wall_misses_ = 0;
    std::size_t stationary_resample_attempts_ = 0;
    std::size_t front_invalid_samples_ = 0;
    double desired_distance_m_ = 0.15;
    bool desired_distance_captured_ = false;
    bool have_control_metrics_ = false;
    std::vector<StopGoReplayOdomSample> pending_replay_odom_samples_;
    bool post_turn_verified_since_last_commit_ = false;
    std::ofstream log_;
};

} // namespace robot_slamd
