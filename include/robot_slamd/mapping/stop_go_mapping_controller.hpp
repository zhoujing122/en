#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/autonomy/ports/relative_motion_step_port.hpp"
#include "robot_slamd/runtime/mapping_settle_gate.hpp"
#include "robot_slamd/sensors/stable_three_tof_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
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
    IssueForwardStep,
    WaitMotionDone,
    WaitMappingSettle,
    AcquireThreeTof,
    CommitMappingSample,
    CheckLimit,
    Completed,
    Fault,
    Estop
};

inline const char *to_string(StopGoMappingControllerState state) {
    switch (state) {
    case StopGoMappingControllerState::Idle: return "IDLE";
    case StopGoMappingControllerState::WaitingForCoreReady: return "WAITING_FOR_CORE_READY";
    case StopGoMappingControllerState::InitialSettle: return "INITIAL_SETTLE";
    case StopGoMappingControllerState::InitialSample: return "INITIAL_SAMPLE";
    case StopGoMappingControllerState::IssueForwardStep: return "ISSUE_FORWARD_STEP";
    case StopGoMappingControllerState::WaitMotionDone: return "WAIT_MOTION_DONE";
    case StopGoMappingControllerState::WaitMappingSettle: return "WAIT_MAPPING_SETTLE";
    case StopGoMappingControllerState::AcquireThreeTof: return "ACQUIRE_THREE_TOF";
    case StopGoMappingControllerState::CommitMappingSample: return "COMMIT_MAPPING_SAMPLE";
    case StopGoMappingControllerState::CheckLimit: return "CHECK_LIMIT";
    case StopGoMappingControllerState::Completed: return "COMPLETED";
    case StopGoMappingControllerState::Fault: return "FAULT";
    case StopGoMappingControllerState::Estop: return "ESTOP";
    }
    return "UNKNOWN";
}

struct StopGoMappingControllerConfig {
    double forward_step_mm = 20.0;
    int max_steps = 20;
    double max_total_distance_mm = 400.0;
    double motion_max_rpm = 12.0;
    std::uint64_t motion_timeout_ms = 3000;
    MappingSettleGateConfig settle;
    StableThreeTofSamplerConfig tof;
    std::string log_path;
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
    return result;
}

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
};

struct StopGoMappingRunReport {
    bool ok = false;
    std::size_t completed_steps = 0;
    std::size_t commands_submitted = 0;
    std::size_t commands_completed = 0;
    std::size_t stable_samples = 0;
    std::size_t map_commits = 0;
    std::uint64_t map_revision = 0;
    std::size_t map_cells = 0;
    std::size_t left_wall_observed_cells = 0;
    std::size_t collision_count = 0;
    std::size_t map_writes_while_moving = 0;
    std::size_t core_wait_ticks = 0;
    bool core_ready_observed = false;
    bool command_speed_used_as_odometry = false;
    bool ground_truth_used_by_algorithm = false;
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
          config_(std::move(config)), gate_(config_.settle) {
        if (!log_path.empty()) config_.log_path = std::move(log_path);
        if (!config_.log_path.empty()) log_.open(config_.log_path, std::ios::out | std::ios::trunc);
        // Core startup owns gyro calibration, wheel baseline, odom_T_base,
        // map_T_odom and the pose buffer.  Stop-go cannot enter its formal
        // state machine until Core explicitly reports localization_ready().
        transition(StopGoMappingControllerState::WaitingForCoreReady);
    }

    bool tick(double now_s) {
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
            state_ == StopGoMappingControllerState::WaitMappingSettle) {
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
               state_ == StopGoMappingControllerState::Fault ||
               state_ == StopGoMappingControllerState::Estop;
    }
    StopGoMappingControllerState state() const { return state_; }
    const StopGoMappingRunReport &report() const { return report_; }

private:
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
    StopGoMappingControllerState state_ = StopGoMappingControllerState::Idle;
    StopGoMappingRunReport report_;
    RelativeMotionStepCommand command_;
    RelativeMotionStepResult last_motion_;
    std::uint64_t next_command_id_ = 1;
    double total_distance_mm_ = 0.0;
    std::ofstream log_;
};

} // namespace robot_slamd
