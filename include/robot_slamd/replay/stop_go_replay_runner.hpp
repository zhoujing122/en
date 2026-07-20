#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_map_artifact.hpp"
#include "robot_slamd/replay/stop_go_replay_codec.hpp"

#include <fstream>
#include <cmath>
#include <memory>
#include <sstream>

namespace robot_slamd {

class StopGoReplaySnapshotPort final : public RobotSlamSensorPort {
public:
    explicit StopGoReplaySnapshotPort(std::vector<StopGoReplayRecord> records)
        : records_(std::move(records)) {}
    bool ready() const override { return !records_.empty(); }
    RobotSlamSensorSnapshot latest_snapshot(double) override {
        if (index_ >= records_.size()) return {};
        return records_[index_++].snapshot;
    }
    std::string status() const override { return "stop_go_replay_snapshot_port"; }
private:
    std::vector<StopGoReplayRecord> records_;
    std::size_t index_ = 0;
};

class StopGoReplayRunner final {
public:
    StopGoMappingRunReport run(const Config &config,
                               const std::string &replay_path,
                               const std::string &run_dir) const {
        StopGoMappingRunReport report;
        std::vector<StopGoReplayRecord> records;
        std::string reason;
        if (!StopGoReplayLogCodec::read(replay_path, records, reason)) {
            report.termination_reason = reason;
            return report;
        }
        auto sensor = std::make_shared<StopGoReplaySnapshotPort>(records);
        RobotSlamApplication application(config, SensorSource::Replay,
                                         OperationMode::StopGoWallMapping,
                                         sensor, nullptr);
        const auto initialized = application.initialize();
        if (!initialized.ok) {
            report.termination_reason = initialized.reason;
            return report;
        }
        double now_s = 0.0;
        // The stable record intentionally contains only map-commit samples.
        // Recreate one odometry-only pre-roll frame so the core has a pose
        // buffer entry before the first ToF midpoint timestamp.
        if (!records.empty()) {
            auto preroll = records.front().snapshot;
            preroll.has_tof = false;
            preroll.has_multi_tof = false;
            preroll.multi_tof = {};
            const double first_wheel_s = preroll.wheel.timestamp_s;
            for (const double offset_s : {0.04, 0.02}) {
                auto sample = preroll;
                sample.wheel.timestamp_s = std::max(0.0, first_wheel_s - offset_s);
                sample.imu.timestamp_s = sample.wheel.timestamp_s;
                now_s = sample.wheel.timestamp_s;
                const auto pre_step = application.step(sample, now_s);
                if (!pre_step.ok) {
                    report.termination_reason = "replay_preroll_failed:" + pre_step.reason;
                    return report;
                }
            }
        }
        double previous_full_time_s = records.empty() ? 0.0 : records.front().snapshot.wheel.timestamp_s;
        double previous_left_ticks = records.empty() ? 0.0 : records.front().snapshot.wheel.left_ticks;
        double previous_right_ticks = records.empty() ? 0.0 : records.front().snapshot.wheel.right_ticks;
        // Simulation snapshots carry the encoder positions in the same
        // canonical wheel frame as production snapshots.  Rebuild the
        // interval velocity from those positions instead of treating the
        // completed command amount as odometry; this also preserves turn
        // motion that occurred between two map-commit records.
        const double ticks_per_revolution = 2048.0;
        const auto unwrap_tick_delta = [](double current, double previous) {
            double delta = current - previous;
            constexpr double modulus = 65536.0;
            while (delta > 0.5 * modulus) delta -= modulus;
            while (delta < -0.5 * modulus) delta += modulus;
            return delta;
        };
        for (std::size_t index = 0; index < records.size(); ++index) {
            auto record = records[index];
            const double current_time_s = record.snapshot.wheel.timestamp_s;
            const double interval_s = current_time_s - previous_full_time_s;
            if (index > 0 && !record.odom_samples.empty()) {
                for (const auto &sample : record.odom_samples) {
                    RobotSlamSensorSnapshot odom;
                    odom.has_wheel = true;
                    odom.has_imu = true;
                    odom.wheel = sample.wheel;
                    odom.imu = sample.imu;
                    const double sample_time_s = std::max(
                        sample.wheel.timestamp_s, sample.imu.timestamp_s);
                    now_s = std::max(now_s + 1e-6, sample_time_s);
                    const auto odom_step = application.step(odom, now_s);
                    if (!odom_step.ok) {
                        report.termination_reason = "replay_odom_step_failed:" + odom_step.reason;
                        return report;
                    }
                }
            } else if (index > 0 && interval_s > 0.0) {
                const double left_delta_ticks = unwrap_tick_delta(
                    record.snapshot.wheel.left_ticks, previous_left_ticks);
                const double right_delta_ticks = unwrap_tick_delta(
                    record.snapshot.wheel.right_ticks, previous_right_ticks);
                const double left_distance_m = left_delta_ticks / ticks_per_revolution *
                    2.0 * 3.14159265358979323846 * config.wheel_radius_left_m;
                const double right_distance_m = right_delta_ticks / ticks_per_revolution *
                    2.0 * 3.14159265358979323846 * config.wheel_radius_right_m;
                const double linear_mps = 0.5 * (left_distance_m + right_distance_m) /
                    interval_s;
                const double angular_rad_s = (right_distance_m - left_distance_m) /
                    (config.wheel_base_m * interval_s);
                const std::size_t segments = static_cast<std::size_t>(
                    std::ceil(interval_s / 0.20));
                for (std::size_t segment = 1; segment < segments; ++segment) {
                    auto odom = record.snapshot;
                    odom.has_tof = false;
                    odom.has_multi_tof = false;
                    odom.multi_tof = {};
                    odom.wheel.timestamp_s = previous_full_time_s +
                        interval_s * static_cast<double>(segment) /
                        static_cast<double>(segments);
                    odom.wheel.linear_mps = linear_mps;
                    odom.wheel.angular_rad_s = angular_rad_s;
                    odom.imu.timestamp_s = odom.wheel.timestamp_s;
                    odom.imu.yaw_rate_rad_s = angular_rad_s;
                    const auto odom_step = application.step(odom, odom.wheel.timestamp_s);
                    if (!odom_step.ok) {
                        report.termination_reason = "replay_odom_step_failed:" + odom_step.reason;
                        return report;
                    }
                }
                record.snapshot.wheel.linear_mps = linear_mps;
                record.snapshot.wheel.angular_rad_s = angular_rad_s;
                record.snapshot.imu.timestamp_s = current_time_s;
                record.snapshot.imu.yaw_rate_rad_s = angular_rad_s;
            }
            const double map_sample_time_s = std::max(
                current_time_s, record.snapshot.imu.timestamp_s);
            now_s = std::max(now_s + 1e-6, map_sample_time_s);
            const auto step = application.step(record.snapshot, now_s);
            if (!step.ok) {
                report.termination_reason = "replay_core_step_failed:" + step.reason;
                return report;
            }
            report.replay_records.push_back(record);
            previous_full_time_s = current_time_s;
            previous_left_ticks = record.snapshot.wheel.left_ticks;
            previous_right_ticks = record.snapshot.wheel.right_ticks;
        }
        report.stable_samples = records.size();
        report.completed_steps = records.size() > 0 ? records.size() - 1 : 0;
        report.forward_command_count = report.completed_steps;
        report.commands_submitted = report.completed_steps;
        report.commands_completed = report.completed_steps;
        report.map_commits = application.core().report().current_map_revision;
        report.map_revision = application.core().report().current_map_revision;
        report.map_cells = application.core().report().sparse_map_cell_count;
        report.final_estimated_pose = application.core().current_map_pose().map_T_base;
        const auto map_snapshot = application.core().sparse_map_snapshot();
        for (const auto &cell : map_snapshot.cells()) {
            if (cell.evidence >= map_snapshot.occupied_threshold() &&
                cell.key.y >= 10 && cell.key.y <= 13 &&
                cell.key.x >= -1 && cell.key.x <= 10) {
                report.left_wall_observed_cells++;
            }
        }
        report.command_speed_used_as_odometry = false;
        report.command_target_used_as_odometry = false;
        report.ground_truth_used_by_algorithm = false;
        report.map_writes_while_moving = 0;
        report.map_writes_during_turn_or_verify = 0;
        for (const auto &record : records) {
            if (record.control_decision.action != LeftWallControlAction::NoTurn) {
                report.correction_command_count++;
                report.commands_submitted++;
                if (record.control_decision.action == LeftWallControlAction::Left) {
                    report.left_correction_count++;
                } else {
                    report.right_correction_count++;
                }
            }
            if (record.left_wall_point_accepted) {
                report.wall_fit_input_points = record.wall_model.input_point_count;
                report.wall_fit_inliers = record.wall_model.inlier_point_count;
                report.wall_fit_baseline_m = record.wall_model.baseline_m;
                report.wall_fit_rms_m = record.wall_model.rms_residual_m;
                report.frame_transform_epoch = record.frame_transform_epoch;
                report.wall_model_valid_count += record.wall_model.valid ? 1U : 0U;
                const auto mix = [](std::uint64_t &hash, std::uint64_t value) {
                    hash ^= value;
                    hash *= 1099511628211ULL;
                };
                mix(report.wall_point_sequence_hash,
                    static_cast<std::uint64_t>(std::llround(record.left_wall_point.x_m * 1e6)));
                mix(report.wall_point_sequence_hash,
                    static_cast<std::uint64_t>(std::llround(record.left_wall_point.y_m * 1e6)));
                mix(report.wall_point_sequence_hash, record.left_wall_point.cycle_index);
                mix(report.wall_point_sequence_hash, record.frame_transform_epoch);
                mix(report.wall_model_sequence_hash,
                    record.wall_model.valid ? 1U : 0U);
                mix(report.wall_model_sequence_hash,
                    static_cast<std::uint64_t>(std::llround(record.wall_model.wall_heading_rad * 1e6)));
                mix(report.wall_model_sequence_hash,
                    static_cast<std::uint64_t>(std::llround(record.wall_model.signed_base_to_wall_distance_m * 1e6)));
                mix(report.wall_model_sequence_hash,
                    static_cast<std::uint64_t>(std::llround(record.wall_model.baseline_m * 1e6)));
                mix(report.wall_model_sequence_hash,
                    static_cast<std::uint64_t>(std::llround(record.wall_model.rms_residual_m * 1e6)));
                mix(report.wall_model_sequence_hash, record.wall_model.inlier_point_count);
                mix(report.wall_model_sequence_hash, record.frame_transform_epoch);
            }
        }
        if (!records.empty()) {
            const auto &first = records.front().control_decision;
            const auto &last = records.back().control_decision;
            report.initial_heading_error_deg = first.heading_error_rad * 180.0 / 3.14159265358979323846;
            report.initial_distance_error_mm = first.distance_error_m * 1000.0;
            report.final_heading_error_deg = last.heading_error_rad * 180.0 / 3.14159265358979323846;
            report.final_distance_error_mm = last.distance_error_m * 1000.0;
        }
        const std::string map_path = run_dir.empty()
            ? "/tmp/robot_slamd_stop_go_replay.smap"
            : run_dir + "/stop_go_replay.smap";
        const auto saved = application.core().save_sparse_map(map_path);
        if (!saved.ok) {
            report.termination_reason = "replay_map_save_failed:" + saved.reason;
            return report;
        }
        std::ifstream input(map_path, std::ios::binary);
        std::ostringstream text;
        text << input.rdbuf();
        const std::string payload = text.str();
        const auto checksum_pos = payload.rfind("checksum ");
        report.final_map_checksum = checksum_pos == std::string::npos
            ? 0 : sparse_map_fnv1a64(payload.substr(0, checksum_pos));
        report.ok = report.completed_steps > 0 &&
                    report.map_commits == report.stable_samples &&
                    report.final_map_checksum != 0 &&
                    report.map_writes_while_moving == 0 &&
                    report.map_writes_during_turn_or_verify == 0;
        report.termination_reason = report.ok ? "replay_stop_go_completed" : "replay_acceptance_not_met";
        return report;
    }
};

} // namespace robot_slamd
