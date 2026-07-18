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
        for (std::size_t index = 0; index < records.size(); ++index) {
            auto record = records[index];
            const double current_time_s = record.snapshot.wheel.timestamp_s;
            const double interval_s = current_time_s - previous_full_time_s;
            if (index > 0 && interval_s > 0.0) {
                const double linear_mps = record.motion.actual_distance_mm / 1000.0 / interval_s;
                const double angular_rad_s = record.motion.actual_angle_deg *
                    3.14159265358979323846 / 180.0 / interval_s;
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
            now_s = std::max(now_s + 1e-6, current_time_s);
            const auto step = application.step(record.snapshot, now_s);
            if (!step.ok) {
                report.termination_reason = "replay_core_step_failed:" + step.reason;
                return report;
            }
            report.replay_records.push_back(record);
            previous_full_time_s = current_time_s;
        }
        report.stable_samples = records.size();
        report.completed_steps = records.size() > 0 ? records.size() - 1 : 0;
        report.commands_completed = report.completed_steps;
        report.map_commits = application.core().report().current_map_revision;
        report.map_revision = application.core().report().current_map_revision;
        report.map_cells = application.core().report().sparse_map_cell_count;
        const auto map_snapshot = application.core().sparse_map_snapshot();
        for (const auto &cell : map_snapshot.cells()) {
            if (cell.evidence >= map_snapshot.occupied_threshold() &&
                cell.key.y >= 10 && cell.key.y <= 13 &&
                cell.key.x >= -1 && cell.key.x <= 10) {
                report.left_wall_observed_cells++;
            }
        }
        report.command_speed_used_as_odometry = false;
        report.ground_truth_used_by_algorithm = false;
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
        report.ok = report.completed_steps > 0 && report.map_commits == report.stable_samples &&
                    report.final_map_checksum != 0;
        report.termination_reason = report.ok ? "replay_stop_go_completed" : "replay_acceptance_not_met";
        return report;
    }
};

} // namespace robot_slamd
