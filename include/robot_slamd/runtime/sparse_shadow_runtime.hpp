#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_contract_checker.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_map_port_adapter.hpp"
#include "robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/runtime/slam_runtime_mode.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

namespace robot_slamd {

struct SparseShadowRuntimeReport {
    bool ok = false;
    std::string runtime_mode = "sparse_shadow";
    std::string sensor_source = "deterministic_simulation";

    bool startup_zero_used = false;
    bool initial_yaw_measured_by_imu = false;
    bool initial_yaw_defined_by_startup_frame = false;
    bool map_from_odom_assumed_identity = false;

    bool sensor_port_constructed = false;
    bool wheel_imu_estimator_constructed = false;
    bool sparse_backend_constructed = false;

    std::size_t sensor_snapshot_count = 0;
    std::size_t wheel_imu_update_attempt_count = 0;
    std::size_t wheel_imu_update_success_count = 0;
    std::size_t wheel_imu_update_reject_count = 0;

    std::size_t backend_update_attempt_count = 0;
    std::size_t backend_accept_count = 0;
    std::size_t backend_reject_count = 0;
    std::size_t backend_fault_count = 0;

    std::size_t predicted_pose_handoff_count = 0;
    std::size_t missing_predicted_pose_count = 0;
    std::size_t native_multi_tof_snapshot_count = 0;

    bool native_multi_tof_consumed = false;
    bool legacy_projection_consumed = false;

    bool old_chunked_grid_constructed = false;
    bool old_matcher_constructed = false;
    bool old_correction_constructed = false;

    bool real_hardware_accessed = false;
    bool real_motion_attempted = false;
    bool real_map_write_attempted = false;
    bool real_pose_writeback_attempted = false;

    RobotPose2D last_odom_pose;
    RobotPose2D last_predicted_map_pose;
    std::size_t sparse_map_cell_count = 0;

    std::string last_fault = "none";
    std::string last_message;
    std::string summary;
};

inline std::string write_sparse_shadow_runtime_report(
    const SparseShadowRuntimeReport &report) {
    std::ostringstream out;
    out << "Sparse Shadow Runtime Report\n";
    out << "ok=" << bool_yaml(report.ok) << "\n";
    out << "runtime_mode=" << report.runtime_mode << "\n";
    out << "sensor_source=" << report.sensor_source << "\n";
    out << "startup_zero_used=" << bool_yaml(report.startup_zero_used) << "\n";
    out << "initial_yaw_measured_by_imu="
        << bool_yaml(report.initial_yaw_measured_by_imu) << "\n";
    out << "initial_yaw_defined_by_startup_frame="
        << bool_yaml(report.initial_yaw_defined_by_startup_frame) << "\n";
    out << "map_from_odom_assumed_identity="
        << bool_yaml(report.map_from_odom_assumed_identity) << "\n";
    out << "sensor_port_constructed="
        << bool_yaml(report.sensor_port_constructed) << "\n";
    out << "wheel_imu_estimator_constructed="
        << bool_yaml(report.wheel_imu_estimator_constructed) << "\n";
    out << "sparse_backend_constructed="
        << bool_yaml(report.sparse_backend_constructed) << "\n";
    out << "sensor_snapshot_count=" << report.sensor_snapshot_count << "\n";
    out << "wheel_imu_update_attempt_count="
        << report.wheel_imu_update_attempt_count << "\n";
    out << "wheel_imu_update_success_count="
        << report.wheel_imu_update_success_count << "\n";
    out << "wheel_imu_update_reject_count="
        << report.wheel_imu_update_reject_count << "\n";
    out << "backend_update_attempt_count="
        << report.backend_update_attempt_count << "\n";
    out << "backend_accept_count=" << report.backend_accept_count << "\n";
    out << "backend_reject_count=" << report.backend_reject_count << "\n";
    out << "backend_fault_count=" << report.backend_fault_count << "\n";
    out << "predicted_pose_handoff_count="
        << report.predicted_pose_handoff_count << "\n";
    out << "missing_predicted_pose_count="
        << report.missing_predicted_pose_count << "\n";
    out << "native_multi_tof_snapshot_count="
        << report.native_multi_tof_snapshot_count << "\n";
    out << "native_multi_tof_consumed="
        << bool_yaml(report.native_multi_tof_consumed) << "\n";
    out << "legacy_projection_consumed="
        << bool_yaml(report.legacy_projection_consumed) << "\n";
    out << "old_chunked_grid_constructed="
        << bool_yaml(report.old_chunked_grid_constructed) << "\n";
    out << "old_matcher_constructed="
        << bool_yaml(report.old_matcher_constructed) << "\n";
    out << "old_correction_constructed="
        << bool_yaml(report.old_correction_constructed) << "\n";
    out << "real_hardware_accessed="
        << bool_yaml(report.real_hardware_accessed) << "\n";
    out << "real_motion_attempted="
        << bool_yaml(report.real_motion_attempted) << "\n";
    out << "real_map_write_attempted="
        << bool_yaml(report.real_map_write_attempted) << "\n";
    out << "real_pose_writeback_attempted="
        << bool_yaml(report.real_pose_writeback_attempted) << "\n";
    out << "last_odom_pose=" << report.last_odom_pose.x_m << ","
        << report.last_odom_pose.y_m << ","
        << report.last_odom_pose.yaw_rad << "\n";
    out << "last_predicted_map_pose="
        << report.last_predicted_map_pose.x_m << ","
        << report.last_predicted_map_pose.y_m << ","
        << report.last_predicted_map_pose.yaw_rad << "\n";
    out << "sparse_map_cell_count=" << report.sparse_map_cell_count << "\n";
    out << "last_fault=" << report.last_fault << "\n";
    out << "last_message=" << report.last_message << "\n";
    out << "summary=" << report.summary << "\n";
    return out.str();
}

class SparseShadowDeterministicSensorPort final : public RobotSlamSensorPort {
public:
    bool ready() const override { return true; }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        if (!std::isfinite(now_s) || now_s <= last_timestamp_s_) {
            now_s = last_timestamp_s_ + 0.10;
        }
        last_timestamp_s_ = now_s;
        sample_index_++;

        RobotSlamSensorSnapshot snapshot;
        snapshot.has_tof = false;
        snapshot.has_multi_tof = true;
        snapshot.multi_tof.has_front = true;
        snapshot.multi_tof.has_left = true;
        snapshot.multi_tof.has_right = true;
        snapshot.multi_tof.valid_tof_count = 3;
        snapshot.multi_tof.front = frame(1000, 0.0, "front", now_s);
        snapshot.multi_tof.left = frame(1200, kPi / 2.0, "left", now_s);
        snapshot.multi_tof.right = frame(1200, -kPi / 2.0, "right", now_s);
        snapshot.multi_tof.synchronized_time_s = now_s;
        snapshot.multi_tof.source = "sparse_shadow_deterministic_simulation";

        snapshot.has_wheel = true;
        snapshot.wheel.timestamp_s = now_s;
        snapshot.wheel.linear_mps = 0.05;
        snapshot.wheel.angular_rad_s = 0.0;
        snapshot.wheel.valid = true;

        snapshot.has_imu = true;
        snapshot.imu.timestamp_s = now_s;
        snapshot.imu.yaw_rate_rad_s = 0.0;
        snapshot.imu.az_mps2 = 9.8;
        return snapshot;
    }

    std::string status() const override {
        return "sparse_shadow_deterministic_sensor_ready";
    }

private:
    static ScalarTofSnapshotFrame frame(std::uint16_t distance_mm,
                                        double yaw_rad,
                                        const std::string &frame_id,
                                        double timestamp_s) {
        ScalarTofSnapshotFrame out;
        out.distance_mm = distance_mm;
        out.distance_m = static_cast<double>(distance_mm) / 1000.0;
        out.confidence = 100;
        out.echo_tag_u48 = 0x120000 + distance_mm;
        out.effective_timestamp_s = timestamp_s;
        out.protocol_valid = true;
        out.usable_for_mapping = true;
        out.full_fov_rad = 1.6 * kPi / 180.0;
        out.mount_yaw_rad = yaw_rad;
        out.frame_id = frame_id;
        out.source = "sparse_shadow_deterministic_simulation";
        return out;
    }

    double last_timestamp_s_ = 0.0;
    std::size_t sample_index_ = 0;
};

inline SlamBackendContractChecker sparse_shadow_backend_checker() {
    SlamBackendContractOptions options;
    options.require_tof = false;
    options.require_imu_or_wheel = true;
    options.allow_predicted_pose_missing = true;
    return SlamBackendContractChecker(options);
}

class SparseShadowRuntime final {
public:
    explicit SparseShadowRuntime(Config config)
        : config_(std::move(config)) {}

    SparseShadowRuntimeReport run(double duration_s,
                                  const std::string &run_dir) {
        SparseShadowRuntimeReport report;
        report.runtime_mode = "sparse_shadow";
        report.sensor_source = config_.sparse_shadow_sensor_source;
        report.startup_zero_used = true;
        report.initial_yaw_measured_by_imu = false;
        report.initial_yaw_defined_by_startup_frame = true;
        report.map_from_odom_assumed_identity = true;

        if (config_.sparse_shadow_sensor_source != "deterministic_simulation") {
            report.ok = false;
            report.last_fault = "sparse_shadow_sensor_source_unsupported";
            report.last_message = "SparseShadow hardware/replay source is fail-closed in M3-D1.1";
            report.summary = "sparse_shadow_runtime_rejected";
            write_report_file(run_dir, report);
            return report;
        }

        auto sensor_port = std::make_unique<SparseShadowDeterministicSensorPort>();
        report.sensor_port_constructed = true;

        WheelImuDeadReckoningConfig odom_config;
        odom_config.wheel_radius_m = config_.wheel_radius_left_m;
        odom_config.wheel_base_m = config_.wheel_base_m;
        odom_config.allow_imu_missing_wheel_yaw_fallback = false;
        WheelImuDeadReckoning2D estimator(odom_config);
        report.wheel_imu_estimator_constructed = true;
        const RobotPose2D startup_zero_pose{};
        auto reset = estimator.reset(startup_zero_pose);
        if (!reset.ok) {
            report.ok = false;
            report.last_fault = to_string(reset.fault);
            report.last_message = reset.message;
            report.summary = "sparse_shadow_runtime_odometry_reset_failed";
            write_report_file(run_dir, report);
            return report;
        }

        SparseMultiTofOccupancyBackendOptions backend_options;
        backend_options.minimum_accepted_snapshots_for_good_quality = 1;
        backend_options.minimum_valid_rays_for_good_quality = 1;
        backend_options.minimum_observed_cells_for_good_quality = 1;
        backend_options.minimum_angular_bins_for_good_quality = 1;
        auto sparse_backend = std::make_shared<SparseMultiTofOccupancyBackendBinding>(backend_options);
        report.sparse_backend_constructed = true;
        SlamBackendMapPortAdapter map_port(sparse_backend,
                                           sparse_shadow_backend_checker());

        const double step_s = 0.10;
        const double clamped_duration_s =
            std::isfinite(duration_s) && duration_s > 0.0 ? duration_s : step_s;
        const int max_steps = std::max(1, std::min(20,
            static_cast<int>(std::ceil(clamped_duration_s / step_s))));

        for (int i = 0; i < max_steps; ++i) {
            const double now_s = static_cast<double>(i + 1) * step_s;
            auto snapshot = sensor_port->latest_snapshot(now_s);
            report.sensor_snapshot_count++;
            if (snapshot.has_multi_tof) {
                report.native_multi_tof_snapshot_count++;
            }

            if (!snapshot.has_wheel || !snapshot.has_imu) {
                report.wheel_imu_update_reject_count++;
                report.missing_predicted_pose_count++;
                continue;
            }

            report.wheel_imu_update_attempt_count++;
            const auto odom = estimator.update(snapshot.wheel, snapshot.imu);
            if (!odom.ok) {
                report.wheel_imu_update_reject_count++;
                report.last_fault = to_string(odom.fault);
                report.last_message = odom.message;
                continue;
            }
            report.wheel_imu_update_success_count++;

            const RobotPose2D odom_pose = estimator.estimated_pose();
            const RobotPose2D predicted_map_pose = odom_pose;
            report.last_odom_pose = odom_pose;
            report.last_predicted_map_pose = predicted_map_pose;

            RobotSlamMapUpdateInput update;
            update.timestamp_s = now_s;
            update.snapshot = snapshot;
            update.predicted_map_pose = predicted_map_pose;
            update.has_predicted_map_pose = true;
            update.mapping_commit_allowed = true;
            update.source = "sparse_shadow_runtime";
            report.predicted_pose_handoff_count++;
            report.backend_update_attempt_count++;
            const bool accepted = map_port.integrate_map_update(update);
            if (accepted) {
                report.backend_accept_count++;
            } else {
                report.backend_reject_count++;
            }
        }

        const auto backend_report = sparse_backend->report();
        report.native_multi_tof_consumed =
            backend_report.native_multi_tof_backend_consumption;
        report.legacy_projection_consumed = backend_report.legacy_projection_consumed;
        report.sparse_map_cell_count = backend_report.active_cell_count;
        report.ok = report.sensor_port_constructed &&
                    report.wheel_imu_estimator_constructed &&
                    report.sparse_backend_constructed &&
                    report.predicted_pose_handoff_count > 0 &&
                    report.backend_accept_count > 0 &&
                    report.native_multi_tof_consumed &&
                    !report.legacy_projection_consumed;
        report.last_fault = report.ok ? "none" : "sparse_shadow_runtime_failed";
        report.last_message = report.ok ? "sparse_shadow_runtime_ok"
                                        : "sparse_shadow_runtime_failed";
        report.summary = report.ok ? "Sparse Shadow runtime reached sparse backend"
                                   : "Sparse Shadow runtime did not reach sparse backend";
        write_report_file(run_dir, report);
        return report;
    }

private:
    static void write_report_file(const std::string &run_dir,
                                  const SparseShadowRuntimeReport &report) {
        if (run_dir.empty()) {
            return;
        }
        std::ofstream out(run_dir + "/sparse_shadow_runtime_report.txt");
        out << write_sparse_shadow_runtime_report(report);
    }

    Config config_;
};

inline SparseShadowRuntimeReport run_sparse_shadow_runtime(
    const Config &config,
    double duration_s,
    const std::string &run_dir) {
    SparseShadowRuntime runtime(config);
    return runtime.run(duration_s, run_dir);
}

} // namespace robot_slamd
