#pragma once

#include "robot_slamd/autonomy/e2e/autonomous_slam_e2e_scenario_types.hpp"

#include <string>

namespace robot_slamd {

class AutonomousSlamE2EScenarioBuilder {
public:
    AutonomousSlamE2EScenarioData build(AutonomousSlamE2EScenarioKind kind,
                                        double start_time_s) const {
        switch (kind) {
        case AutonomousSlamE2EScenarioKind::MinimalMapAlreadyGood:
            return build_minimal_map_already_good(start_time_s);
        case AutonomousSlamE2EScenarioKind::ActiveScanThenMapGood:
            return build_active_scan_then_map_good(start_time_s);
        case AutonomousSlamE2EScenarioKind::SensorContractFailure:
            return build_sensor_contract_failure(start_time_s);
        case AutonomousSlamE2EScenarioKind::SlamBackendUpdateRejected:
            return build_slam_backend_update_rejected(start_time_s);
        case AutonomousSlamE2EScenarioKind::SlamBackendQualityInvalid:
            return build_slam_backend_quality_invalid(start_time_s);
        case AutonomousSlamE2EScenarioKind::MotionRejected:
            return build_motion_rejected(start_time_s);
        case AutonomousSlamE2EScenarioKind::MapQualityStuck:
            return build_map_quality_stuck(start_time_s);
        }
        return {};
    }

private:
    AutonomousSlamE2EScenarioData build_minimal_map_already_good(
        double start_time_s) const {
        AutonomousSlamE2EScenarioData data;
        data.sensor_snapshots = make_snapshots(start_time_s, 8);
        const auto good = make_quality(true, 0.85, 0.80, 6, "map_quality_good");
        data.backend_qualities = {good, good, good, good, good};
        data.backend_update_results = {
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               true,
                               1,
                               good,
                               "slam_update_ok"),
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               true,
                               2,
                               good,
                               "slam_update_ok"),
        };
        return data;
    }

    AutonomousSlamE2EScenarioData build_active_scan_then_map_good(
        double start_time_s) const {
        AutonomousSlamE2EScenarioData data;
        data.sensor_snapshots = make_snapshots(start_time_s, 20);
        const auto poor = make_quality(false, 0.20, 0.15, 1, "map_quality_poor");
        const auto good = make_quality(true, 0.82, 0.78, 6, "map_quality_good");
        data.backend_qualities = {poor, poor, poor, poor, poor, good, good, good};
        data.backend_update_results = {
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               false,
                               1,
                               poor,
                               "slam_update_ok"),
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               true,
                               2,
                               poor,
                               "slam_update_ok"),
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               true,
                               3,
                               good,
                               "slam_update_ok"),
        };
        return data;
    }

    AutonomousSlamE2EScenarioData build_sensor_contract_failure(
        double start_time_s) const {
        AutonomousSlamE2EScenarioData data;
        data.sensor_snapshots = {make_invalid_tof_snapshot(start_time_s)};
        const auto poor = make_quality(false, 0.10, 0.10, 0, "map_quality_poor");
        data.backend_qualities = {poor};
        data.backend_update_results = {
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               false,
                               1,
                               poor,
                               "slam_update_ok"),
        };
        return data;
    }

    AutonomousSlamE2EScenarioData build_slam_backend_update_rejected(
        double start_time_s) const {
        AutonomousSlamE2EScenarioData data;
        data.sensor_snapshots = make_snapshots(start_time_s, 6);
        const auto poor = make_quality(false, 0.20, 0.20, 1, "map_quality_poor");
        data.backend_qualities = {poor, poor};
        data.backend_update_results = {
            make_update_result(SlamBackendUpdateStatus::Rejected,
                               false,
                               false,
                               1,
                               poor,
                               "slam_update_rejected"),
        };
        return data;
    }

    AutonomousSlamE2EScenarioData build_slam_backend_quality_invalid(
        double start_time_s) const {
        AutonomousSlamE2EScenarioData data;
        data.sensor_snapshots = make_snapshots(start_time_s, 6);
        const auto invalid = make_quality(false, 2.0, 0.20, 1, "map_quality_invalid");
        data.backend_qualities = {invalid, invalid};
        data.backend_update_results = {
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               false,
                               1,
                               invalid,
                               "slam_update_ok"),
        };
        return data;
    }

    AutonomousSlamE2EScenarioData build_motion_rejected(double start_time_s) const {
        auto data = build_active_scan_then_map_good(start_time_s);
        data.reject_motion = true;
        return data;
    }

    AutonomousSlamE2EScenarioData build_map_quality_stuck(double start_time_s) const {
        AutonomousSlamE2EScenarioData data;
        data.sensor_snapshots = make_snapshots(start_time_s, 20);
        const auto poor = make_quality(false, 0.18, 0.12, 1, "map_quality_poor");
        data.backend_qualities = {poor, poor, poor, poor, poor, poor, poor, poor};
        data.backend_update_results = {
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               false,
                               1,
                               poor,
                               "slam_update_ok"),
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               false,
                               2,
                               poor,
                               "slam_update_ok"),
            make_update_result(SlamBackendUpdateStatus::Accepted,
                               true,
                               false,
                               3,
                               poor,
                               "slam_update_ok"),
        };
        return data;
    }

    std::vector<RobotSlamSensorSnapshot> make_snapshots(double start_time_s,
                                                        int count) const {
        std::vector<RobotSlamSensorSnapshot> snapshots;
        for (int i = 0; i < count; ++i) {
            snapshots.push_back(make_valid_snapshot(start_time_s + i * 0.10));
        }
        return snapshots;
    }

    RobotSlamSensorSnapshot make_valid_snapshot(double timestamp_s) const {
        RobotSlamSensorSnapshot snapshot;
        snapshot.has_tof = true;
        snapshot.tof.timestamp_s = timestamp_s;
        snapshot.tof.ranges_m = {0.55, 0.62, 0.70, 0.76};
        snapshot.tof.angle_min_rad = -0.15;
        snapshot.tof.angle_increment_rad = 0.10;
        snapshot.tof.max_range_m = 3.0;
        snapshot.tof.frame_id = "front_tof";
        snapshot.has_imu = true;
        snapshot.imu.timestamp_s = timestamp_s;
        snapshot.imu.yaw_rate_rad_s = 0.0;
        snapshot.imu.ax_mps2 = 0.0;
        snapshot.imu.ay_mps2 = 0.0;
        snapshot.imu.az_mps2 = 9.8;
        snapshot.has_wheel = true;
        snapshot.wheel.timestamp_s = timestamp_s;
        snapshot.wheel.valid = true;
        return snapshot;
    }

    RobotSlamSensorSnapshot make_invalid_tof_snapshot(double timestamp_s) const {
        auto snapshot = make_valid_snapshot(timestamp_s);
        snapshot.tof.ranges_m.clear();
        snapshot.tof.frame_id.clear();
        return snapshot;
    }

    SlamBackendUpdateResult make_update_result(
        SlamBackendUpdateStatus status,
        bool map_updated,
        bool keyframe_added,
        int integrated_scan_count,
        const RobotSlamMapQuality &quality,
        const std::string &message) const {
        SlamBackendUpdateResult result;
        result.status = status;
        result.fault = status == SlamBackendUpdateStatus::Accepted
                           ? SlamBackendFault::None
                           : SlamBackendFault::UpdateRejected;
        result.map_updated = map_updated;
        result.keyframe_added = keyframe_added;
        result.integrated_scan_count = integrated_scan_count;
        result.update_latency_s = 0.05;
        result.quality = quality;
        result.message = message;
        return result;
    }

    RobotSlamMapQuality make_quality(bool good_enough,
                                     double coverage_ratio,
                                     double yaw_coverage_ratio,
                                     int valid_scan_count,
                                     const std::string &reason) const {
        RobotSlamMapQuality quality;
        quality.good_enough = good_enough;
        quality.coverage_ratio = coverage_ratio;
        quality.yaw_coverage_ratio = yaw_coverage_ratio;
        quality.valid_scan_count = valid_scan_count;
        quality.reason = reason;
        return quality;
    }
};

} // namespace robot_slamd
