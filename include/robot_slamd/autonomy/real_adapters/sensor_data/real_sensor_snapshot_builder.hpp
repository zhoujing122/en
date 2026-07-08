#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_checker.hpp"

#include <utility>

namespace robot_slamd {

class RealSensorSnapshotBuilder {
public:
    RealSensorSnapshotBuilder() = default;

    explicit RealSensorSnapshotBuilder(
        RealSensorDataContractChecker checker)
        : checker_(std::move(checker)) {}

    RealSensorSnapshotBuildResult build_snapshot(
        const RealSensorRawPacket &packet,
        double now_s) const {
        RealSensorSnapshotBuildResult result;

        // Contract check.
        result.contract_result = checker_.check_packet(packet, now_s);
        if (!result.contract_result.ok) {
            result.failed.push_back("real_sensor_snapshot_contract_failed");
            result.failed.insert(result.failed.end(),
                                 result.contract_result.failed.begin(),
                                 result.contract_result.failed.end());
            result.warnings = result.contract_result.warnings;
            result.message = "real_sensor_snapshot_contract_failed";
            return result;
        }
        result.passed = result.contract_result.passed;
        result.warnings = result.contract_result.warnings;

        // ToF: timestamp is the request-window midpoint estimate.
        if (packet.has_tof) {
            result.snapshot.has_tof = true;
            result.snapshot.tof.timestamp_s =
                real_sensor_effective_time_s(packet.tof.timing);
            result.snapshot.tof.frame_id = packet.tof.frame_id;
            result.snapshot.tof.ranges_m = packet.tof.ranges_m;
            result.snapshot.tof.angle_min_rad = packet.tof.angle_min_rad;
            result.snapshot.tof.angle_increment_rad =
                packet.tof.angle_increment_rad;
            result.snapshot.tof.max_range_m = packet.tof.range_max_m;
        }

        // IMU: timestamp keeps the IMU sample timestamp semantics.
        if (packet.has_imu) {
            result.snapshot.has_imu = true;
            result.snapshot.imu.timestamp_s = packet.imu.timestamp_s;
            result.snapshot.imu.yaw_rate_rad_s = packet.imu.yaw_rate_rad_s;
            result.snapshot.imu.ax_mps2 = packet.imu.accel_x_mps2;
            result.snapshot.imu.ay_mps2 = packet.imu.accel_y_mps2;
            result.snapshot.imu.az_mps2 = packet.imu.accel_z_mps2;
        }

        // Wheel: timestamp is the request-window midpoint estimate.
        if (packet.has_wheel) {
            result.snapshot.has_wheel = true;
            result.snapshot.wheel.timestamp_s =
                real_sensor_effective_time_s(packet.wheel.timing);
            result.snapshot.wheel.valid = packet.wheel.valid;
            result.snapshot.wheel.linear_mps =
                packet.wheel.linear_velocity_mps;
            result.snapshot.wheel.angular_rad_s =
                packet.wheel.angular_velocity_rad_s;
        }

        // Finish.
        result.ok = true;
        result.passed.push_back("real_sensor_snapshot_build_ok");
        result.message = "real_sensor_snapshot_build_ok";
        return result;
    }

private:
    RealSensorDataContractChecker checker_;
};

} // namespace robot_slamd
