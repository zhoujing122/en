#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_types.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_checker.hpp"

#include <cmath>
#include <limits>
#include <string>

namespace robot_slamd {

class MultiTofSnapshotBuilder {
public:
    MultiTofSnapshotBuilder(
        MultiTofContractOptions contract_options = {},
        MultiTofSyncOptions sync_options = {},
        MultiTofSnapshotBuildOptions build_options = {})
        : contract_options_(contract_options),
          sync_options_(sync_options),
          build_options_(build_options),
          sync_checker_(contract_options, sync_options) {}

    MultiTofSnapshotBuildResult build(
        const MultiTofRawPacket &packet,
        double now_s) const {
        if (!build_options_.enabled) {
            return reject(MultiTofSnapshotBuildFault::BuilderDisabled,
                          "multi_tof_snapshot_builder_disabled");
        }

        auto sync = sync_checker_.check_packet(packet, now_s);
        if (build_options_.require_sync_pass && !sync.ok) {
            auto result = reject(MultiTofSnapshotBuildFault::SyncFailed,
                                 "multi_tof_snapshot_sync_failed");
            result.sync_result = sync;
            result.failed.insert(result.failed.end(),
                                 sync.failed.begin(),
                                 sync.failed.end());
            return result;
        }
        if (sync.valid_tof_count < build_options_.min_required_tof_count) {
            auto result = reject(MultiTofSnapshotBuildFault::TooFewTofFrames,
                                 "multi_tof_snapshot_too_few_tof_frames");
            result.sync_result = sync;
            return result;
        }
        if (!std::isfinite(sync.times.multi_tof_time_s)) {
            auto result = reject(
                MultiTofSnapshotBuildFault::InvalidSynchronizedTime,
                "multi_tof_snapshot_invalid_synchronized_time");
            result.sync_result = sync;
            return result;
        }

        MultiTofSnapshotBuildResult result;
        result.sync_result = sync;
        result.valid_tof_count = sync.valid_tof_count;
        result.degraded = sync.degraded;
        result.snapshot.has_multi_tof = true;
        result.snapshot.multi_tof.has_front = packet.has_front;
        result.snapshot.multi_tof.has_left = packet.has_left;
        result.snapshot.multi_tof.has_right = packet.has_right;
        if (packet.has_front) {
            result.snapshot.multi_tof.front =
                convert_frame(packet.front, packet.front.timing.estimated_sample_time_s);
        }
        if (packet.has_left) {
            result.snapshot.multi_tof.left =
                convert_frame(packet.left, packet.left.timing.estimated_sample_time_s);
        }
        if (packet.has_right) {
            result.snapshot.multi_tof.right =
                convert_frame(packet.right, packet.right.timing.estimated_sample_time_s);
        }
        result.snapshot.multi_tof.synchronized_time_s =
            sync.times.multi_tof_time_s;
        result.snapshot.multi_tof.valid_tof_count = sync.valid_tof_count;
        result.snapshot.multi_tof.degraded = sync.degraded;
        result.snapshot.multi_tof.front_left_dt_s = sync.front_left_dt_s;
        result.snapshot.multi_tof.front_right_dt_s = sync.front_right_dt_s;
        result.snapshot.multi_tof.left_right_dt_s = sync.left_right_dt_s;
        result.snapshot.multi_tof.multi_tof_imu_dt_s = sync.multi_tof_imu_dt_s;
        result.snapshot.multi_tof.multi_tof_wheel_dt_s = sync.multi_tof_wheel_dt_s;

        if (packet.has_imu) {
            result.snapshot.has_imu = true;
            result.snapshot.imu.timestamp_s = packet.imu.timestamp_s;
            result.snapshot.imu.yaw_rate_rad_s = packet.imu.yaw_rate_rad_s;
            result.snapshot.imu.ax_mps2 = packet.imu.accel_x_mps2;
            result.snapshot.imu.ay_mps2 = packet.imu.accel_y_mps2;
            result.snapshot.imu.az_mps2 = packet.imu.accel_z_mps2;
        }
        if (packet.has_wheel) {
            result.snapshot.has_wheel = true;
            result.snapshot.wheel.timestamp_s =
                packet.wheel.timing.estimated_sample_time_s;
            result.snapshot.wheel.valid = packet.wheel.valid;
            result.snapshot.wheel.linear_mps =
                packet.wheel.linear_velocity_mps;
            result.snapshot.wheel.angular_rad_s =
                packet.wheel.angular_velocity_rad_s;
        }

        if (build_options_.populate_legacy_tof) {
            TofScanFrame legacy;
            if (!select_legacy_primary(packet, sync, &legacy)) {
                if (build_options_.require_legacy_primary) {
                    auto rejected = reject(
                        MultiTofSnapshotBuildFault::LegacyPrimaryMissing,
                        "multi_tof_snapshot_legacy_primary_missing");
                    rejected.sync_result = sync;
                    return rejected;
                }
            } else {
                result.snapshot.has_tof = true;
                result.snapshot.tof = legacy;
            }
        }

        result.ok = true;
        result.status = sync.degraded ? MultiTofSnapshotBuildStatus::BuiltDegraded
                                      : MultiTofSnapshotBuildStatus::Built;
        result.fault = MultiTofSnapshotBuildFault::None;
        result.passed.push_back("multi_tof_snapshot_build_ok");
        result.message = sync.degraded ? "multi_tof_snapshot_built_degraded"
                                       : "multi_tof_snapshot_built";
        return result;
    }

private:
    MultiTofContractOptions contract_options_;
    MultiTofSyncOptions sync_options_;
    MultiTofSnapshotBuildOptions build_options_;
    MultiTofSyncChecker sync_checker_;

    TofScanFrame convert_frame(
        const MultiTofRawFrame &raw,
        double timestamp_s) const {
        TofScanFrame frame;
        frame.timestamp_s = timestamp_s;
        frame.ranges_m = raw.ranges_m;
        frame.angle_min_rad = raw.angle_min_rad;
        frame.angle_increment_rad = raw.angle_increment_rad;
        frame.max_range_m = raw.range_max_m;
        frame.frame_id = raw.frame_id;
        return frame;
    }

    bool select_legacy_primary(
        const MultiTofRawPacket &packet,
        const MultiTofSyncResult &sync,
        TofScanFrame *out) const {
        if (out == nullptr) {
            return false;
        }
        if (build_options_.legacy_primary_mode ==
            MultiTofLegacyPrimaryMode::Front) {
            if (!packet.has_front) {
                return false;
            }
            *out = convert_frame(packet.front,
                                 packet.front.timing.estimated_sample_time_s);
            return true;
        }
        if (build_options_.legacy_primary_mode ==
            MultiTofLegacyPrimaryMode::FirstPresent) {
            if (packet.has_front) {
                *out = convert_frame(packet.front,
                                     packet.front.timing.estimated_sample_time_s);
                return true;
            }
            if (packet.has_left) {
                *out = convert_frame(packet.left,
                                     packet.left.timing.estimated_sample_time_s);
                return true;
            }
            if (packet.has_right) {
                *out = convert_frame(packet.right,
                                     packet.right.timing.estimated_sample_time_s);
                return true;
            }
            return false;
        }

        double best_dt = std::numeric_limits<double>::infinity();
        bool found = false;
        auto consider = [&](const MultiTofRawFrame &raw) {
            const double timestamp_s = raw.timing.estimated_sample_time_s;
            const double dt = std::fabs(timestamp_s - sync.times.multi_tof_time_s);
            if (!found || dt < best_dt) {
                *out = convert_frame(raw, timestamp_s);
                best_dt = dt;
                found = true;
            }
        };
        if (packet.has_front) {
            consider(packet.front);
        }
        if (packet.has_left) {
            consider(packet.left);
        }
        if (packet.has_right) {
            consider(packet.right);
        }
        return found;
    }

    MultiTofSnapshotBuildResult reject(
        MultiTofSnapshotBuildFault fault,
        const std::string &message) const {
        MultiTofSnapshotBuildResult result;
        result.ok = false;
        result.status = MultiTofSnapshotBuildStatus::Rejected;
        result.fault = fault;
        result.failed.push_back(message);
        result.message = message;
        return result;
    }
};

} // namespace robot_slamd
