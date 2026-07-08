#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_contract_types.hpp"

#include <cmath>
#include <string>

namespace robot_slamd {

class RealAdapterContractChecker {
public:
    RealAdapterContractChecker() = default;

    explicit RealAdapterContractChecker(RealAdapterContractOptions options)
        : options_(options) {}

    RealAdapterContractReport check_tof_frame(const TofScanFrame &frame,
                                              double now_s) const {
        RealAdapterContractReport report;

        // timestamp / age validation
        if (!std::isfinite(frame.timestamp_s)) {
            add_error(report, RealAdapterKind::Tof, "tof_non_finite_timestamp");
            return finalize(report);
        }
        if (std::isfinite(now_s) && now_s - frame.timestamp_s > options_.tof.max_frame_age_s) {
            add_error(report, RealAdapterKind::Tof, "tof_frame_stale");
        }

        // shape validation
        const auto range_count = static_cast<int>(frame.ranges_m.size());
        if (range_count < options_.tof.min_range_count) {
            add_error(report, RealAdapterKind::Tof, "tof_range_count_too_low");
        }
        if (range_count > options_.tof.max_range_count) {
            add_error(report, RealAdapterKind::Tof, "tof_range_count_too_high");
        }
        if (!std::isfinite(frame.angle_increment_rad) || frame.angle_increment_rad <= 0.0) {
            add_error(report, RealAdapterKind::Tof, "tof_invalid_angle_increment");
        }
        if (!std::isfinite(frame.max_range_m) || frame.max_range_m <= 0.0) {
            add_error(report, RealAdapterKind::Tof, "tof_invalid_max_range");
        }
        if (options_.tof.require_frame_id && frame.frame_id.empty()) {
            add_error(report, RealAdapterKind::Tof, "tof_frame_id_required");
        }

        // range validation
        int nan_count = 0;
        for (double range : frame.ranges_m) {
            if (std::isnan(range)) {
                nan_count++;
                continue;
            }
            if (!std::isfinite(range)) {
                nan_count++;
                continue;
            }
            if (range < options_.tof.min_range_m) {
                add_error(report, RealAdapterKind::Tof, "tof_range_below_min");
                break;
            }
            if (range > options_.tof.max_range_m) {
                add_error(report, RealAdapterKind::Tof, "tof_range_above_max");
                break;
            }
        }
        if (range_count > 0) {
            const double nan_ratio = static_cast<double>(nan_count) /
                                     static_cast<double>(range_count);
            if (nan_ratio > options_.tof.max_allowed_nan_ratio) {
                add_error(report, RealAdapterKind::Tof, "tof_nan_ratio_too_high");
            }
        }

        return finalize(report);
    }

    RealAdapterContractReport check_imu_frame(const ImuFrame &frame,
                                              double now_s) const {
        RealAdapterContractReport report;

        // timestamp / age validation
        if (!std::isfinite(frame.timestamp_s)) {
            add_error(report, RealAdapterKind::Imu, "imu_non_finite_timestamp");
            return finalize(report);
        }
        if (std::isfinite(now_s) && now_s - frame.timestamp_s > options_.imu.max_frame_age_s) {
            add_error(report, RealAdapterKind::Imu, "imu_frame_stale");
        }

        // motion validation
        if (!std::isfinite(frame.yaw_rate_rad_s)) {
            add_error(report, RealAdapterKind::Imu, "imu_non_finite_yaw_rate");
        } else if (std::fabs(frame.yaw_rate_rad_s) > options_.imu.max_abs_yaw_rate_rad_s) {
            add_error(report, RealAdapterKind::Imu, "imu_yaw_rate_out_of_range");
        }

        const bool acc_finite = std::isfinite(frame.ax_mps2) &&
                                std::isfinite(frame.ay_mps2) &&
                                std::isfinite(frame.az_mps2);
        if (!acc_finite) {
            add_error(report, RealAdapterKind::Imu, "imu_non_finite_acc");
        } else if (std::fabs(frame.ax_mps2) > options_.imu.max_abs_acc_mps2 ||
                   std::fabs(frame.ay_mps2) > options_.imu.max_abs_acc_mps2 ||
                   std::fabs(frame.az_mps2) > options_.imu.max_abs_acc_mps2) {
            add_error(report, RealAdapterKind::Imu, "imu_acc_out_of_range");
        }

        return finalize(report);
    }

    RealAdapterContractReport check_wheel_frame(const WheelOdomFrame &frame,
                                                double now_s) const {
        RealAdapterContractReport report;

        // validity / timestamp validation
        if (!frame.valid) {
            add_error(report, RealAdapterKind::Wheel, "wheel_invalid_flag");
        }
        if (!std::isfinite(frame.timestamp_s)) {
            add_error(report, RealAdapterKind::Wheel, "wheel_non_finite_timestamp");
            return finalize(report);
        }
        if (std::isfinite(now_s) && now_s - frame.timestamp_s > options_.wheel.max_frame_age_s) {
            add_error(report, RealAdapterKind::Wheel, "wheel_frame_stale");
        }

        // motion validation
        if (!std::isfinite(frame.linear_mps) || !std::isfinite(frame.angular_rad_s)) {
            add_error(report, RealAdapterKind::Wheel, "wheel_non_finite_motion");
        } else {
            if (std::fabs(frame.linear_mps) > options_.wheel.max_abs_linear_mps) {
                add_error(report, RealAdapterKind::Wheel, "wheel_linear_out_of_range");
            }
            if (std::fabs(frame.angular_rad_s) > options_.wheel.max_abs_angular_rad_s) {
                add_error(report, RealAdapterKind::Wheel, "wheel_angular_out_of_range");
            }
        }

        return finalize(report);
    }

    RealAdapterContractReport check_sensor_snapshot(const RobotSlamSensorSnapshot &snapshot,
                                                    double now_s) const {
        RealAdapterContractReport report;

        // required sensor presence
        if (options_.require_tof && !snapshot.has_tof) {
            add_error(report, RealAdapterKind::SensorFusion, "snapshot_missing_tof");
        }
        if (options_.require_imu_or_wheel && !snapshot.has_imu && !snapshot.has_wheel) {
            add_error(report, RealAdapterKind::SensorFusion, "snapshot_missing_imu_or_wheel");
        }

        // nested frame contracts
        if (snapshot.has_tof) {
            merge(report, check_tof_frame(snapshot.tof, now_s));
        }
        if (snapshot.has_imu) {
            merge(report, check_imu_frame(snapshot.imu, now_s));
        }
        if (snapshot.has_wheel) {
            merge(report, check_wheel_frame(snapshot.wheel, now_s));
        }

        return finalize(report);
    }

    RealAdapterContractReport check_map_quality(const RobotSlamMapQuality &quality) const {
        RealAdapterContractReport report;

        // numeric validation
        if (!std::isfinite(quality.coverage_ratio)) {
            add_error(report, RealAdapterKind::Map, "map_non_finite_coverage");
        } else if (quality.coverage_ratio < options_.map.min_coverage_ratio ||
                   quality.coverage_ratio > options_.map.max_coverage_ratio) {
            add_error(report, RealAdapterKind::Map, "map_coverage_out_of_range");
        }
        if (!std::isfinite(quality.yaw_coverage_ratio)) {
            add_error(report, RealAdapterKind::Map, "map_non_finite_yaw_coverage");
        } else if (quality.yaw_coverage_ratio < options_.map.min_yaw_coverage_ratio ||
                   quality.yaw_coverage_ratio > options_.map.max_yaw_coverage_ratio) {
            add_error(report, RealAdapterKind::Map, "map_yaw_coverage_out_of_range");
        }
        if (quality.valid_scan_count < 0) {
            add_error(report, RealAdapterKind::Map, "map_negative_scan_count");
        }

        // reason validation
        if (!quality.good_enough && quality.reason.empty()) {
            add_warning(report, "map_poor_quality_reason_empty");
        }

        return finalize(report);
    }

private:
    static void add_error(RealAdapterContractReport &report,
                          RealAdapterKind kind,
                          const std::string &code) {
        RealAdapterContractViolation violation;
        violation.kind = kind;
        violation.severity = RealAdapterSeverity::Error;
        violation.code = code;
        violation.message = code;
        report.violations.push_back(violation);
    }

    static void add_warning(RealAdapterContractReport &report,
                            const std::string &code) {
        report.warnings.push_back(code);
    }

    static void merge(RealAdapterContractReport &target,
                      const RealAdapterContractReport &source) {
        target.violations.insert(target.violations.end(),
                                 source.violations.begin(),
                                 source.violations.end());
        target.warnings.insert(target.warnings.end(),
                               source.warnings.begin(),
                               source.warnings.end());
    }

    static RealAdapterContractReport finalize(RealAdapterContractReport report) {
        report.ok = report.violations.empty();
        report.readiness = report.ok ? RealAdapterReadiness::ShadowReady
                                     : RealAdapterReadiness::NotReady;
        return report;
    }

    RealAdapterContractOptions options_;
};

} // namespace robot_slamd
