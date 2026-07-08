#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_types.hpp"

#include <cmath>
#include <string>

namespace robot_slamd {

class SlamBackendContractChecker {
public:
    SlamBackendContractChecker() = default;

    explicit SlamBackendContractChecker(SlamBackendContractOptions options)
        : options_(options) {}

    SlamBackendUpdateResult check_input_frame(const SlamBackendInputFrame &input,
                                              double now_s) const {
        // Timestamp and freshness validation.
        if (!std::isfinite(input.timestamp_s)) {
            return make_error(SlamBackendFault::InvalidTimestamp,
                              "slam_input_non_finite_timestamp");
        }
        if (now_s - input.timestamp_s > options_.max_input_age_s) {
            return make_error(SlamBackendFault::InvalidTimestamp,
                              "slam_input_stale");
        }

        // Required sensor validation.
        if (options_.require_tof && !input.snapshot.has_tof) {
            return make_error(SlamBackendFault::MissingTof,
                              "slam_input_missing_tof");
        }
        if (options_.require_imu_or_wheel &&
            !input.snapshot.has_imu &&
            !input.snapshot.has_wheel) {
            return make_error(SlamBackendFault::MissingMotionReference,
                              "slam_input_missing_imu_or_wheel");
        }

        // ToF scan validation.
        if (input.snapshot.has_tof && input.snapshot.tof.ranges_m.empty()) {
            return make_error(SlamBackendFault::InvalidTofScan,
                              "slam_input_empty_tof_ranges");
        }
        if (input.snapshot.has_tof &&
            (!std::isfinite(input.snapshot.tof.angle_increment_rad) ||
             input.snapshot.tof.angle_increment_rad <= 0.0)) {
            return make_error(SlamBackendFault::InvalidTofScan,
                              "slam_input_invalid_tof_angle_increment");
        }

        // Motion reference validation.
        if (input.snapshot.has_imu && !std::isfinite(input.snapshot.imu.timestamp_s)) {
            return make_error(SlamBackendFault::InvalidImu,
                              "slam_input_invalid_imu_timestamp");
        }
        if (input.snapshot.has_wheel && !input.snapshot.wheel.valid) {
            return make_error(SlamBackendFault::InvalidWheel,
                              "slam_input_invalid_wheel_flag");
        }
        if (!options_.allow_predicted_pose_missing && !input.has_predicted_pose) {
            return make_error(SlamBackendFault::UpdateRejected,
                              "slam_input_missing_predicted_pose");
        }

        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Accepted;
        result.fault = SlamBackendFault::None;
        result.message = "slam_input_ok";
        return result;
    }

    SlamBackendUpdateResult check_update_result(
        const SlamBackendUpdateResult &result) const {
        // Latency and scan count validation.
        if (!std::isfinite(result.update_latency_s)) {
            return make_fault("slam_update_non_finite_latency");
        }
        if (result.update_latency_s > options_.max_update_latency_s) {
            return make_fault("slam_update_latency_too_high");
        }
        if (result.integrated_scan_count < 0) {
            return make_fault("slam_update_negative_scan_count");
        }
        if (result.integrated_scan_count < options_.min_integrated_scan_count_for_quality) {
            return make_error(SlamBackendFault::UpdateRejected,
                              "slam_update_scan_count_too_low");
        }

        // Accepted update semantics.
        if (result.status == SlamBackendUpdateStatus::Accepted && !result.map_updated) {
            return make_error(SlamBackendFault::UpdateRejected,
                              "slam_update_accepted_without_map_update");
        }
        if (result.status != SlamBackendUpdateStatus::Accepted) {
            SlamBackendUpdateResult rejected = result;
            if (rejected.message.empty()) {
                rejected.message = "slam_update_not_accepted";
            }
            return rejected;
        }

        // Quality validation.
        auto quality_result = check_quality(result.quality);
        if (quality_result.status != SlamBackendUpdateStatus::Accepted) {
            return quality_result;
        }
        return result;
    }

    SlamBackendUpdateResult check_quality(const RobotSlamMapQuality &quality) const {
        if (!std::isfinite(quality.coverage_ratio)) {
            return make_error(SlamBackendFault::QualityInvalid,
                              "slam_quality_non_finite_coverage");
        }
        if (!std::isfinite(quality.yaw_coverage_ratio)) {
            return make_error(SlamBackendFault::QualityInvalid,
                              "slam_quality_non_finite_yaw_coverage");
        }
        if (quality.coverage_ratio < 0.0 || quality.coverage_ratio > 1.0) {
            return make_error(SlamBackendFault::QualityInvalid,
                              "slam_quality_coverage_out_of_range");
        }
        if (quality.yaw_coverage_ratio < 0.0 || quality.yaw_coverage_ratio > 1.0) {
            return make_error(SlamBackendFault::QualityInvalid,
                              "slam_quality_yaw_coverage_out_of_range");
        }
        if (quality.valid_scan_count < 0) {
            return make_error(SlamBackendFault::QualityInvalid,
                              "slam_quality_negative_scan_count");
        }
        if (!quality.good_enough && quality.reason.empty()) {
            return make_error(SlamBackendFault::QualityInvalid,
                              "slam_quality_poor_reason_empty");
        }

        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Accepted;
        result.fault = SlamBackendFault::None;
        result.quality = quality;
        result.message = "slam_quality_ok";
        return result;
    }

private:
    static SlamBackendUpdateResult make_error(SlamBackendFault fault,
                                              const std::string &message) {
        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Rejected;
        result.fault = fault;
        result.message = message;
        return result;
    }

    static SlamBackendUpdateResult make_fault(const std::string &message) {
        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Fault;
        result.fault = SlamBackendFault::UpdateRejected;
        result.message = message;
        return result;
    }

    SlamBackendContractOptions options_;
};

} // namespace robot_slamd
