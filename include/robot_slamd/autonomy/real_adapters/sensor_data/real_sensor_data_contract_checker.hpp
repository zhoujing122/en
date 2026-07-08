#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_types.hpp"

#include <cmath>
#include <utility>

namespace robot_slamd {

class RealSensorDataContractChecker {
public:
    RealSensorDataContractChecker() = default;

    explicit RealSensorDataContractChecker(
        RealSensorDataContractOptions options)
        : options_(std::move(options)) {}

    RealSensorDataContractResult check_packet(
        const RealSensorRawPacket &packet,
        double now_s) const {
        RealSensorDataContractResult result;

        // Packet timestamp and required sensor presence.
        if (!std::isfinite(packet.packet_timestamp_s)) {
            return reject(RealSensorDataFault::NonFiniteTimestamp,
                          "real_sensor_packet_non_finite_timestamp");
        }
        if (options_.reject_future_sensor_time &&
            packet.packet_timestamp_s - now_s >
                options_.max_future_timestamp_skew_s) {
            return reject(RealSensorDataFault::FutureTimestamp,
                          "real_sensor_packet_timestamp_in_future");
        }
        if (now_s - packet.packet_timestamp_s > options_.max_packet_age_s) {
            return reject(RealSensorDataFault::StaleTimestamp,
                          "real_sensor_packet_stale");
        }
        if (options_.require_tof && !packet.has_tof) {
            return reject(RealSensorDataFault::MissingTof,
                          "real_sensor_packet_missing_tof");
        }
        if (options_.require_imu_or_wheel && !packet.has_imu &&
            !packet.has_wheel) {
            return reject(RealSensorDataFault::MissingImuAndWheel,
                          "real_sensor_packet_missing_imu_and_wheel");
        }

        // Per-sensor contracts.
        if (packet.has_tof) {
            auto tof = check_tof_frame(packet.tof, now_s);
            append(result, tof);
            if (!tof.ok) {
                return result;
            }
        }
        if (packet.has_imu) {
            auto imu = check_imu_sample(packet.imu, now_s);
            append(result, imu);
            if (!imu.ok) {
                return result;
            }
        }
        if (packet.has_wheel) {
            auto wheel = check_wheel_sample(packet.wheel, now_s);
            append(result, wheel);
            if (!wheel.ok) {
                return result;
            }
        }

        // Packet time must stay close to each sensor effective time.
        if (packet.has_tof) {
            const double tof_time =
                real_sensor_effective_time_s(packet.tof.timing);
            if (std::fabs(packet.packet_timestamp_s - tof_time) >
                options_.max_packet_sensor_time_dt_s) {
                return reject(RealSensorDataFault::PacketSensorTimeMismatch,
                              "real_sensor_packet_tof_time_mismatch");
            }
        }
        if (packet.has_imu) {
            if (std::fabs(packet.packet_timestamp_s - packet.imu.timestamp_s) >
                options_.max_packet_sensor_time_dt_s) {
                return reject(RealSensorDataFault::PacketSensorTimeMismatch,
                              "real_sensor_packet_imu_time_mismatch");
            }
        }
        if (packet.has_wheel) {
            const double wheel_time =
                real_sensor_effective_time_s(packet.wheel.timing);
            if (std::fabs(packet.packet_timestamp_s - wheel_time) >
                options_.max_packet_sensor_time_dt_s) {
                return reject(RealSensorDataFault::PacketSensorTimeMismatch,
                              "real_sensor_packet_wheel_time_mismatch");
            }
        }

        // Cross-sensor sync uses request-window estimates for ToF and wheel.
        if (packet.has_tof && packet.has_imu) {
            const double tof_time =
                real_sensor_effective_time_s(packet.tof.timing);
            const double dt = std::fabs(tof_time - packet.imu.timestamp_s);
            if (dt > options_.max_sensor_sync_dt_s) {
                return reject(RealSensorDataFault::SensorSyncTooLarge,
                              "real_sensor_tof_imu_sync_too_large");
            }
        }
        if (packet.has_tof && packet.has_wheel) {
            const double tof_time =
                real_sensor_effective_time_s(packet.tof.timing);
            const double wheel_time =
                real_sensor_effective_time_s(packet.wheel.timing);
            const double dt = std::fabs(tof_time - wheel_time);
            if (dt > options_.max_sensor_sync_dt_s) {
                return reject(RealSensorDataFault::SensorSyncTooLarge,
                              "real_sensor_tof_wheel_sync_too_large");
            }
        }

        result.ok = true;
        result.status = result.warnings.empty()
                            ? RealSensorDataContractStatus::Accepted
                            : RealSensorDataContractStatus::Warning;
        result.fault = RealSensorDataFault::None;
        result.passed.push_back("real_sensor_packet_contract_ok");
        result.message = "real_sensor_packet_contract_ok";
        return result;
    }

    RealSensorDataContractResult check_tof_frame(
        const RealSensorRawTofFrame &tof,
        double now_s) const {
        RealSensorDataContractResult result;

        // Request-window timing.
        auto timing = check_request_timing(tof.timing, now_s, "real_sensor_tof");
        append(result, timing);
        if (!timing.ok) {
            return result;
        }

        // Frame and angular geometry.
        if (options_.require_frame_id && tof.frame_id.empty()) {
            return reject(RealSensorDataFault::EmptyFrameId,
                          "real_sensor_tof_empty_frame_id");
        }
        if (tof.ranges_m.empty()) {
            return reject(RealSensorDataFault::EmptyTofRanges,
                          "real_sensor_tof_empty_ranges");
        }
        if (!std::isfinite(tof.angle_increment_rad) ||
            tof.angle_increment_rad <= 0.0) {
            return reject(RealSensorDataFault::InvalidTofAngle,
                          "real_sensor_tof_invalid_angle_increment");
        }
        if (!std::isfinite(tof.angle_min_rad) ||
            !std::isfinite(tof.angle_max_rad)) {
            return reject(RealSensorDataFault::InvalidTofAngle,
                          "real_sensor_tof_invalid_angle_bounds");
        }

        // Range limits.
        if (!std::isfinite(tof.range_min_m) || tof.range_min_m <= 0.0) {
            return reject(RealSensorDataFault::InvalidTofRangeLimits,
                          "real_sensor_tof_invalid_range_min");
        }
        if (!std::isfinite(tof.range_max_m) ||
            tof.range_max_m <= tof.range_min_m) {
            return reject(RealSensorDataFault::InvalidTofRangeLimits,
                          "real_sensor_tof_invalid_range_max");
        }
        if (tof.range_min_m < options_.min_tof_range_m) {
            result.warnings.push_back("real_sensor_tof_range_min_below_option");
        }
        if (tof.range_max_m > options_.max_tof_range_m) {
            result.warnings.push_back("real_sensor_tof_range_max_above_option");
        }

        // Range samples.
        int non_finite_count = 0;
        for (double range_m : tof.ranges_m) {
            if (!std::isfinite(range_m)) {
                non_finite_count++;
            }
        }
        if (non_finite_count > 0 && !options_.allow_nan_ranges) {
            return reject(RealSensorDataFault::TofNanRatioTooHigh,
                          "real_sensor_tof_nan_not_allowed");
        }
        const double nan_ratio =
            static_cast<double>(non_finite_count) /
            static_cast<double>(tof.ranges_m.size());
        if (nan_ratio > options_.max_tof_nan_ratio) {
            return reject(RealSensorDataFault::TofNanRatioTooHigh,
                          "real_sensor_tof_nan_ratio_too_high");
        }

        result.ok = true;
        result.status = result.warnings.empty()
                            ? RealSensorDataContractStatus::Accepted
                            : RealSensorDataContractStatus::Warning;
        result.fault = RealSensorDataFault::None;
        result.passed.push_back("real_sensor_tof_contract_ok");
        result.message = "real_sensor_tof_contract_ok";
        return result;
    }

    RealSensorDataContractResult check_imu_sample(
        const RealSensorRawImuSample &imu,
        double now_s) const {
        // Timestamp and frame.
        if (!std::isfinite(imu.timestamp_s)) {
            return reject(RealSensorDataFault::InvalidImuTimestamp,
                          "real_sensor_imu_non_finite_timestamp");
        }
        if (options_.reject_future_sensor_time &&
            imu.timestamp_s - now_s > options_.max_future_timestamp_skew_s) {
            return reject(RealSensorDataFault::FutureTimestamp,
                          "real_sensor_imu_timestamp_in_future");
        }
        if (now_s - imu.timestamp_s > options_.max_packet_age_s) {
            return reject(RealSensorDataFault::StaleTimestamp,
                          "real_sensor_imu_stale");
        }
        if (options_.require_frame_id && imu.frame_id.empty()) {
            return reject(RealSensorDataFault::EmptyFrameId,
                          "real_sensor_imu_empty_frame_id");
        }

        // Yaw-rate and acceleration bounds.
        if (!std::isfinite(imu.yaw_rate_rad_s)) {
            return reject(RealSensorDataFault::InvalidImuYawRate,
                          "real_sensor_imu_non_finite_yaw_rate");
        }
        if (std::fabs(imu.yaw_rate_rad_s) >
            options_.max_abs_yaw_rate_rad_s) {
            return reject(RealSensorDataFault::InvalidImuYawRate,
                          "real_sensor_imu_yaw_rate_out_of_range");
        }
        if (!std::isfinite(imu.accel_x_mps2) ||
            !std::isfinite(imu.accel_y_mps2) ||
            !std::isfinite(imu.accel_z_mps2)) {
            return reject(RealSensorDataFault::InvalidImuAcceleration,
                          "real_sensor_imu_non_finite_acceleration");
        }
        const double norm = std::sqrt(imu.accel_x_mps2 * imu.accel_x_mps2 +
                                      imu.accel_y_mps2 * imu.accel_y_mps2 +
                                      imu.accel_z_mps2 * imu.accel_z_mps2);
        if (norm > options_.max_accel_norm_mps2) {
            return reject(RealSensorDataFault::InvalidImuAcceleration,
                          "real_sensor_imu_acceleration_out_of_range");
        }

        auto result = accepted("real_sensor_imu_contract_ok");
        return result;
    }

    RealSensorDataContractResult check_wheel_sample(
        const RealSensorRawWheelSample &wheel,
        double now_s) const {
        RealSensorDataContractResult result;

        // Request-window timing.
        auto timing =
            check_request_timing(wheel.timing, now_s, "real_sensor_wheel");
        append(result, timing);
        if (!timing.ok) {
            return result;
        }

        // Frame, validity, and velocity bounds.
        if (options_.require_frame_id && wheel.frame_id.empty()) {
            return reject(RealSensorDataFault::EmptyFrameId,
                          "real_sensor_wheel_empty_frame_id");
        }
        if (!wheel.valid) {
            return reject(RealSensorDataFault::InvalidWheelFlag,
                          "real_sensor_wheel_invalid_flag");
        }
        if (!std::isfinite(wheel.linear_velocity_mps)) {
            return reject(RealSensorDataFault::InvalidWheelVelocity,
                          "real_sensor_wheel_non_finite_linear_velocity");
        }
        if (!std::isfinite(wheel.angular_velocity_rad_s)) {
            return reject(RealSensorDataFault::InvalidWheelVelocity,
                          "real_sensor_wheel_non_finite_angular_velocity");
        }
        if (std::fabs(wheel.linear_velocity_mps) >
            options_.max_abs_wheel_linear_mps) {
            return reject(RealSensorDataFault::InvalidWheelVelocity,
                          "real_sensor_wheel_linear_velocity_out_of_range");
        }
        if (std::fabs(wheel.angular_velocity_rad_s) >
            options_.max_abs_wheel_angular_rad_s) {
            return reject(RealSensorDataFault::InvalidWheelVelocity,
                          "real_sensor_wheel_angular_velocity_out_of_range");
        }

        result.ok = true;
        result.status = result.warnings.empty()
                            ? RealSensorDataContractStatus::Accepted
                            : RealSensorDataContractStatus::Warning;
        result.fault = RealSensorDataFault::None;
        result.passed.push_back("real_sensor_wheel_contract_ok");
        result.message = "real_sensor_wheel_contract_ok";
        return result;
    }

    RealSensorDataContractResult check_request_timing(
        const RealSensorRequestTiming &timing,
        double now_s,
        const std::string &label) const {
        RealSensorDataContractResult result;
        if (!options_.require_request_timing) {
            return accepted(label + "_request_timing_not_required");
        }

        // Request-window finite and monotonic checks.
        if (!std::isfinite(timing.request_start_s)) {
            return reject(RealSensorDataFault::InvalidRequestTiming,
                          label + "_request_start_non_finite");
        }
        if (!std::isfinite(timing.response_received_s)) {
            return reject(RealSensorDataFault::InvalidRequestTiming,
                          label + "_response_received_non_finite");
        }
        if (timing.response_received_s < timing.request_start_s) {
            return reject(RealSensorDataFault::InvalidRequestTiming,
                          label + "_response_before_request");
        }
        if (!std::isfinite(timing.estimated_sample_time_s)) {
            return reject(RealSensorDataFault::InvalidRequestTiming,
                          label + "_estimated_sample_time_non_finite");
        }
        if (!std::isfinite(timing.request_latency_s)) {
            return reject(RealSensorDataFault::InvalidRequestTiming,
                          label + "_request_latency_non_finite");
        }

        // Latency must match the request window exactly enough for replay.
        if (timing.request_latency_s < 0.0) {
            return reject(RealSensorDataFault::InvalidRequestTiming,
                          label + "_request_latency_negative");
        }
        const double expected_latency =
            timing.response_received_s - timing.request_start_s;
        if (std::fabs(timing.request_latency_s - expected_latency) >
            options_.max_request_latency_mismatch_s) {
            if (options_.reject_request_latency_mismatch) {
                return reject(RealSensorDataFault::RequestLatencyMismatch,
                              label + "_request_latency_mismatch");
            }
            result.warnings.push_back(label + "_request_latency_mismatch");
        }
        if (timing.request_latency_s > options_.max_request_latency_s) {
            return reject(RealSensorDataFault::RequestLatencyTooHigh,
                          label + "_request_latency_too_high");
        }

        // Estimated sample time is a request-window midpoint estimate.
        if (options_.require_estimated_sample_time_in_window &&
            (timing.estimated_sample_time_s < timing.request_start_s ||
             timing.estimated_sample_time_s > timing.response_received_s)) {
            return reject(
                RealSensorDataFault::EstimatedSampleTimeOutsideWindow,
                label + "_estimated_sample_time_outside_request_window");
        }
        const double midpoint =
            0.5 * (timing.request_start_s + timing.response_received_s);
        if (options_.require_estimated_sample_time_midpoint &&
            std::fabs(timing.estimated_sample_time_s - midpoint) >
                options_.max_estimated_sample_time_midpoint_error_s) {
            return reject(
                RealSensorDataFault::EstimatedSampleTimeMidpointMismatch,
                label + "_estimated_sample_time_not_midpoint");
        }
        if (options_.reject_future_sensor_time &&
            timing.estimated_sample_time_s - now_s >
                options_.max_future_timestamp_skew_s) {
            return reject(RealSensorDataFault::FutureTimestamp,
                          label + "_estimated_sample_time_in_future");
        }
        if (now_s - timing.estimated_sample_time_s > options_.max_packet_age_s) {
            return reject(RealSensorDataFault::StaleTimestamp,
                          label + "_estimated_sample_time_stale");
        }

        result.ok = true;
        result.status = result.warnings.empty()
                            ? RealSensorDataContractStatus::Accepted
                            : RealSensorDataContractStatus::Warning;
        result.fault = RealSensorDataFault::None;
        result.passed.push_back(label + "_request_timing_ok");
        result.message = label + "_request_timing_ok";
        return result;
    }

private:
    static RealSensorDataContractResult accepted(const std::string &message) {
        RealSensorDataContractResult result;
        result.ok = true;
        result.status = RealSensorDataContractStatus::Accepted;
        result.fault = RealSensorDataFault::None;
        result.passed.push_back(message);
        result.message = message;
        return result;
    }

    static RealSensorDataContractResult reject(
        RealSensorDataFault fault,
        const std::string &message) {
        RealSensorDataContractResult result;
        result.ok = false;
        result.status = RealSensorDataContractStatus::Rejected;
        result.fault = fault;
        result.failed.push_back(message);
        result.message = message;
        return result;
    }

    static void append(RealSensorDataContractResult &dst,
                       const RealSensorDataContractResult &src) {
        dst.passed.insert(dst.passed.end(), src.passed.begin(), src.passed.end());
        dst.failed.insert(dst.failed.end(), src.failed.begin(), src.failed.end());
        dst.warnings.insert(dst.warnings.end(),
                            src.warnings.begin(),
                            src.warnings.end());
        if (!src.ok) {
            dst.ok = false;
            dst.status = src.status;
            dst.fault = src.fault;
            dst.message = src.message;
        }
    }

    RealSensorDataContractOptions options_;
};

} // namespace robot_slamd
