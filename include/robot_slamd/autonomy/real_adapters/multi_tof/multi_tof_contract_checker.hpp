#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_types.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

class MultiTofContractChecker {
public:
    explicit MultiTofContractChecker(
        MultiTofContractOptions options = {})
        : options_(options) {}

    MultiTofContractResult check_frame(
        const MultiTofRawFrame &frame,
        double now_s,
        const std::string &label) const {
        if (frame.frame_id.empty()) {
            return reject(MultiTofContractFault::EmptyFrameId, label + "_empty_frame_id");
        }
        if (std::fabs(frame.mount_yaw_rad - expected_mount_yaw_rad(frame.mount_id, options_)) >
            options_.max_mount_yaw_error_rad) {
            return reject(MultiTofContractFault::InvalidMountYaw, label + "_invalid_mount_yaw");
        }
        const auto timing = check_request_timing(frame.timing, now_s, label);
        if (!timing.ok) {
            return timing;
        }
        if (frame.ranges_m.empty()) {
            return reject(MultiTofContractFault::EmptyRanges, label + "_empty_ranges");
        }
        if (!std::isfinite(frame.angle_min_rad) ||
            !std::isfinite(frame.angle_max_rad) ||
            !std::isfinite(frame.angle_increment_rad) ||
            frame.angle_increment_rad <= 0.0 ||
            frame.angle_max_rad < frame.angle_min_rad) {
            return reject(MultiTofContractFault::InvalidAngle, label + "_invalid_angle");
        }
        if (!std::isfinite(frame.range_min_m) ||
            !std::isfinite(frame.range_max_m) ||
            frame.range_min_m <= 0.0 ||
            frame.range_max_m <= frame.range_min_m) {
            return reject(MultiTofContractFault::InvalidRangeLimits, label + "_invalid_range_limits");
        }

        int nan_count = 0;
        int valid_count = 0;
        for (const double range_m : frame.ranges_m) {
            if (!std::isfinite(range_m)) {
                nan_count++;
                continue;
            }
            if (range_m >= frame.range_min_m &&
                range_m <= frame.range_max_m &&
                range_m >= options_.min_range_m &&
                range_m <= options_.max_range_m) {
                valid_count++;
            }
        }
        const double nan_ratio = static_cast<double>(nan_count) /
                                 static_cast<double>(frame.ranges_m.size());
        if ((!options_.allow_nan_ranges && nan_count > 0) || nan_ratio > options_.max_nan_ratio) {
            return reject(MultiTofContractFault::NanRatioTooHigh, label + "_nan_ratio_too_high");
        }
        if (valid_count <= 0) {
            return reject(MultiTofContractFault::EmptyRanges, label + "_no_valid_ranges");
        }

        MultiTofContractResult result;
        result.ok = true;
        result.status = MultiTofContractStatus::Accepted;
        result.fault = MultiTofContractFault::None;
        result.valid_tof_count = 1;
        result.passed.push_back("multi_tof_" + label + "_ok");
        result.passed.push_back("multi_tof_" + label + "_request_timing_ok");
        result.message = label + "_tof_frame_ok";
        return result;
    }

    MultiTofContractResult check_packet(
        const MultiTofRawPacket &packet,
        double now_s) const {
        if (!std::isfinite(packet.packet_timestamp_s) ||
            packet.packet_timestamp_s - now_s > options_.max_future_timestamp_skew_s) {
            return reject(MultiTofContractFault::InvalidPacketTimestamp,
                          "multi_tof_invalid_packet_timestamp");
        }

        const int present_count = count_present_tof(packet);
        if (present_count < options_.min_required_tof_count) {
            return reject(MultiTofContractFault::TooFewTofFrames,
                          "multi_tof_too_few_tof_frames");
        }
        if (options_.require_front && !packet.has_front) {
            return reject(MultiTofContractFault::MissingFrontTof,
                          "multi_tof_missing_front");
        }
        if (options_.require_left && !packet.has_left) {
            return reject(MultiTofContractFault::MissingLeftTof,
                          "multi_tof_missing_left");
        }
        if (options_.require_right && !packet.has_right) {
            return reject(MultiTofContractFault::MissingRightTof,
                          "multi_tof_missing_right");
        }

        std::vector<std::pair<std::string, MultiTofRawFrame>> frames;
        if (packet.has_front) {
            frames.push_back({"front", packet.front});
        }
        if (packet.has_left) {
            frames.push_back({"left", packet.left});
        }
        if (packet.has_right) {
            frames.push_back({"right", packet.right});
        }

        if (options_.require_unique_mount_ids) {
            std::set<int> mount_ids;
            for (const auto &entry : frames) {
                if (!mount_ids.insert(multi_tof_mount_id_id(entry.second.mount_id)).second) {
                    return reject(MultiTofContractFault::DuplicateMountId,
                                  "multi_tof_duplicate_mount_id");
                }
            }
        }
        if (options_.require_unique_frame_ids) {
            std::set<std::string> frame_ids;
            for (const auto &entry : frames) {
                if (!entry.second.frame_id.empty() &&
                    !frame_ids.insert(entry.second.frame_id).second) {
                    return reject(MultiTofContractFault::DuplicateFrameId,
                                  "multi_tof_duplicate_frame_id");
                }
            }
        }

        MultiTofContractResult result;
        result.ok = true;
        result.status = MultiTofContractStatus::Warning;
        result.fault = MultiTofContractFault::None;
        result.valid_tof_count = present_count;
        for (const auto &entry : frames) {
            const auto frame_result = check_frame(entry.second, now_s, entry.first);
            if (!frame_result.ok) {
                return frame_result;
            }
            result.passed.insert(result.passed.end(),
                                 frame_result.passed.begin(),
                                 frame_result.passed.end());
        }
        result.warnings.push_back("multi_tof_pairwise_sync_deferred_to_m3c1");
        result.warnings.push_back("multi_tof_imu_wheel_sync_deferred_to_m3c1");
        result.message = "multi_tof_packet_ok_sync_deferred";
        return result;
    }

private:
    MultiTofContractOptions options_;

    MultiTofContractResult check_request_timing(
        const RealSensorRequestTiming &timing,
        double now_s,
        const std::string &label) const {
        if (!options_.require_request_timing) {
            MultiTofContractResult result;
            result.ok = true;
            result.status = MultiTofContractStatus::Accepted;
            result.fault = MultiTofContractFault::None;
            result.message = label + "_request_timing_not_required";
            return result;
        }
        if (!std::isfinite(timing.request_start_s) ||
            !std::isfinite(timing.response_received_s) ||
            !std::isfinite(timing.estimated_sample_time_s) ||
            !std::isfinite(timing.request_latency_s) ||
            timing.response_received_s < timing.request_start_s ||
            timing.request_latency_s < 0.0) {
            return reject(MultiTofContractFault::InvalidRequestTiming,
                          label + "_invalid_request_timing");
        }
        const double expected_latency_s =
            timing.response_received_s - timing.request_start_s;
        if (std::fabs(expected_latency_s - timing.request_latency_s) >
            options_.max_request_latency_mismatch_s) {
            return reject(MultiTofContractFault::RequestLatencyMismatch,
                          label + "_request_latency_mismatch");
        }
        if (timing.request_latency_s > options_.max_request_latency_s) {
            return reject(MultiTofContractFault::RequestLatencyTooHigh,
                          label + "_request_latency_too_high");
        }
        if (timing.estimated_sample_time_s < timing.request_start_s ||
            timing.estimated_sample_time_s > timing.response_received_s) {
            return reject(MultiTofContractFault::EstimatedSampleTimeOutsideWindow,
                          label + "_estimated_sample_time_outside_window");
        }
        const double midpoint_s =
            0.5 * (timing.request_start_s + timing.response_received_s);
        if (std::fabs(timing.estimated_sample_time_s - midpoint_s) >
            options_.max_estimated_sample_time_midpoint_error_s) {
            return reject(MultiTofContractFault::EstimatedSampleTimeMidpointMismatch,
                          label + "_estimated_sample_time_midpoint_mismatch");
        }
        if (timing.estimated_sample_time_s - now_s > options_.max_future_timestamp_skew_s) {
            return reject(MultiTofContractFault::FutureTimestamp,
                          label + "_future_timestamp");
        }
        if (now_s - timing.estimated_sample_time_s > options_.max_packet_age_s) {
            return reject(MultiTofContractFault::InvalidRequestTiming,
                          label + "_stale_request_timing");
        }

        MultiTofContractResult result;
        result.ok = true;
        result.status = MultiTofContractStatus::Accepted;
        result.fault = MultiTofContractFault::None;
        result.message = label + "_request_timing_ok";
        return result;
    }

    MultiTofContractResult reject(
        MultiTofContractFault fault,
        const std::string &message) const {
        MultiTofContractResult result;
        result.ok = false;
        result.status = MultiTofContractStatus::Rejected;
        result.fault = fault;
        result.failed.push_back(message);
        result.message = message;
        return result;
    }
};

} // namespace robot_slamd
