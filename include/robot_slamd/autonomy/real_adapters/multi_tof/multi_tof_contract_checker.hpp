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

    ScalarTofValidatedReading validate_reading(
        const MultiTofRawFrame &frame,
        double now_s) const {
        ScalarTofValidatedReading reading;
        reading.raw = frame;
        reading.distance_m = static_cast<double>(frame.distance_mm) / 1000.0;
        reading.effective_timestamp_s = frame.timing.estimated_sample_time_s;

        const auto timing = check_request_timing(frame.timing, now_s, "scalar_tof");
        if (!timing.ok) {
            reading.fault = timing_to_scalar_fault(timing.fault);
            reading.reason = timing.message;
            return reading;
        }
        if (!std::isfinite(frame.full_fov_rad) || frame.full_fov_rad <= 0.0) {
            reading.fault = ScalarTofFault::InvalidFullFov;
            reading.reason = "invalid_full_fov";
            return reading;
        }
        if (frame.distance_mm < options_.protocol_min_distance_mm) {
            reading.fault = ScalarTofFault::DistanceBelowProtocolMinimum;
            reading.reason = "distance_below_protocol_minimum";
            return reading;
        }
        if (frame.distance_mm > options_.protocol_max_distance_mm) {
            reading.fault = ScalarTofFault::DistanceAboveProtocolMaximum;
            reading.reason = "distance_above_protocol_maximum";
            return reading;
        }
        if (frame.confidence > 100) {
            reading.fault = ScalarTofFault::ConfidenceOutOfRange;
            reading.reason = "confidence_out_of_range";
            return reading;
        }
        if (frame.confidence == 0) {
            reading.protocol_valid = false;
            reading.usable_for_mapping = false;
            reading.validity = ScalarTofValidity::Invalid;
            reading.fault = ScalarTofFault::ConfidenceZero;
            reading.reason = "confidence_zero";
            return reading;
        }

        reading.protocol_valid = true;
        if (frame.confidence < options_.mapping_min_confidence) {
            reading.validity = ScalarTofValidity::DiagnosticOnly;
            reading.fault = ScalarTofFault::ConfidenceBelowMappingThreshold;
            reading.reason = "low_confidence";
            return reading;
        }
        if (frame.distance_mm < options_.mapping_min_distance_mm ||
            frame.distance_mm > options_.mapping_max_distance_mm) {
            reading.validity = ScalarTofValidity::DiagnosticOnly;
            reading.fault = ScalarTofFault::DistanceOutsideMappingRange;
            reading.reason = "distance_outside_mapping_range";
            return reading;
        }

        reading.usable_for_mapping = true;
        reading.validity = ScalarTofValidity::ValidForMapping;
        reading.fault = ScalarTofFault::None;
        reading.reason = "ok";
        return reading;
    }

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
        const auto reading = validate_reading(frame, now_s);
        if (reading.fault == ScalarTofFault::InvalidRequestTiming ||
            reading.fault == ScalarTofFault::RequestLatencyTooHigh ||
            reading.fault == ScalarTofFault::FutureTimestamp ||
            reading.fault == ScalarTofFault::StaleTimestamp) {
            return reject(scalar_to_contract_fault(reading.fault), label + "_" + reading.reason);
        }
        if (!reading.protocol_valid) {
            return reject(scalar_to_contract_fault(reading.fault), label + "_" + reading.reason);
        }

        MultiTofContractResult result;
        result.ok = true;
        result.status = reading.usable_for_mapping ? MultiTofContractStatus::Accepted
                                                   : MultiTofContractStatus::Warning;
        result.fault = reading.usable_for_mapping ? MultiTofContractFault::None
                                                  : scalar_to_contract_fault(reading.fault);
        result.valid_tof_count = reading.usable_for_mapping ? 1 : 0;
        result.passed.push_back("multi_tof_" + label + "_protocol_ok");
        result.passed.push_back("multi_tof_" + label + "_request_timing_ok");
        if (reading.usable_for_mapping) {
            result.passed.push_back("multi_tof_" + label + "_mapping_usable");
            result.message = label + "_tof_frame_ok";
        } else {
            result.warnings.push_back("multi_tof_" + label + "_diagnostic_only_" + reading.reason);
            result.message = label + "_tof_frame_diagnostic_only";
        }
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
        if (packet.has_front) frames.push_back({"front", packet.front});
        if (packet.has_left) frames.push_back({"left", packet.left});
        if (packet.has_right) frames.push_back({"right", packet.right});

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
        result.status = MultiTofContractStatus::Accepted;
        result.fault = MultiTofContractFault::None;
        for (const auto &entry : frames) {
            const auto frame_result = check_frame(entry.second, now_s, entry.first);
            if (!frame_result.ok) {
                return frame_result;
            }
            result.valid_tof_count += frame_result.valid_tof_count;
            result.passed.insert(result.passed.end(), frame_result.passed.begin(), frame_result.passed.end());
            result.warnings.insert(result.warnings.end(), frame_result.warnings.begin(), frame_result.warnings.end());
            if (frame_result.status == MultiTofContractStatus::Warning) {
                result.status = MultiTofContractStatus::Warning;
            }
        }
        if (result.valid_tof_count < options_.min_required_tof_count) {
            return reject(MultiTofContractFault::TooFewTofFrames,
                          "multi_tof_too_few_mapping_usable_tof_frames");
        }
        result.message = result.status == MultiTofContractStatus::Warning
                             ? "multi_tof_packet_ok_with_diagnostic_only_frames"
                             : "multi_tof_packet_ok";
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
        const double midpoint_s = 0.5 * (timing.request_start_s + timing.response_received_s);
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
                          label + "_stale_timestamp");
        }

        MultiTofContractResult result;
        result.ok = true;
        result.status = MultiTofContractStatus::Accepted;
        result.fault = MultiTofContractFault::None;
        result.message = label + "_request_timing_ok";
        return result;
    }

    static ScalarTofFault timing_to_scalar_fault(MultiTofContractFault fault) {
        if (fault == MultiTofContractFault::RequestLatencyTooHigh) {
            return ScalarTofFault::RequestLatencyTooHigh;
        }
        if (fault == MultiTofContractFault::FutureTimestamp) {
            return ScalarTofFault::FutureTimestamp;
        }
        return ScalarTofFault::InvalidRequestTiming;
    }

    static MultiTofContractFault scalar_to_contract_fault(ScalarTofFault fault) {
        switch (fault) {
        case ScalarTofFault::DistanceBelowProtocolMinimum:
            return MultiTofContractFault::DistanceBelowProtocolMinimum;
        case ScalarTofFault::DistanceAboveProtocolMaximum:
            return MultiTofContractFault::DistanceAboveProtocolMaximum;
        case ScalarTofFault::ConfidenceZero:
            return MultiTofContractFault::ConfidenceZero;
        case ScalarTofFault::ConfidenceOutOfRange:
            return MultiTofContractFault::ConfidenceOutOfRange;
        case ScalarTofFault::ConfidenceBelowMappingThreshold:
            return MultiTofContractFault::ConfidenceBelowMappingThreshold;
        case ScalarTofFault::DistanceOutsideMappingRange:
            return MultiTofContractFault::DistanceOutsideMappingRange;
        case ScalarTofFault::RequestLatencyTooHigh:
            return MultiTofContractFault::RequestLatencyTooHigh;
        case ScalarTofFault::FutureTimestamp:
            return MultiTofContractFault::FutureTimestamp;
        case ScalarTofFault::InvalidFullFov:
            return MultiTofContractFault::InvalidFullFov;
        case ScalarTofFault::InvalidRequestTiming:
        case ScalarTofFault::StaleTimestamp:
            return MultiTofContractFault::InvalidRequestTiming;
        case ScalarTofFault::None:
        case ScalarTofFault::InvalidFrameLength:
            return MultiTofContractFault::None;
        }
        return MultiTofContractFault::None;
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
