#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_tof_ray_observation.hpp"

#include <cmath>
#include <string>

namespace robot_slamd {

struct SparseTofObservationBuilderOptions {
    double protocol_min_range_m = 0.02;
    double protocol_max_range_m = 12.0;
    double mapping_min_range_m = 0.05;
    double mapping_max_range_m = 12.0;
    double no_return_free_space_range_m = 12.0;
    int mapping_min_confidence = 70;
    double max_future_timestamp_skew_s = 0.05;
    double stale_after_s = 0.50;
};

struct SparseTofObservationBuildInput {
    ScalarTofSnapshotFrame frame;
    RobotPose2D estimated_pose;
    ScalarTofReturnKind explicit_return_kind = ScalarTofReturnKind::Unspecified;
    bool synchronized = true;
    double now_s = 0.0;
};

class SparseTofObservationBuilder {
public:
    explicit SparseTofObservationBuilder(
        SparseTofObservationBuilderOptions options = {})
        : options_(options) {}

    SparseTofObservationBuildResult build(
        const SparseTofObservationBuildInput &input) const {
        SparseTofObservationBuildResult result;
        result.observation.sensor_origin_x_m = input.estimated_pose.x_m;
        result.observation.sensor_origin_y_m = input.estimated_pose.y_m;
        result.observation.ray_yaw_rad =
            normalize_yaw(input.estimated_pose.yaw_rad +
                          input.frame.mount_yaw_rad);
        result.observation.measured_range_m = input.frame.distance_m;
        result.observation.free_space_range_m = input.frame.distance_m;
        result.observation.confidence = input.frame.confidence;
        result.observation.echo_tag_u48 = input.frame.echo_tag_u48;
        result.observation.effective_timestamp_s =
            input.frame.effective_timestamp_s;
        result.observation.protocol_valid = input.frame.protocol_valid;
        result.observation.synchronized = input.synchronized;
        result.observation.usable_for_mapping = input.frame.usable_for_mapping;
        result.observation.frame_id = input.frame.frame_id;

        const auto explicit_kind = explicit_kind_from_input(input);
        const auto invalid_reason = invalid_reason_for(input);
        if (!invalid_reason.empty()) {
            return finish(result,
                          ScalarTofReturnKind::Invalid,
                          invalid_reason,
                          "sparse_tof_observation_invalid");
        }

        if (explicit_kind == ScalarTofReturnKind::NoReturn) {
            result.observation.free_space_range_m =
                options_.no_return_free_space_range_m;
            return finish(result,
                          ScalarTofReturnKind::NoReturn,
                          "explicit_no_return",
                          "sparse_tof_observation_no_return");
        }

        if (explicit_kind == ScalarTofReturnKind::Invalid) {
            return finish(result,
                          ScalarTofReturnKind::Invalid,
                          "explicit_invalid",
                          "sparse_tof_observation_invalid");
        }

        return finish(result,
                      ScalarTofReturnKind::Hit,
                      "derived_hit",
                      "sparse_tof_observation_hit");
    }

private:
    SparseTofObservationBuilderOptions options_;

    static double normalize_yaw(double yaw) {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 6.28318530717958647692;
        while (yaw > kPi) {
            yaw -= kTwoPi;
        }
        while (yaw < -kPi) {
            yaw += kTwoPi;
        }
        return yaw;
    }

    ScalarTofReturnKind explicit_kind_from_input(
        const SparseTofObservationBuildInput &input) const {
        if (input.explicit_return_kind != ScalarTofReturnKind::Unspecified) {
            return input.explicit_return_kind;
        }
        if (input.frame.source.find("explicit_no_return") !=
            std::string::npos) {
            return ScalarTofReturnKind::NoReturn;
        }
        return ScalarTofReturnKind::Unspecified;
    }

    std::string invalid_reason_for(
        const SparseTofObservationBuildInput &input) const {
        if (!std::isfinite(input.now_s) ||
            !std::isfinite(input.estimated_pose.x_m) ||
            !std::isfinite(input.estimated_pose.y_m) ||
            !std::isfinite(input.estimated_pose.yaw_rad)) {
            return "pose_or_time_invalid";
        }
        if (!input.synchronized) {
            return "sync_rejected";
        }
        if (!input.frame.protocol_valid) {
            return "protocol_invalid";
        }
        if (!std::isfinite(input.frame.distance_m) ||
            !std::isfinite(input.frame.effective_timestamp_s)) {
            return "measurement_not_finite";
        }
        if (input.frame.effective_timestamp_s >
            input.now_s + options_.max_future_timestamp_skew_s) {
            return "future_timestamp";
        }
        if (input.now_s - input.frame.effective_timestamp_s >
            options_.stale_after_s) {
            return "stale_timestamp";
        }
        if (input.frame.confidence == 0) {
            return "confidence_zero";
        }
        if (input.frame.confidence > 100) {
            return "confidence_out_of_range";
        }
        if (input.frame.confidence <
            static_cast<std::uint8_t>(options_.mapping_min_confidence)) {
            return "confidence_below_mapping_threshold";
        }
        if (input.frame.distance_m < options_.protocol_min_range_m ||
            input.frame.distance_m > options_.protocol_max_range_m) {
            return "distance_outside_protocol_range";
        }
        if (input.explicit_return_kind != ScalarTofReturnKind::NoReturn &&
            input.frame.source.find("explicit_no_return") ==
                std::string::npos &&
            (input.frame.distance_m < options_.mapping_min_range_m ||
             input.frame.distance_m > options_.mapping_max_range_m)) {
            return "distance_outside_mapping_range";
        }
        return "";
    }

    static SparseTofObservationBuildResult finish(
        SparseTofObservationBuildResult result,
        ScalarTofReturnKind kind,
        const std::string &reason,
        const std::string &message) {
        result.observation.return_kind = kind;
        result.observation.reason = reason;
        result.ok = kind != ScalarTofReturnKind::Invalid;
        result.fault = kind == ScalarTofReturnKind::Invalid ? reason : "";
        result.message = message;
        if (kind == ScalarTofReturnKind::Invalid) {
            result.observation.usable_for_mapping = false;
        }
        return result;
    }
};

} // namespace robot_slamd
