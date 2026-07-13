#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_types.hpp"
#include "robot_slamd/simulation/robot/sim_robot_plant.hpp"

#include <cmath>
#include <cstdint>

namespace robot_slamd {

struct SimWheelEncoderConfig {
    double ticks_per_revolution = 2048.0;
    std::uint32_t position_modulus = 65536;
    int left_direction_sign = 1;
    int right_direction_sign = 1;
    double sample_period_s = 0.02;
    double left_request_latency_s = 0.002;
    double right_request_latency_s = 0.002;
    double injected_pair_skew_s = 0.0;
    bool dropout = false;
    bool stale = false;
};

struct SimWheelEncoderSample {
    bool ok = false;
    bool fresh = false;
    RealSensorRawWheelSample raw;
    std::uint32_t left_position_ticks = 0;
    std::uint32_t right_position_ticks = 0;
    double left_effective_time_s = 0.0;
    double right_effective_time_s = 0.0;
    double pair_timestamp_s = 0.0;
    double pair_skew_s = 0.0;
    std::string message;
};

class SimWheelEncoder {
public:
    explicit SimWheelEncoder(SimWheelEncoderConfig config = {})
        : config_(config) {}

    bool valid() const {
        return sim_finite(config_.ticks_per_revolution) &&
               config_.ticks_per_revolution > 0.0 &&
               config_.position_modulus > 0 &&
               sim_finite(config_.sample_period_s) &&
               config_.sample_period_s > 0.0 &&
               sim_finite(config_.left_request_latency_s) &&
               config_.left_request_latency_s >= 0.0 &&
               sim_finite(config_.right_request_latency_s) &&
               config_.right_request_latency_s >= 0.0 &&
               sim_finite(config_.injected_pair_skew_s);
    }

    SimWheelEncoderSample sample(const SimRobotState &state, double now_s) {
        SimWheelEncoderSample sample;
        sample.message = "sim_wheel_not_ready";
        if (!valid() || !sim_finite(now_s) || config_.dropout) {
            sample.message = config_.dropout ? "sim_wheel_dropout" : "sim_wheel_invalid_config";
            return sample;
        }
        const double left_response = config_.stale ? last_publish_time_s_ : now_s;
        const double right_response = left_response + config_.injected_pair_skew_s;
        const double left_start = left_response - config_.left_request_latency_s;
        const double right_start = right_response - config_.right_request_latency_s;
        sample.left_effective_time_s = 0.5 * (left_start + left_response);
        sample.right_effective_time_s = 0.5 * (right_start + right_response);
        sample.pair_timestamp_s = 0.5 * (sample.left_effective_time_s +
                                         sample.right_effective_time_s);
        sample.pair_skew_s = sample.right_effective_time_s - sample.left_effective_time_s;
        sample.raw.timing = make_request_timing(
            sample.pair_timestamp_s - 0.5 * config_.sample_period_s,
            sample.pair_timestamp_s + 0.5 * config_.sample_period_s,
            "sim_bl4820_pair_midpoint");
        sample.raw.timing.estimated_sample_time_s = sample.pair_timestamp_s;
        sample.raw.frame_id = "sim_wheel_pair";
        sample.raw.valid = !config_.stale;
        sample.raw.linear_velocity_mps = state.linear_velocity_mps;
        sample.raw.angular_velocity_rad_s = state.angular_velocity_rad_s;
        sample.raw.sequence = sequence_++;
        sample.raw.source = "sim_wheel_encoder";
        sample.left_position_ticks = to_ticks(state.left_wheel_position_rad,
                                             config_.left_direction_sign);
        sample.right_position_ticks = to_ticks(state.right_wheel_position_rad,
                                              config_.right_direction_sign);
        sample.ok = sample.raw.valid;
        sample.fresh = sample.raw.valid;
        sample.message = sample.ok ? "sim_wheel_ok" : "sim_wheel_stale";
        if (sample.ok) {
            last_publish_time_s_ = now_s;
        }
        return sample;
    }

    const SimWheelEncoderConfig &config() const {
        return config_;
    }

private:
    std::uint32_t to_ticks(double wheel_position_rad, int sign) const {
        const double revolutions =
            static_cast<double>(sign) * wheel_position_rad / kSimTwoPi;
        const long long raw_ticks =
            static_cast<long long>(std::llround(revolutions * config_.ticks_per_revolution));
        const long long modulus = static_cast<long long>(config_.position_modulus);
        long long wrapped = raw_ticks % modulus;
        if (wrapped < 0) {
            wrapped += modulus;
        }
        return static_cast<std::uint32_t>(wrapped);
    }

    SimWheelEncoderConfig config_;
    int sequence_ = 1;
    double last_publish_time_s_ = 0.0;
};

} // namespace robot_slamd
