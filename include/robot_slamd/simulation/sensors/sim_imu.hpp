#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_data_contract_types.hpp"
#include "robot_slamd/simulation/robot/sim_robot_plant.hpp"

namespace robot_slamd {

struct SimImuConfig {
    double sample_period_s = 0.02;
    double yaw_rate_bias_rad_s = 0.0;
    double accel_x_bias_mps2 = 0.0;
    double accel_y_bias_mps2 = 0.0;
    double accel_z_bias_mps2 = 0.0;
    double gravity_mps2 = 9.80665;
    bool dropout = false;
    bool stale = false;
    double timestamp_offset_s = 0.0;
};

struct SimImuSample {
    bool ok = false;
    bool fresh = false;
    RealSensorRawImuSample raw;
    std::string message;
};

class SimImu {
public:
    explicit SimImu(SimImuConfig config = {})
        : config_(config) {}

    bool valid() const {
        return sim_finite(config_.sample_period_s) &&
               config_.sample_period_s > 0.0 &&
               sim_finite(config_.yaw_rate_bias_rad_s) &&
               sim_finite(config_.accel_x_bias_mps2) &&
               sim_finite(config_.accel_y_bias_mps2) &&
               sim_finite(config_.accel_z_bias_mps2) &&
               sim_finite(config_.gravity_mps2) &&
               sim_finite(config_.timestamp_offset_s);
    }

    SimImuSample sample(const SimRobotState &state, double now_s) {
        SimImuSample sample;
        if (!valid() || !sim_finite(now_s) || config_.dropout) {
            sample.message = config_.dropout ? "sim_imu_dropout" : "sim_imu_invalid_config";
            return sample;
        }
        const double timestamp = config_.stale ? last_timestamp_s_
                                               : now_s + config_.timestamp_offset_s;
        const double dt = (last_timestamp_s_ > 0.0 && timestamp > last_timestamp_s_)
                              ? timestamp - last_timestamp_s_
                              : config_.sample_period_s;
        const double accel_longitudinal =
            (state.linear_velocity_mps - last_linear_velocity_mps_) / dt;
        sample.raw.timestamp_s = timestamp;
        sample.raw.frame_id = "sim_imu";
        sample.raw.yaw_rate_rad_s = state.angular_velocity_rad_s +
                                    config_.yaw_rate_bias_rad_s;
        sample.raw.accel_x_mps2 = accel_longitudinal * std::cos(state.pose.yaw_rad) +
                                  config_.accel_x_bias_mps2;
        sample.raw.accel_y_mps2 = accel_longitudinal * std::sin(state.pose.yaw_rad) +
                                  config_.accel_y_bias_mps2;
        sample.raw.accel_z_mps2 = config_.gravity_mps2 + config_.accel_z_bias_mps2;
        sample.raw.sequence = sequence_++;
        sample.raw.source = "sim_imu";
        sample.ok = !config_.stale;
        sample.fresh = sample.ok;
        sample.message = sample.ok ? "sim_imu_ok" : "sim_imu_stale";
        if (sample.ok) {
            last_timestamp_s_ = timestamp;
            last_linear_velocity_mps_ = state.linear_velocity_mps;
        }
        return sample;
    }

private:
    SimImuConfig config_;
    int sequence_ = 1;
    double last_timestamp_s_ = 0.0;
    double last_linear_velocity_mps_ = 0.0;
};

} // namespace robot_slamd
