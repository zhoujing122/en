#pragma once

#include <cmath>
#include <cstddef>
#include <string>

namespace robot_slamd {

struct MotionSettleGateConfig {
    double max_abs_linear_speed_mps = 0.02;
    double max_abs_wheel_yaw_rate_rad_s = 0.02;
    double max_abs_imu_yaw_rate_rad_s = 0.02;
    std::size_t min_consecutive_samples = 3;
    double min_stable_duration_s = 0.20;
    double max_sample_gap_s = 0.20;
};

inline bool motion_settle_gate_config_valid(
    const MotionSettleGateConfig &config) {
    return std::isfinite(config.max_abs_linear_speed_mps) &&
           config.max_abs_linear_speed_mps >= 0.0 &&
           std::isfinite(config.max_abs_wheel_yaw_rate_rad_s) &&
           config.max_abs_wheel_yaw_rate_rad_s >= 0.0 &&
           std::isfinite(config.max_abs_imu_yaw_rate_rad_s) &&
           config.max_abs_imu_yaw_rate_rad_s >= 0.0 &&
           config.min_consecutive_samples > 0 &&
           std::isfinite(config.min_stable_duration_s) &&
           config.min_stable_duration_s >= 0.0 &&
           std::isfinite(config.max_sample_gap_s) &&
           config.max_sample_gap_s > 0.0;
}

struct MotionSettleSample {
    double timestamp_s = 0.0;
    bool wheel_fresh = false;
    double linear_mps = 0.0;
    double wheel_angular_rad_s = 0.0;
    bool imu_fresh = false;
    double imu_yaw_rate_rad_s = 0.0;
};

struct MotionSettleGateResult {
    bool ok = false;
    bool stable = false;
    bool reset = false;
    std::size_t consecutive_sample_count = 0;
    double stable_duration_s = 0.0;
    std::string reason;
};

class MotionSettleGate {
public:
    explicit MotionSettleGate(MotionSettleGateConfig config = {})
        : config_(config) {}

    void reset() {
        last_timestamp_s_ = 0.0;
        stable_start_s_ = 0.0;
        consecutive_sample_count_ = 0;
        has_last_timestamp_ = false;
        has_stable_start_ = false;
    }

    MotionSettleGateResult update(const MotionSettleSample &sample) {
        MotionSettleGateResult result;
        if (!motion_settle_gate_config_valid(config_)) {
            result.reason = "motion_settle_invalid_config";
            return result;
        }
        if (!sample_valid(sample)) {
            reset();
            result.ok = false;
            result.reset = true;
            result.reason = "motion_settle_sample_invalid";
            return result;
        }
        if (has_last_timestamp_ && sample.timestamp_s <= last_timestamp_s_) {
            reset();
            result.ok = false;
            result.reset = true;
            result.reason = "motion_settle_timestamp_not_increasing";
            return result;
        }
        if (has_last_timestamp_ &&
            sample.timestamp_s - last_timestamp_s_ > config_.max_sample_gap_s) {
            reset_stability(sample.timestamp_s);
            result.reset = true;
        }
        last_timestamp_s_ = sample.timestamp_s;
        has_last_timestamp_ = true;

        if (!within_thresholds(sample)) {
            reset_stability(sample.timestamp_s);
            result.ok = true;
            result.reset = true;
            result.reason = "motion_settle_speed_not_stable";
            result.consecutive_sample_count = consecutive_sample_count_;
            result.stable_duration_s = 0.0;
            return result;
        }

        if (!has_stable_start_) {
            stable_start_s_ = sample.timestamp_s;
            has_stable_start_ = true;
            consecutive_sample_count_ = 1;
        } else {
            consecutive_sample_count_++;
        }
        result.ok = true;
        result.consecutive_sample_count = consecutive_sample_count_;
        result.stable_duration_s = sample.timestamp_s - stable_start_s_;
        result.stable = consecutive_sample_count_ >= config_.min_consecutive_samples &&
                        result.stable_duration_s >= config_.min_stable_duration_s;
        result.reason = result.stable ? "motion_settle_stable"
                                      : "motion_settle_collecting";
        return result;
    }

    std::size_t consecutive_sample_count() const {
        return consecutive_sample_count_;
    }
    double stable_duration_s(double now_s) const {
        if (!has_stable_start_ || !std::isfinite(now_s)) {
            return 0.0;
        }
        return now_s - stable_start_s_;
    }

private:
    bool sample_valid(const MotionSettleSample &sample) const {
        return sample.wheel_fresh && sample.imu_fresh &&
               std::isfinite(sample.timestamp_s) &&
               std::isfinite(sample.linear_mps) &&
               std::isfinite(sample.wheel_angular_rad_s) &&
               std::isfinite(sample.imu_yaw_rate_rad_s);
    }

    bool within_thresholds(const MotionSettleSample &sample) const {
        return std::fabs(sample.linear_mps) <=
                   config_.max_abs_linear_speed_mps &&
               std::fabs(sample.wheel_angular_rad_s) <=
                   config_.max_abs_wheel_yaw_rate_rad_s &&
               std::fabs(sample.imu_yaw_rate_rad_s) <=
                   config_.max_abs_imu_yaw_rate_rad_s;
    }

    void reset_stability(double timestamp_s) {
        stable_start_s_ = timestamp_s;
        consecutive_sample_count_ = 0;
        has_stable_start_ = false;
    }

    MotionSettleGateConfig config_;
    double last_timestamp_s_ = 0.0;
    double stable_start_s_ = 0.0;
    std::size_t consecutive_sample_count_ = 0;
    bool has_last_timestamp_ = false;
    bool has_stable_start_ = false;
};

} // namespace robot_slamd
