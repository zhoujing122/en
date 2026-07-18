#pragma once

#include "robot_slamd/autonomy/ports/relative_motion_step_port.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_slamd {

struct MappingSettleGateConfig {
    double wheel_rpm_threshold = 1.0;
    double imu_gyro_threshold_deg_s = 1.0;
    std::size_t consecutive_frames = 3;
    std::uint64_t hold_ms = 200;
    double max_pair_skew_ms = 10.0;
};

struct MappingSettleGateResult {
    bool stable = false;
    std::size_t consecutive_frames = 0;
    double stable_duration_ms = 0.0;
    std::uint64_t stable_timestamp_us = 0;
    std::string reason = "not_started";
};

class MappingSettleGate final {
public:
    explicit MappingSettleGate(MappingSettleGateConfig config = {})
        : config_(config) {}

    void reset() {
        consecutive_ = 0;
        first_stable_us_ = 0;
        last_timestamp_us_ = 0;
        result_ = {};
    }

    MappingSettleGateResult update(
        const RelativeMotionStepResult &motion,
        const RobotSlamSensorSnapshot &snapshot,
        std::uint64_t now_us,
        bool initial_settle = false) {
        result_ = {};
        result_.stable_timestamp_us = now_us;
        if (!initial_settle && motion.state != RelativeMotionStepState::Done) {
            return fail("motion_not_done");
        }
        if (!snapshot.has_wheel || !snapshot.wheel.valid || !snapshot.has_imu) {
            return fail("wheel_or_imu_invalid");
        }
        if (!std::isfinite(snapshot.wheel.left_rpm) ||
            !std::isfinite(snapshot.wheel.right_rpm) ||
            !std::isfinite(snapshot.imu.yaw_rate_rad_s) ||
            !std::isfinite(snapshot.wheel.timestamp_s) ||
            !std::isfinite(snapshot.imu.timestamp_s) ||
            !std::isfinite(snapshot.wheel.pair_skew_s)) {
            return fail("settle_feedback_non_finite");
        }
        if (!snapshot.wheel.pair_skew_valid ||
            std::fabs(snapshot.wheel.pair_skew_s) * 1000.0 >
                config_.max_pair_skew_ms + 1e-9) {
            return fail("wheel_pair_skew_exceeded");
        }
        if (snapshot.wheel.timestamp_s < 0.0 ||
            snapshot.imu.timestamp_s < 0.0 ||
            snapshot.wheel.timestamp_s > now_us / 1e6 + 1e-6 ||
            snapshot.imu.timestamp_s > now_us / 1e6 + 1e-6) {
            return fail("feedback_timestamp_invalid");
        }
        if (last_timestamp_us_ != 0 && now_us <= last_timestamp_us_) {
            return fail("timestamp_not_monotonic");
        }
        last_timestamp_us_ = now_us;
        const double gyro_deg_s = std::fabs(snapshot.imu.yaw_rate_rad_s) *
                                  180.0 / 3.14159265358979323846;
        const bool quiet =
            std::fabs(snapshot.wheel.left_rpm) <= config_.wheel_rpm_threshold &&
            std::fabs(snapshot.wheel.right_rpm) <= config_.wheel_rpm_threshold &&
            gyro_deg_s <= config_.imu_gyro_threshold_deg_s;
        if (!quiet) {
            consecutive_ = 0;
            first_stable_us_ = 0;
            result_.reason = "motion_not_quiet";
            return result_;
        }
        if (consecutive_ == 0) first_stable_us_ = now_us;
        consecutive_++;
        result_.consecutive_frames = consecutive_;
        result_.stable_duration_ms =
            first_stable_us_ == 0 ? 0.0 : (now_us - first_stable_us_) / 1000.0;
        result_.reason = "quiet_frame";
        result_.stable = consecutive_ >= config_.consecutive_frames &&
                         result_.stable_duration_ms + 1e-9 >= config_.hold_ms;
        if (result_.stable) result_.reason = "mapping_ready_settled";
        return result_;
    }

    const MappingSettleGateResult &last_result() const { return result_; }

private:
    MappingSettleGateResult fail(const char *reason) {
        consecutive_ = 0;
        first_stable_us_ = 0;
        result_.reason = reason;
        return result_;
    }

    MappingSettleGateConfig config_;
    std::size_t consecutive_ = 0;
    std::uint64_t first_stable_us_ = 0;
    std::uint64_t last_timestamp_us_ = 0;
    MappingSettleGateResult result_;
};

} // namespace robot_slamd
