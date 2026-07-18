#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_builder.hpp"
#include "robot_slamd/simulation/core/sim_clock.hpp"
#include "robot_slamd/simulation/sensors/sim_imu.hpp"
#include "robot_slamd/simulation/sensors/sim_three_scalar_tof.hpp"
#include "robot_slamd/simulation/sensors/sim_wheel_encoder.hpp"

#include <memory>
#include <sstream>

namespace robot_slamd {

struct SimSensorPortConfig {
    MultiTofContractOptions contract_options;
    MultiTofSyncOptions sync_options;
    MultiTofSnapshotBuildOptions build_options;
};

class SimSensorPort final : public RobotSlamSensorPort {
public:
    SimSensorPort(std::shared_ptr<SimClock> clock,
                  std::shared_ptr<FakeWorld2D> world,
                  std::shared_ptr<SimRobotPlant> plant,
                  SimSensorPortConfig config = {},
                  SimWheelEncoder wheel = SimWheelEncoder{},
                  SimImu imu = SimImu{},
                  SimThreeScalarTof tof = SimThreeScalarTof{})
        : clock_(std::move(clock)),
          world_(std::move(world)),
          plant_(std::move(plant)),
          config_(config),
          wheel_(wheel),
          imu_(imu),
          tof_(tof),
          builder_(config_.contract_options, config_.sync_options, config_.build_options) {
        config_.sync_options.enabled = true;
        config_.build_options.enabled = true;
        builder_ = MultiTofSnapshotBuilder(config_.contract_options,
                                           config_.sync_options,
                                           config_.build_options);
    }

    bool ready() const override {
        return clock_ && world_ && plant_ && plant_->valid() &&
               wheel_.valid() && imu_.valid() && tof_.valid() &&
               config_.build_options.enabled;
    }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        read_call_count_++;
        last_message_ = "sim_sensor_not_ready";
        if (!ready() || !sim_finite(now_s) ||
            std::fabs(now_s - clock_->now_s()) > 1e-9) {
            rejected_count_++;
            last_message_ = "sim_sensor_invalid_time_or_not_ready";
            return RobotSlamSensorSnapshot{};
        }
        if (last_success_time_s_ >= 0.0 && now_s <= last_success_time_s_) {
            rejected_count_++;
            last_message_ = "sim_sensor_time_not_advanced";
            return RobotSlamSensorSnapshot{};
        }

        const auto &state = plant_->state();
        auto wheel = wheel_.sample(state, now_s);
        auto imu = imu_.sample(state, now_s);
        auto tof = tof_.sample(*world_, state, now_s);
        wheel_sample_count_ += wheel.ok ? 1 : 0;
        imu_sample_count_ += imu.ok ? 1 : 0;
        front_tof_count_ += tof.front_count;
        left_tof_count_ += tof.left_count;
        right_tof_count_ += tof.right_count;

        if (!wheel.ok || !imu.ok || !tof.ok) {
            rejected_count_++;
            last_message_ = "sim_sensor_sample_failed";
            return RobotSlamSensorSnapshot{};
        }
        tof.packet.has_imu = true;
        tof.packet.imu = imu.raw;
        tof.packet.has_wheel = true;
        tof.packet.wheel = wheel.raw;

        last_build_result_ = builder_.build(tof.packet, now_s);
        if (!last_build_result_.ok) {
            rejected_count_++;
            last_message_ = last_build_result_.message;
            return RobotSlamSensorSnapshot{};
        }
        // Preserve wheel feedback diagnostics in the canonical snapshot.  The
        // mapping controller uses these fields only for its settle gate; the
        // SLAM core still integrates the canonical wheel pose fields above.
        last_build_result_.snapshot.wheel.left_ticks = wheel.left_position_ticks;
        last_build_result_.snapshot.wheel.right_ticks = wheel.right_position_ticks;
        last_build_result_.snapshot.wheel.pair_skew_s = wheel.pair_skew_s;
        last_build_result_.snapshot.wheel.pair_skew_valid =
            std::isfinite(wheel.pair_skew_s) &&
            std::fabs(wheel.pair_skew_s) <= 0.010000001;
        const double left_rpm = state.left_wheel_speed_rad_s * 60.0 /
                                (2.0 * kPi);
        const double right_rpm = state.right_wheel_speed_rad_s * 60.0 /
                                 (2.0 * kPi);
        last_build_result_.snapshot.wheel.left_rpm = left_rpm;
        last_build_result_.snapshot.wheel.right_rpm = right_rpm;
        success_count_++;
        last_success_time_s_ = now_s;
        last_message_ = "sim_sensor_snapshot_ok";
        return last_build_result_.snapshot;
    }

    std::string status() const override {
        std::ostringstream out;
        out << "sim_sensor ready=" << (ready() ? 1 : 0)
            << " read_calls=" << read_call_count_
            << " success=" << success_count_
            << " rejected=" << rejected_count_
            << " wheel=" << wheel_sample_count_
            << " imu=" << imu_sample_count_
            << " front=" << front_tof_count_
            << " left=" << left_tof_count_
            << " right=" << right_tof_count_
            << " last_message=" << last_message_;
        return out.str();
    }

    int read_call_count() const { return read_call_count_; }
    int success_count() const { return success_count_; }
    int rejected_count() const { return rejected_count_; }
    int wheel_sample_count() const { return wheel_sample_count_; }
    int imu_sample_count() const { return imu_sample_count_; }
    int front_tof_count() const { return front_tof_count_; }
    int left_tof_count() const { return left_tof_count_; }
    int right_tof_count() const { return right_tof_count_; }
    std::string last_message() const { return last_message_; }
    const MultiTofSnapshotBuildResult &last_build_result() const {
        return last_build_result_;
    }

private:
    std::shared_ptr<SimClock> clock_;
    std::shared_ptr<FakeWorld2D> world_;
    std::shared_ptr<SimRobotPlant> plant_;
    SimSensorPortConfig config_;
    SimWheelEncoder wheel_;
    SimImu imu_;
    SimThreeScalarTof tof_;
    MultiTofSnapshotBuilder builder_;
    MultiTofSnapshotBuildResult last_build_result_;
    int read_call_count_ = 0;
    int success_count_ = 0;
    int rejected_count_ = 0;
    int wheel_sample_count_ = 0;
    int imu_sample_count_ = 0;
    int front_tof_count_ = 0;
    int left_tof_count_ = 0;
    int right_tof_count_ = 0;
    double last_success_time_s_ = -1.0;
    std::string last_message_ = "sim_sensor_not_read";
};

} // namespace robot_slamd
