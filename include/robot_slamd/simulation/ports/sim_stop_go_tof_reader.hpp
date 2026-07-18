#pragma once

#include "robot_slamd/sensors/nuona_tof_i2c_frame.hpp"
#include "robot_slamd/sensors/stable_three_tof_sampler.hpp"
#include "robot_slamd/simulation/core/sim_clock.hpp"
#include "robot_slamd/simulation/sensors/sim_three_scalar_tof.hpp"

#include <cmath>
#include <memory>

namespace robot_slamd {

class SimStopGoTofReader {
public:
    SimStopGoTofReader(std::shared_ptr<SimClock> clock,
                       std::shared_ptr<FakeWorld2D> world,
                       std::shared_ptr<SimRobotPlant> plant,
                       SimThreeScalarTof tof,
                       StableTofDirection direction)
        : clock_(std::move(clock)), world_(std::move(world)),
          plant_(std::move(plant)), tof_(std::move(tof)), direction_(direction) {}

    TimedTofFrame read() {
        TimedTofFrame timed;
        if (!clock_ || !world_ || !plant_) return timed;
        const double now = clock_->now_s();
        const auto sample = tof_.sample(*world_, plant_->state(), now);
        const MultiTofRawFrame *raw = nullptr;
        if (direction_ == StableTofDirection::Front && sample.packet.has_front) raw = &sample.packet.front;
        if (direction_ == StableTofDirection::Left && sample.packet.has_left) raw = &sample.packet.left;
        if (direction_ == StableTofDirection::Right && sample.packet.has_right) raw = &sample.packet.right;
        if (!raw) return timed;
        const auto bytes = encode_nuona_tof_i2c_frame(raw->distance_mm, raw->confidence);
        timed = make_timed_tof_frame(
            parse_nuona_tof_i2c_frame(bytes),
            static_cast<std::uint64_t>(std::llround(raw->timing.request_start_s * 1e6)),
            static_cast<std::uint64_t>(std::llround(raw->timing.response_received_s * 1e6)));
        return timed;
    }

private:
    std::shared_ptr<SimClock> clock_;
    std::shared_ptr<FakeWorld2D> world_;
    std::shared_ptr<SimRobotPlant> plant_;
    SimThreeScalarTof tof_;
    StableTofDirection direction_;
};

} // namespace robot_slamd
