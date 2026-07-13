#pragma once

#include "robot_slamd/simulation/core/sim_math.hpp"

namespace robot_slamd {

class SimClock {
public:
    explicit SimClock(double start_time_s = 0.0) {
        reset(start_time_s);
    }

    double now_s() const {
        return now_s_;
    }

    bool advance(double dt_s) {
        if (!sim_finite(dt_s) || dt_s <= 0.0) {
            return false;
        }
        const double next = now_s_ + dt_s;
        if (!sim_finite(next) || next <= now_s_) {
            return false;
        }
        now_s_ = next;
        return true;
    }

    bool reset(double start_time_s = 0.0) {
        if (!sim_finite(start_time_s)) {
            return false;
        }
        now_s_ = start_time_s;
        return true;
    }

private:
    double now_s_ = 0.0;
};

} // namespace robot_slamd
