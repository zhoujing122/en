#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"

#include <string>

namespace robot_slamd {

class RobotSlamSensorPort {
public:
    virtual ~RobotSlamSensorPort() = default;

    virtual bool ready() const = 0;
    virtual RobotSlamSensorSnapshot latest_snapshot(double now_s) = 0;
    virtual std::string status() const = 0;
};

} // namespace robot_slamd
