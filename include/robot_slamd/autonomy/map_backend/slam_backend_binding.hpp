#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_types.hpp"

#include <string>

namespace robot_slamd {

class SlamBackendBinding {
public:
    virtual ~SlamBackendBinding() = default;

    virtual bool ready() const = 0;

    virtual SlamBackendUpdateResult update(
        const SlamBackendInputFrame &input) = 0;

    virtual RobotSlamMapQuality latest_quality(double now_s) const = 0;

    virtual SlamBackendSaveResult save_map(
        const std::string &output_path) = 0;

    virtual std::string status() const = 0;
};

} // namespace robot_slamd
