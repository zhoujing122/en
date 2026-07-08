#pragma once

#include "robot_slamd/autonomy/real_adapters/real_slam_backend_binding_stub.hpp"
#include "robot_slamd/autonomy/real_adapters/real_software_motion_port_stub.hpp"
#include "robot_slamd/autonomy/real_adapters/real_tof_imu_wheel_sensor_port_stub.hpp"

#include <memory>

namespace robot_slamd {

class RealAdapterFactory {
public:
    explicit RealAdapterFactory(RealAdapterFactoryOptions options = {})
        : options_(options) {}

    std::shared_ptr<RobotSlamSensorPort> create_sensor_port() const {
        if (!options_.enabled || !options_.create_sensor_stub) {
            return nullptr;
        }

        return std::make_shared<RealTofImuWheelSensorPortStub>();
    }

    std::shared_ptr<RobotSlamMotionPort> create_motion_port() const {
        if (!options_.enabled || !options_.create_motion_stub) {
            return nullptr;
        }

        return std::make_shared<RealSoftwareMotionPortStub>();
    }

    std::shared_ptr<SlamBackendBinding> create_slam_backend() const {
        if (!options_.enabled || !options_.create_slam_backend_stub) {
            return nullptr;
        }

        return std::make_shared<RealSlamBackendBindingStub>();
    }

    RealAdapterFactoryOptions options() const {
        return options_;
    }

private:
    RealAdapterFactoryOptions options_;
};

} // namespace robot_slamd
