#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/real_adapters/real_adapter_stub_types.hpp"

#include <utility>

namespace robot_slamd {

class RealTofImuWheelSensorPortStub final : public RobotSlamSensorPort {
public:
    explicit RealTofImuWheelSensorPortStub(
        RealAdapterStubStatus status = default_status())
        : status_(std::move(status)) {}

    bool ready() const override {
        return status_.ready;
    }

    RobotSlamSensorSnapshot latest_snapshot(double now_s) override {
        (void)now_s;
        return {};
    }

    std::string status() const override {
        return "RealTofImuWheelSensorPortStub: "
               "waiting_for_real_sensor_adapter_implementation";
    }

    RealAdapterStubStatus stub_status() const {
        return status_;
    }

private:
    static RealAdapterStubStatus default_status() {
        RealAdapterStubStatus status;
        status.kind = RealAdapterStubKind::Sensor;
        status.state = RealAdapterStubState::WaitingForImplementation;
        status.ready = false;
        status.name = "RealTofImuWheelSensorPortStub";
        status.message = "waiting_for_real_sensor_adapter_implementation";
        return status;
    }

    RealAdapterStubStatus status_;
};

} // namespace robot_slamd
