#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_binding.hpp"
#include "robot_slamd/autonomy/real_adapters/real_adapter_stub_types.hpp"

#include <utility>

namespace robot_slamd {

class RealSlamBackendBindingStub final : public SlamBackendBinding {
public:
    explicit RealSlamBackendBindingStub(
        RealAdapterStubStatus status = default_status())
        : status_(std::move(status)) {}

    bool ready() const override {
        return status_.ready;
    }

    SlamBackendUpdateResult update(
        const SlamBackendInputFrame &input) override {
        (void)input;
        update_call_count_++;

        SlamBackendUpdateResult result;
        result.status = SlamBackendUpdateStatus::Rejected;
        result.fault = SlamBackendFault::BackendNotReady;
        result.map_updated = false;
        result.message = "real_slam_backend_binding_stub_not_implemented";
        return result;
    }

    RobotSlamMapQuality latest_quality(double now_s) const override {
        (void)now_s;
        RobotSlamMapQuality quality;
        quality.good_enough = false;
        quality.reason = "real_slam_backend_binding_stub_not_implemented";
        return quality;
    }

    SlamBackendSaveResult save_map(
        const std::string &output_path) override {
        SlamBackendSaveResult result;
        result.ok = false;
        result.output_path = output_path;
        result.error = "real_slam_backend_binding_stub_not_implemented";
        return result;
    }

    std::string status() const override {
        return "RealSlamBackendBindingStub: "
               "waiting_for_real_slam_backend_implementation";
    }

    RealAdapterStubStatus stub_status() const {
        return status_;
    }

    int update_call_count() const {
        return update_call_count_;
    }

private:
    static RealAdapterStubStatus default_status() {
        RealAdapterStubStatus status;
        status.kind = RealAdapterStubKind::SlamBackend;
        status.state = RealAdapterStubState::WaitingForImplementation;
        status.ready = false;
        status.name = "RealSlamBackendBindingStub";
        status.message = "waiting_for_real_slam_backend_implementation";
        return status;
    }

    RealAdapterStubStatus status_;
    int update_call_count_ = 0;
};

} // namespace robot_slamd
