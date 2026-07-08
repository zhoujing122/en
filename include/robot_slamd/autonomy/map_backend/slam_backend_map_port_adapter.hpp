#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_binding.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_contract_checker.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_map_port.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

class SlamBackendMapPortAdapter final : public RobotSlamMapPort {
public:
    SlamBackendMapPortAdapter(std::shared_ptr<SlamBackendBinding> backend,
                              SlamBackendContractChecker checker = {})
        : backend_(std::move(backend)),
          checker_(checker) {}

    bool ready() const override {
        return backend_ && backend_->ready();
    }

    bool integrate_sensor_snapshot(const RobotSlamSensorSnapshot &snapshot,
                                   double now_s) override {
        // Input frame contract check.
        SlamBackendInputFrame input;
        input.timestamp_s = now_s;
        input.snapshot = snapshot;
        input.source = "robot_slam_map_port";
        const auto input_check = checker_.check_input_frame(input, now_s);
        if (input_check.status != SlamBackendUpdateStatus::Accepted) {
            history_.push_back(input_check);
            return false;
        }

        // Backend update.
        if (!backend_ || !backend_->ready()) {
            auto missing = input_check;
            missing.status = SlamBackendUpdateStatus::Fault;
            missing.fault = SlamBackendFault::BackendNotReady;
            missing.message = "slam_backend_not_ready";
            history_.push_back(missing);
            return false;
        }
        const auto update = backend_->update(input);

        // Update result contract check.
        const auto checked = checker_.check_update_result(update);
        history_.push_back(checked);
        if (checked.status == SlamBackendUpdateStatus::Accepted && checked.map_updated) {
            last_quality_ = checked.quality;
            return true;
        }
        return false;
    }

    RobotSlamMapQuality latest_quality(double now_s) const override {
        if (backend_) {
            return backend_->latest_quality(now_s);
        }
        return last_quality_;
    }

    bool save_map(const std::string &output_path) override {
        if (!backend_) {
            return false;
        }
        return backend_->save_map(output_path).ok;
    }

    std::string status() const override {
        if (!backend_) {
            return "slam_backend_missing";
        }
        return backend_->status();
    }

    const std::vector<SlamBackendUpdateResult> &history() const {
        return history_;
    }

private:
    std::shared_ptr<SlamBackendBinding> backend_;
    SlamBackendContractChecker checker_;
    std::vector<SlamBackendUpdateResult> history_;
    RobotSlamMapQuality last_quality_;
};

} // namespace robot_slamd
