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

    bool integrate_map_update(const RobotSlamMapUpdateInput &map_update) override {
        if (!map_update.mapping_commit_allowed) {
            SlamBackendUpdateResult skipped;
            skipped.status = SlamBackendUpdateStatus::Skipped;
            skipped.fault = SlamBackendFault::None;
            skipped.map_updated = false;
            skipped.message = "mapping_commit_not_allowed";
            history_.push_back(skipped);
            return false;
        }

        // Input frame contract check.
        SlamBackendInputFrame input;
        input.timestamp_s = map_update.timestamp_s;
        input.snapshot = map_update.snapshot;
        input.predicted_pose = map_update.predicted_map_pose;
        input.has_predicted_pose = map_update.has_predicted_map_pose;
        input.multi_tof_measurement_poses =
            map_update.multi_tof_measurement_poses;
        input.has_multi_tof_measurement_poses =
            map_update.has_multi_tof_measurement_poses;
        input.source = map_update.source.empty()
                           ? "robot_slam_map_port"
                           : map_update.source;
        const auto input_check = checker_.check_input_frame(input, map_update.timestamp_s);
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

    bool integrate_sensor_snapshot(const RobotSlamSensorSnapshot &snapshot,
                                   double now_s) override {
        RobotSlamMapUpdateInput input;
        input.timestamp_s = now_s;
        input.snapshot = snapshot;
        input.source = "robot_slam_map_port";
        input.has_predicted_map_pose = false;
        input.mapping_commit_allowed = true;
        return integrate_map_update(input);
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
