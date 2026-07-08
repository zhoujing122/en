#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_binding.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

class ReplaySlamBackendBinding final : public SlamBackendBinding {
public:
    ReplaySlamBackendBinding(std::vector<SlamBackendUpdateResult> update_results,
                             std::vector<RobotSlamMapQuality> qualities,
                             bool ready = true)
        : update_results_(std::move(update_results)),
          qualities_(std::move(qualities)),
          ready_(ready) {}

    bool ready() const override {
        return ready_;
    }

    SlamBackendUpdateResult update(const SlamBackendInputFrame &input) override {
        (void)input;
        update_call_count_++;
        if (update_results_.empty()) {
            SlamBackendUpdateResult result;
            result.status = SlamBackendUpdateStatus::Fault;
            result.fault = SlamBackendFault::BackendNotReady;
            result.message = "replay_slam_backend_no_result";
            return result;
        }
        const auto index = update_index_ < update_results_.size()
                               ? update_index_
                               : update_results_.size() - 1;
        if (update_index_ + 1 < update_results_.size()) {
            update_index_++;
        }
        return update_results_[index];
    }

    RobotSlamMapQuality latest_quality(double now_s) const override {
        (void)now_s;
        if (qualities_.empty()) {
            return RobotSlamMapQuality{};
        }
        const auto index = quality_index_ < qualities_.size()
                               ? quality_index_
                               : qualities_.size() - 1;
        if (quality_index_ + 1 < qualities_.size()) {
            quality_index_++;
        }
        return qualities_[index];
    }

    SlamBackendSaveResult save_map(const std::string &output_path) override {
        last_saved_path_ = output_path;
        SlamBackendSaveResult result;
        result.ok = ready_ && !output_path.empty();
        result.output_path = output_path;
        if (!result.ok) {
            result.error = "replay_slam_backend_save_failed";
        }
        return result;
    }

    std::string status() const override {
        return ready_ ? "replay_slam_backend_ready" : "replay_slam_backend_not_ready";
    }

    void set_ready(bool ready) {
        ready_ = ready;
    }

    int update_call_count() const {
        return update_call_count_;
    }

    const std::string &last_saved_path() const {
        return last_saved_path_;
    }

private:
    std::vector<SlamBackendUpdateResult> update_results_;
    std::vector<RobotSlamMapQuality> qualities_;
    bool ready_ = true;
    std::size_t update_index_ = 0;
    mutable std::size_t quality_index_ = 0;
    int update_call_count_ = 0;
    std::string last_saved_path_;
};

} // namespace robot_slamd
