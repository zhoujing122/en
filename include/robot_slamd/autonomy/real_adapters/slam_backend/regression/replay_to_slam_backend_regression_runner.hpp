#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_binding.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct ReplayToSlamBackendRegressionOptions {
    bool enabled = false;
    bool require_valid_replay_updates_map = true;
    bool require_invalid_replay_rejected = true;
    int min_accepted_updates = 3;
};

struct ReplayToSlamBackendRegressionReport {
    bool ok = false;
    int replay_packet_count = 0;
    int accepted_update_count = 0;
    int rejected_update_count = 0;
    RobotSlamMapQuality final_quality;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class ReplayToSlamBackendRegressionRunner {
public:
    explicit ReplayToSlamBackendRegressionRunner(
        ReplayToSlamBackendRegressionOptions options = {})
        : options_(options) {}

    ReplayToSlamBackendRegressionReport run_once() const {
        ReplayToSlamBackendRegressionReport report;
        DeterministicSlamBackendOptions backend_options;
        backend_options.enabled = true;
        backend_options.ready = true;
        backend_options.min_yaw_coverage_ratio_for_good = 0.10;
        DeterministicSlamBackendBinding backend(backend_options);

        RealSensorReplayPort replay(RealSensorReplaySampleLog::valid_log());
        for (int i = 0; i < options_.min_accepted_updates; ++i) {
            const auto snapshot = replay.latest_snapshot(100.0);
            if (!snapshot.has_tof) {
                report.failed.push_back("replay_to_slam_snapshot_missing");
                break;
            }
            report.replay_packet_count++;
            SlamBackendInputFrame input;
            input.timestamp_s = snapshot.tof.timestamp_s;
            input.snapshot = snapshot;
            input.source = "replay_to_slam_backend_regression";
            const auto update = backend.update(input);
            if (update.status == SlamBackendUpdateStatus::Accepted &&
                update.map_updated) {
                report.accepted_update_count++;
            } else {
                report.rejected_update_count++;
            }
        }
        report.final_quality = backend.latest_quality(100.0);
        if (options_.require_valid_replay_updates_map &&
            report.accepted_update_count >= options_.min_accepted_updates &&
            report.final_quality.valid_scan_count >= options_.min_accepted_updates) {
            report.passed.push_back("valid_replay_updates_map");
        } else if (options_.require_valid_replay_updates_map) {
            report.failed.push_back("valid_replay_failed_to_update_map");
        }

        if (options_.require_invalid_replay_rejected && invalid_logs_rejected()) {
            report.passed.push_back("invalid_replay_rejected");
        } else if (options_.require_invalid_replay_rejected) {
            report.failed.push_back("invalid_replay_updated_map");
        }

        report.ok = report.failed.empty();
        report.summary = report.ok ? "replay_to_slam_backend_regression_passed"
                                   : "replay_to_slam_backend_regression_failed";
        return report;
    }

private:
    bool invalid_logs_rejected() const {
        return invalid_log_rejected(RealSensorReplaySampleLog::invalid_latency_log()) &&
               invalid_log_rejected(RealSensorReplaySampleLog::sync_too_large_log());
    }

    bool invalid_log_rejected(const RealSensorReplayLog &log) const {
        DeterministicSlamBackendOptions backend_options;
        backend_options.enabled = true;
        backend_options.ready = true;
        DeterministicSlamBackendBinding backend(backend_options);
        RealSensorReplayPort replay(log);
        const auto snapshot = replay.latest_snapshot(100.0);
        if (!snapshot.has_tof) {
            return true;
        }
        SlamBackendInputFrame input;
        input.timestamp_s = snapshot.tof.timestamp_s;
        input.snapshot = snapshot;
        input.source = "replay_to_slam_backend_invalid_case";
        const auto update = backend.update(input);
        return update.status != SlamBackendUpdateStatus::Accepted ||
               !update.map_updated;
    }

    ReplayToSlamBackendRegressionOptions options_;
};

} // namespace robot_slamd
