#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct RealSensorReplayAcceptanceRunnerOptions {
    bool require_valid_log_pass = true;
    bool require_invalid_latency_fail = true;
    bool require_sync_too_large_fail = true;
    bool require_port_ready = true;
};

struct RealSensorReplayAcceptanceRunnerResult {
    bool ok = false;
    RealSensorReplayResult valid_log_result;
    RealSensorReplayResult invalid_latency_result;
    RealSensorReplayResult sync_too_large_result;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class RealSensorReplayAcceptanceRunner {
public:
    RealSensorReplayAcceptanceRunner() = default;

    explicit RealSensorReplayAcceptanceRunner(
        RealSensorReplayAcceptanceRunnerOptions options)
        : options_(options) {}

    RealSensorReplayAcceptanceRunnerResult run_once() const {
        RealSensorReplayAcceptanceRunnerResult result;

        // Valid log must parse, build snapshots, and advance through packets.
        RealSensorReplayPort valid_port(RealSensorReplaySampleLog::valid_log());
        if (options_.require_port_ready && !valid_port.ready()) {
            result.failed.push_back("real_sensor_replay_valid_port_not_ready");
        }
        for (int i = 0; i < 3; ++i) {
            valid_port.latest_snapshot(100.0);
        }
        result.valid_log_result = valid_port.result();
        if (options_.require_valid_log_pass &&
            result.valid_log_result.accepted_packet_count >= 3 &&
            result.valid_log_result.rejected_packet_count == 0) {
            result.passed.push_back("real_sensor_replay_valid_log_passed");
        } else if (options_.require_valid_log_pass) {
            result.failed.push_back("real_sensor_replay_valid_log_failed");
        }

        // Bad latency log must be rejected by the hardened request timing check.
        RealSensorReplayPort latency_port(
            RealSensorReplaySampleLog::invalid_latency_log());
        latency_port.latest_snapshot(100.0);
        result.invalid_latency_result = latency_port.result();
        if (options_.require_invalid_latency_fail &&
            result.invalid_latency_result.rejected_packet_count > 0) {
            result.passed.push_back("real_sensor_replay_invalid_latency_failed");
        } else if (options_.require_invalid_latency_fail) {
            result.failed.push_back("real_sensor_replay_invalid_latency_did_not_fail");
        }

        // Bad sync log must be rejected before it reaches live code paths.
        RealSensorReplayPort sync_port(
            RealSensorReplaySampleLog::sync_too_large_log());
        sync_port.latest_snapshot(100.15);
        result.sync_too_large_result = sync_port.result();
        if (options_.require_sync_too_large_fail &&
            result.sync_too_large_result.rejected_packet_count > 0) {
            result.passed.push_back("real_sensor_replay_sync_too_large_failed");
        } else if (options_.require_sync_too_large_fail) {
            result.failed.push_back("real_sensor_replay_sync_too_large_did_not_fail");
        }

        result.ok = result.failed.empty();
        result.summary = result.ok ? "real_sensor_replay_acceptance_passed"
                                   : "real_sensor_replay_acceptance_failed";
        return result;
    }

private:
    RealSensorReplayAcceptanceRunnerOptions options_;
};

} // namespace robot_slamd
