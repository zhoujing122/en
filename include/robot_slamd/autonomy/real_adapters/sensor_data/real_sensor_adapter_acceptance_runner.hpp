#pragma once

#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_sample_data.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_snapshot_builder.hpp"

#include <utility>

namespace robot_slamd {

struct RealSensorAdapterAcceptanceRunnerOptions {
    bool require_valid_packet_pass = true;
    bool require_missing_tof_fail = true;
    bool require_missing_imu_and_wheel_fail = true;
    bool require_empty_tof_ranges_fail = true;
    bool require_sync_too_large_fail = true;
    bool require_request_latency_fail = true;
    double start_time_s = 100.0;
};

struct RealSensorAdapterAcceptanceRunnerResult {
    bool ok = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    RealSensorSnapshotBuildResult valid_build_result;
    std::string summary;
};

class RealSensorAdapterAcceptanceRunner {
public:
    RealSensorAdapterAcceptanceRunner() = default;

    RealSensorAdapterAcceptanceRunner(
        RealSensorSnapshotBuilder builder,
        RealSensorAdapterAcceptanceRunnerOptions options = {})
        : builder_(std::move(builder)), options_(options) {}

    RealSensorAdapterAcceptanceRunnerResult run_once() const {
        RealSensorAdapterAcceptanceRunnerResult result;
        const double now_s = options_.start_time_s;

        // Valid packet must build.
        result.valid_build_result =
            builder_.build_snapshot(RealSensorSampleData::valid_packet(now_s),
                                    now_s);
        if (options_.require_valid_packet_pass &&
            !result.valid_build_result.ok) {
            result.failed.push_back("real_sensor_valid_packet_failed");
        } else {
            result.passed.push_back("real_sensor_valid_packet_passed");
        }
        append_warnings(result, result.valid_build_result);

        // Missing ToF must fail.
        check_expected_failure(
            result,
            options_.require_missing_tof_fail,
            RealSensorSampleData::missing_tof_packet(now_s),
            now_s,
            "real_sensor_missing_tof_failed_as_expected",
            "real_sensor_missing_tof_did_not_fail");

        // Missing motion reference must fail.
        check_expected_failure(
            result,
            options_.require_missing_imu_and_wheel_fail,
            RealSensorSampleData::missing_imu_and_wheel_packet(now_s),
            now_s,
            "real_sensor_missing_imu_and_wheel_failed_as_expected",
            "real_sensor_missing_imu_and_wheel_did_not_fail");

        // Empty ToF ranges must fail.
        check_expected_failure(
            result,
            options_.require_empty_tof_ranges_fail,
            RealSensorSampleData::empty_tof_ranges_packet(now_s),
            now_s,
            "real_sensor_empty_tof_ranges_failed_as_expected",
            "real_sensor_empty_tof_ranges_did_not_fail");

        // Sync too large must fail.
        check_expected_failure(
            result,
            options_.require_sync_too_large_fail,
            RealSensorSampleData::sync_too_large_packet(now_s),
            now_s,
            "real_sensor_sync_too_large_failed_as_expected",
            "real_sensor_sync_too_large_did_not_fail");

        // Request latency too high must fail.
        check_expected_failure(
            result,
            options_.require_request_latency_fail,
            RealSensorSampleData::request_latency_too_high_packet(now_s),
            now_s,
            "real_sensor_request_latency_too_high_failed_as_expected",
            "real_sensor_request_latency_too_high_did_not_fail");

        result.ok = result.failed.empty();
        result.summary = result.ok ? "real_sensor_adapter_acceptance_passed"
                                   : "real_sensor_adapter_acceptance_failed";
        return result;
    }

private:
    void check_expected_failure(
        RealSensorAdapterAcceptanceRunnerResult &result,
        bool required,
        const RealSensorRawPacket &packet,
        double now_s,
        const std::string &passed_case,
        const std::string &failed_case) const {
        if (!required) {
            return;
        }

        const auto build = builder_.build_snapshot(packet, now_s);
        if (build.ok) {
            result.failed.push_back(failed_case);
            return;
        }
        result.passed.push_back(passed_case);
        result.warnings.insert(result.warnings.end(),
                               build.warnings.begin(),
                               build.warnings.end());
    }

    static void append_warnings(
        RealSensorAdapterAcceptanceRunnerResult &result,
        const RealSensorSnapshotBuildResult &build) {
        result.warnings.insert(result.warnings.end(),
                               build.warnings.begin(),
                               build.warnings.end());
    }

    RealSensorSnapshotBuilder builder_;
    RealSensorAdapterAcceptanceRunnerOptions options_;
};

} // namespace robot_slamd
