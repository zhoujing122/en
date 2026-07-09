#pragma once

#include "robot_slamd/autonomy/autonomous_slam_coordinator.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_pipeline_types.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/deterministic_slam_backend_types.hpp"

#include <string>

namespace robot_slamd {

struct FullAutonomousSlamScenario {
    FullAutonomousSlamPipelineScenarioKind kind =
        FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes;
    RealSensorReplayLog replay_log;
    DeterministicSlamBackendOptions backend_options;
    AutonomousSlamCoordinatorOptions coordinator_options;
    FullAutonomousSlamPipelineOptions pipeline_options;
    bool reject_motion = false;
    std::string name;
};

class FullAutonomousSlamScenarioBuilder {
public:
    FullAutonomousSlamScenario build(
        FullAutonomousSlamPipelineScenarioKind kind) const {
        FullAutonomousSlamScenario scenario;
        scenario.kind = kind;
        scenario.name = to_string(kind);
        scenario.replay_log = RealSensorReplaySampleLog::valid_log();
        scenario.backend_options = default_backend_options();
        scenario.coordinator_options = default_coordinator_options();
        scenario.pipeline_options = default_pipeline_options();

        if (kind == FullAutonomousSlamPipelineScenarioKind::CompletesWithoutActiveScan) {
            scenario.backend_options.min_valid_scan_count_for_good = 1;
            scenario.backend_options.min_coverage_ratio_for_good = 0.5;
            scenario.backend_options.min_yaw_coverage_ratio_for_good = 0.0;
            scenario.pipeline_options.min_backend_accepted_updates = 1;
            return scenario;
        }

        if (kind == FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes) {
            scenario.backend_options.min_valid_scan_count_for_good = 2;
            scenario.backend_options.min_coverage_ratio_for_good = 0.75;
            scenario.backend_options.min_yaw_coverage_ratio_for_good = 0.0;
            scenario.pipeline_options.min_backend_accepted_updates = 2;
            return scenario;
        }

        if (kind == FullAutonomousSlamPipelineScenarioKind::SensorReplayInvalid) {
            scenario.replay_log = RealSensorReplaySampleLog::invalid_latency_log();
            scenario.pipeline_options.min_backend_accepted_updates = 1;
            return scenario;
        }

        if (kind == FullAutonomousSlamPipelineScenarioKind::BackendRejected) {
            scenario.backend_options.ready = false;
            scenario.pipeline_options.min_backend_accepted_updates = 1;
            return scenario;
        }

        if (kind == FullAutonomousSlamPipelineScenarioKind::MotionRejected) {
            scenario.backend_options.min_valid_scan_count_for_good = 2;
            scenario.backend_options.min_coverage_ratio_for_good = 0.75;
            scenario.backend_options.min_yaw_coverage_ratio_for_good = 0.0;
            scenario.pipeline_options.min_backend_accepted_updates = 2;
            scenario.reject_motion = true;
            return scenario;
        }

        if (kind == FullAutonomousSlamPipelineScenarioKind::MapQualityStuck) {
            scenario.backend_options.min_valid_scan_count_for_good = 100;
            scenario.backend_options.min_coverage_ratio_for_good = 1.0;
            scenario.backend_options.min_yaw_coverage_ratio_for_good = 1.0;
            scenario.pipeline_options.max_steps = 8;
            scenario.pipeline_options.max_active_scan_commands = 2;
            scenario.pipeline_options.min_backend_accepted_updates = 1;
            return scenario;
        }

        return scenario;
    }

private:
    static DeterministicSlamBackendOptions default_backend_options() {
        DeterministicSlamBackendOptions options;
        options.enabled = true;
        options.ready = true;
        options.require_tof = true;
        options.require_imu_or_wheel = true;
        options.allow_save_map = false;
        options.min_valid_ranges = 3;
        options.min_valid_scan_count_for_good = 2;
        options.min_valid_range_ratio = 0.30;
        options.min_coverage_ratio_for_good = 0.75;
        options.min_yaw_coverage_ratio_for_good = 0.0;
        options.keyframe_yaw_delta_rad = 0.15;
        options.min_range_m = 0.02;
        options.max_range_m = 8.0;
        options.max_update_latency_s = 0.50;
        options.assumed_scan_yaw_span_rad = 0.52;
        options.yaw_bin_size_rad = 0.26;
        return options;
    }

    static AutonomousSlamCoordinatorOptions default_coordinator_options() {
        AutonomousSlamCoordinatorOptions options;
        options.enabled = true;
        options.max_iterations = 100;
        options.sensor_timeout_s = 0.50;
        options.motion_settle_timeout_s = 1.00;
        options.active_scan_speed = 0.05;
        options.active_scan_duration_s = 0.50;
        options.max_active_scan_commands = 6;
        options.prefer_left_turn = true;
        return options;
    }

    static FullAutonomousSlamPipelineOptions default_pipeline_options() {
        FullAutonomousSlamPipelineOptions options;
        options.enabled = true;
        options.run_on_startup = false;
        options.max_steps = 20;
        options.max_active_scan_commands = 6;
        options.min_backend_accepted_updates = 2;
        options.require_completion = true;
        options.require_shadow_motion_only = true;
        options.require_no_forward_backward = true;
        options.require_map_quality_good = true;
        options.motion_settle_s = 0.20;
        return options;
    }
};

} // namespace robot_slamd
