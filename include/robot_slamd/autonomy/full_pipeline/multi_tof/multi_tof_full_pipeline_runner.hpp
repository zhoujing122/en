#pragma once

#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_runner.hpp"
#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_scenario_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_sensor_port.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace robot_slamd {

class MultiTofFullPipelineRunner {
public:
    MultiTofFullPipelineReport run_once(
        const MultiTofFullPipelineScenario &scenario) const {
        if (scenario.kind == MultiTofFullPipelineScenarioKind::NullSensorPort) {
            FullAutonomousSlamRunner runner;
            MultiTofFullPipelineReport report;
            report.scenario = scenario.kind;
            report.full_pipeline = runner.run_once_with_sensor(
                scenario.full_pipeline_scenario,
                nullptr);
            return finalize_report(report, nullptr);
        }

        auto replay = MultiTofReplayPort(
            scenario.records,
            scenario.contract_options,
            scenario.sync_options,
            scenario.snapshot_options,
            scenario.replay_options);
        auto sensor_port = std::make_shared<MultiTofReplaySensorPort>(
            std::move(replay));

        if (scenario.kind == MultiTofFullPipelineScenarioKind::NotReadySensorPort) {
            MultiTofReplayOptions disabled = scenario.replay_options;
            disabled.enabled = false;
            auto not_ready = std::make_shared<MultiTofReplaySensorPort>(
                MultiTofReplayPort(scenario.records,
                                   scenario.contract_options,
                                   scenario.sync_options,
                                   scenario.snapshot_options,
                                   disabled));
            FullAutonomousSlamRunner runner;
            MultiTofFullPipelineReport report;
            report.scenario = scenario.kind;
            report.full_pipeline = runner.run_once_with_sensor(
                scenario.full_pipeline_scenario,
                not_ready);
            return finalize_report(report, not_ready);
        }

        FullAutonomousSlamRunner runner;
        MultiTofFullPipelineReport report;
        report.scenario = scenario.kind;
        report.full_pipeline = runner.run_once_with_sensor(
            scenario.full_pipeline_scenario,
            sensor_port);
        return finalize_report(report, sensor_port);
    }

private:
    static MultiTofFullPipelineReport finalize_report(
        MultiTofFullPipelineReport report,
        const std::shared_ptr<MultiTofReplaySensorPort> &sensor_port) {
        report.pipeline_report = report.full_pipeline;
        report.fault = to_multi_tof_full_pipeline_fault(report.full_pipeline.fault);
        report.ok = report.full_pipeline.ok;

        report.multi_tof_snapshot_count = report.full_pipeline.multi_tof_snapshot_count;
        report.degraded_snapshot_count =
            report.full_pipeline.degraded_multi_tof_snapshot_count;
        report.front_present_count = report.full_pipeline.snapshots_with_front;
        report.left_present_count = report.full_pipeline.snapshots_with_left;
        report.right_present_count = report.full_pipeline.snapshots_with_right;
        report.legacy_projection_count =
            report.full_pipeline.legacy_tof_snapshot_count;
        report.multi_tof_reached_runner = report.full_pipeline.saw_multi_tof_snapshot;
        report.multi_tof_reached_coordinator = report.full_pipeline.saw_multi_tof_snapshot;
        report.legacy_projection_reached_backend =
            report.full_pipeline.backend_accepted_update_count > 0 &&
            report.full_pipeline.saw_legacy_scalar_projection;
        report.phase_aware_sensor_consumption_verified =
            report.full_pipeline.sensor_skipped_count > 0;

        report.fake_motion_only = true;
        report.native_multi_tof_backend_consumption = false;
        report.deterministic_backend_uses_legacy_tof = true;
        report.multi_tof_real_mapping_fusion = false;
        report.real_hardware_accessed = false;
        report.real_motion_attempted = false;
        report.real_map_write_attempted = false;
        report.real_pose_writeback_attempted = false;
        report.forward_seen = false;
        report.backward_seen = false;
        report.stale_snapshot_reused = false;
        report.warnings.push_back(
            "multi_tof_pipeline_uses_legacy_scalar_projection_for_deterministic_backend");

        if (sensor_port) {
            report.replay_read_call_count = sensor_port->read_call_count();
            report.successful_snapshot_count =
                sensor_port->successful_snapshot_count();
            report.rejected_snapshot_count =
                sensor_port->rejected_snapshot_count();
            report.end_of_replay_return_count =
                sensor_port->end_of_replay_return_count();
            report.consumed_packet_count = sensor_port->consumed_packet_count();
            report.has_last_success_snapshot =
                sensor_port->has_last_success_snapshot();
            if (report.has_last_success_snapshot) {
                report.last_success_snapshot = sensor_port->last_success_snapshot();
                collect_snapshot_facts(report, report.last_success_snapshot);
            }
            if (sensor_port->has_last_read_result()) {
                report.last_replay_result = sensor_port->last_read_result();
                report.last_replay_status = report.last_replay_result.status;
                report.last_replay_fault = report.last_replay_result.fault;
                report.last_replay_message = report.last_replay_result.message;
                if (!report.last_replay_result.ok &&
                    report.last_replay_fault != MultiTofReplayFault::EndOfReplay) {
                    report.failed.push_back(report.last_replay_message);
                }
            }
        }

        report.scalar_tof_codec_used = true;
        if (report.ok) {
            report.passed.push_back("multi_tof_full_pipeline_completed");
            report.passed.push_back("scalar_three_tof_reached_runner");
            report.passed.push_back("scalar_three_tof_reached_coordinator");
            report.passed.push_back("legacy_scalar_projection_reached_backend");
            report.passed.push_back("fake_motion_only");
            report.summary = "multi_tof_full_pipeline_passed";
        } else if (report.last_replay_fault == MultiTofReplayFault::EndOfReplay) {
            report.failed.push_back("multi_tof_replay_end_of_replay");
            report.summary = "multi_tof_replay_end_of_replay";
        } else if (!report.last_replay_message.empty()) {
            report.summary = report.last_replay_message;
        } else {
            report.failed.push_back(report.full_pipeline.summary);
            report.summary = "multi_tof_full_pipeline_failed";
        }
        return report;
    }

    static void collect_snapshot_facts(
        MultiTofFullPipelineReport &report,
        const RobotSlamSensorSnapshot &snapshot) {
        if (snapshot.has_tof) {
            report.legacy_projection_count = std::max(report.legacy_projection_count, 1);
            report.maximum_legacy_projection_range_count = std::max(
                report.maximum_legacy_projection_range_count,
                static_cast<int>(snapshot.tof.ranges_m.size()));
        }
        if (!snapshot.has_multi_tof) {
            return;
        }
        if (snapshot.multi_tof.has_front) {
            report.front_present_count = std::max(report.front_present_count, 1);
            if (snapshot.multi_tof.front.usable_for_mapping) {
                report.front_mapping_usable_count++;
            }
        }
        if (snapshot.multi_tof.has_left) {
            report.left_present_count = std::max(report.left_present_count, 1);
            if (snapshot.multi_tof.left.usable_for_mapping) {
                report.left_mapping_usable_count++;
            }
        }
        if (snapshot.multi_tof.has_right) {
            report.right_present_count = std::max(report.right_present_count, 1);
            if (snapshot.multi_tof.right.usable_for_mapping) {
                report.right_mapping_usable_count++;
            }
        }
        report.echo_tag_preserved =
            snapshot.multi_tof.front.echo_tag_u48 != 0 ||
            snapshot.multi_tof.left.echo_tag_u48 != 0 ||
            snapshot.multi_tof.right.echo_tag_u48 != 0;
        report.confidence_preserved =
            snapshot.multi_tof.front.confidence > 0 ||
            snapshot.multi_tof.left.confidence > 0 ||
            snapshot.multi_tof.right.confidence > 0;
        report.effective_timestamp_preserved =
            std::isfinite(snapshot.multi_tof.front.effective_timestamp_s) &&
            snapshot.multi_tof.front.effective_timestamp_s > 0.0;
        report.synchronized_time_verified =
            std::isfinite(snapshot.multi_tof.synchronized_time_s) &&
            snapshot.multi_tof.synchronized_time_s > 0.0;
    }
};

} // namespace robot_slamd
