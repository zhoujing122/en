#pragma once

#include "robot_slamd/autonomy/fake_map/fake_map_storage.hpp"
#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_binding.hpp"
#include "robot_slamd/autonomy/full_pipeline/full_autonomous_slam_runner.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_data/real_sensor_snapshot_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_port.hpp"
#include "robot_slamd/autonomy/real_adapters/sensor_replay/real_sensor_replay_sample_log.hpp"

#include <memory>
#include <string>
#include <vector>

namespace robot_slamd {

struct FakeMapRelocalizationRunnerOptions {
    bool enabled = false;
    bool require_pipeline_map_artifact = true;
    bool require_relocalization_success = true;
    bool require_no_pose_writeback = true;
};

struct FakeMapRelocalizationRunnerReport {
    bool ok = false;
    FakeRelocalizationResult relocalization_result;
    FullAutonomousSlamPipelineReport pipeline_report;
    int replay_snapshot_count = 0;
    bool map_loaded = false;
    bool pose_writeback_attempted = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class FakeMapRelocalizationRunner {
public:
    explicit FakeMapRelocalizationRunner(
        FakeMapRelocalizationRunnerOptions options = {})
        : options_(options) {}

    FakeMapRelocalizationRunnerReport run_once() const {
        FakeMapRelocalizationRunnerReport report;
        if (!options_.enabled) {
            return fail(report, "fake_map_relocalization_runner_disabled");
        }

        FullAutonomousSlamScenarioBuilder scenario_builder;
        FullAutonomousSlamRunner pipeline_runner;
        report.pipeline_report = pipeline_runner.run_once(
            scenario_builder.build(
                FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes));
        if (!report.pipeline_report.ok) {
            return fail(report, "fake_map_relocalization_pipeline_failed");
        }
        if (options_.require_pipeline_map_artifact &&
            (!report.pipeline_report.fake_map_built ||
             !report.pipeline_report.fake_map_saved ||
             report.pipeline_report.fake_map_artifact.metadata.map_id.empty())) {
            return fail(report, "fake_map_relocalization_missing_artifact");
        }

        FakeMapSaveOptions save_options;
        save_options.enabled = true;
        save_options.allow_overwrite = false;
        save_options.require_quality_good = true;
        save_options.require_completed_pipeline = true;
        FakeMapLoadOptions load_options;
        load_options.enabled = true;
        load_options.require_valid_artifact = true;
        FakeMapStorage storage(save_options, load_options);
        const auto saved = storage.save_artifact(
            report.pipeline_report.fake_map_artifact);
        if (!saved.ok) {
            return fail(report, saved.summary);
        }
        const auto loaded = storage.load_artifact(
            report.pipeline_report.fake_map_artifact.metadata.map_id);
        if (!loaded.ok) {
            return fail(report, loaded.summary);
        }
        report.map_loaded = true;
        report.passed.push_back("fake_map_relocalization_map_loaded");

        auto replay = make_replay_port();
        if (!replay.ready()) {
            return fail(report, "fake_map_relocalization_replay_not_ready");
        }
        const auto snapshot = replay.latest_snapshot(100.0);
        if (!snapshot.has_tof && !snapshot.has_imu && !snapshot.has_wheel) {
            return fail(report, "fake_map_relocalization_snapshot_missing");
        }
        report.replay_snapshot_count = 1;

        FakeRelocalizationOptions relocalization_options;
        relocalization_options.enabled = true;
        relocalization_options.allow_pose_writeback = false;
        relocalization_options.require_map_quality_good = true;
        relocalization_options.require_tof = true;
        relocalization_options.min_valid_ranges = 3;
        relocalization_options.min_confidence = 0.70;
        relocalization_options.high_confidence_threshold = 0.85;
        relocalization_options.min_map_coverage_ratio = 0.60;
        relocalization_options.min_map_yaw_coverage_ratio = 0.0;
        FakeRelocalizationBinding binding(relocalization_options);
        report.relocalization_result = binding.relocalize(
            loaded.artifact,
            snapshot);
        report.pose_writeback_attempted = false;
        if (options_.require_relocalization_success &&
            !report.relocalization_result.ok) {
            return fail(report, report.relocalization_result.summary);
        }
        if (options_.require_no_pose_writeback &&
            report.pose_writeback_attempted) {
            return fail(report, "fake_map_relocalization_pose_writeback_attempted");
        }

        report.ok = true;
        report.passed.push_back("fake_map_relocalization_pipeline_passed");
        report.passed.push_back("fake_map_relocalization_no_pose_writeback");
        report.summary = "fake_map_relocalization_pipeline_passed";
        return report;
    }

private:
    static RealSensorReplayPort make_replay_port() {
        RealSensorReplayOptions replay_options;
        replay_options.enabled = true;
        replay_options.loop = false;
        replay_options.fail_on_contract_error = true;
        replay_options.require_non_empty_log = true;
        replay_options.time_mode = RealSensorReplayTimeMode::RecordPacketTime;
        replay_options.reject_invalid_records = true;
        replay_options.require_packet_records = true;
        replay_options.preserve_parse_errors = true;
        replay_options.max_records_per_run = 10000;
        return RealSensorReplayPort(
            RealSensorReplaySampleLog::valid_log(),
            RealSensorSnapshotBuilder{},
            replay_options);
    }

    static FakeMapRelocalizationRunnerReport fail(
        FakeMapRelocalizationRunnerReport report,
        const std::string &summary) {
        report.ok = false;
        report.failed.push_back(summary);
        report.summary = summary;
        return report;
    }

    FakeMapRelocalizationRunnerOptions options_;
};

} // namespace robot_slamd
