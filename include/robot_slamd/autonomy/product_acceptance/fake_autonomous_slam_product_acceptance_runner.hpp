#pragma once

#include "robot_slamd/autonomy/product_acceptance/fake_autonomous_slam_product_acceptance_types.hpp"
#include "robot_slamd/autonomy/product_acceptance/fake_to_real_adapter_replacement_manifest.hpp"

#include <algorithm>
#include <string>

namespace robot_slamd {

class FakeAutonomousSlamProductAcceptanceRunner {
public:
    explicit FakeAutonomousSlamProductAcceptanceRunner(
        FakeAutonomousSlamProductAcceptanceOptions options = {})
        : options_(options) {}

    FakeAutonomousSlamProductAcceptanceReport run_once() const {
        FakeAutonomousSlamProductAcceptanceReport report;
        if (!options_.enabled) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::MappingPipelineFailed,
                "fake_product_acceptance_disabled");
        }

        report.status = FakeAutonomousSlamProductAcceptanceStatus::RunningMappingPipeline;
        FullAutonomousSlamScenarioBuilder scenario_builder;
        const auto scenario_kind = options_.force_mapping_failure
                                       ? FullAutonomousSlamPipelineScenarioKind::SensorReplayInvalid
                                       : FullAutonomousSlamPipelineScenarioKind::RequiresActiveScanThenCompletes;
        FullAutonomousSlamRunner mapping_runner;
        report.mapping_report = mapping_runner.run_once(
            scenario_builder.build(scenario_kind));
        if (options_.force_fake_map_missing) {
            report.mapping_report.fake_map_built = false;
            report.mapping_report.fake_map_saved = false;
        }
        if (options_.require_mapping_pipeline_success &&
            !report.mapping_report.ok) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::MappingPipelineFailed,
                "fake_product_acceptance_mapping_failed");
        }
        if (options_.require_fake_map_saved &&
            (!report.mapping_report.fake_map_built ||
             !report.mapping_report.fake_map_saved)) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::FakeMapMissing,
                "fake_product_acceptance_fake_map_missing");
        }
        if (options_.require_no_forward_backward &&
            report.mapping_report.failed.end() != std::find(
                report.mapping_report.failed.begin(),
                report.mapping_report.failed.end(),
                "full_pipeline_forward_backward_seen")) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::ForwardBackwardCommandObserved,
                "fake_product_acceptance_forward_backward_observed");
        }

        report.status = FakeAutonomousSlamProductAcceptanceStatus::RunningRelocalization;
        FakeMapRelocalizationRunnerOptions relocalization_options;
        relocalization_options.enabled = true;
        if (options_.force_relocalization_failure) {
            relocalization_options.use_strict_default_relocalization_options = false;
            relocalization_options.relocalization_options =
                strict_fake_map_relocalization_options();
            relocalization_options.relocalization_options.min_confidence = 0.99;
        }
        FakeMapRelocalizationRunner relocalization_runner(relocalization_options);
        report.relocalization_report = relocalization_runner.run_once();
        if (options_.require_relocalization_success &&
            !report.relocalization_report.ok) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::RelocalizationFailed,
                "fake_product_acceptance_relocalization_failed");
        }
        if (options_.require_no_pose_writeback &&
            report.relocalization_report.pose_writeback_attempted) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::PoseWritebackAttempted,
                "fake_product_acceptance_pose_writeback_attempted");
        }

        report.status = FakeAutonomousSlamProductAcceptanceStatus::CheckingReadiness;
        FakeRelocalizationReadinessOptions readiness_options;
        readiness_options.enabled = true;
        readiness_options.require_binding_ready = true;
        readiness_options.require_no_pose_writeback = true;
        readiness_options.require_map_quality_good = true;
        readiness_options.min_confidence = options_.force_readiness_blocked
                                               ? 0.99
                                               : 0.70;
        readiness_options.min_map_coverage_ratio = 0.60;
        readiness_options.min_map_yaw_coverage_ratio = 0.30;
        FakeRelocalizationReadinessGate readiness_gate(readiness_options);
        report.readiness_result = readiness_gate.check(
            report.relocalization_report.relocalization_result,
            report.relocalization_report.pose_writeback_attempted);
        if (options_.require_relocalization_readiness &&
            !report.readiness_result.ok) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::RelocalizationReadinessBlocked,
                "fake_product_acceptance_readiness_blocked");
        }

        FakeToRealAdapterReplacementManifestBuilder manifest_builder;
        auto manifest = manifest_builder.build_default_manifest();
        if (options_.force_manifest_invalid) {
            manifest.valid = false;
            manifest.items.clear();
            manifest.summary = "fake_to_real_adapter_manifest_invalid";
        }
        report.adapter_manifest_text = manifest_builder.write_text(manifest);
        if (options_.require_adapter_manifest && !manifest.valid) {
            return reject(
                report,
                FakeAutonomousSlamProductAcceptanceBlockReason::AdapterManifestInvalid,
                "fake_product_acceptance_manifest_invalid");
        }

        report.ok = true;
        report.status = FakeAutonomousSlamProductAcceptanceStatus::Accepted;
        report.block_reason = FakeAutonomousSlamProductAcceptanceBlockReason::None;
        report.passed.push_back("fake_product_acceptance_mapping_passed");
        report.passed.push_back("fake_product_acceptance_relocalization_passed");
        report.passed.push_back("fake_product_acceptance_readiness_passed");
        report.passed.push_back("fake_product_acceptance_manifest_valid");
        report.summary = "fake_autonomous_slam_product_acceptance_passed";
        return report;
    }

private:
    static FakeAutonomousSlamProductAcceptanceReport reject(
        FakeAutonomousSlamProductAcceptanceReport report,
        FakeAutonomousSlamProductAcceptanceBlockReason reason,
        const std::string &summary) {
        report.ok = false;
        report.status = FakeAutonomousSlamProductAcceptanceStatus::Rejected;
        report.block_reason = reason;
        report.failed.push_back(summary);
        report.summary = summary;
        return report;
    }

    FakeAutonomousSlamProductAcceptanceOptions options_;
};

} // namespace robot_slamd
