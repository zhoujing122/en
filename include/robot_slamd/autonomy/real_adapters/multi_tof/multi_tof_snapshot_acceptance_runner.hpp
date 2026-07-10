#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_builder.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_sample_data.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct MultiTofSnapshotAcceptanceReport {
    bool ok = false;
    MultiTofSnapshotBuildResult valid_result;
    MultiTofSnapshotBuildResult degraded_result;
    int case_count = 0;
    int passed_case_count = 0;
    int failed_case_count = 0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class MultiTofSnapshotAcceptanceRunner {
public:
    MultiTofSnapshotAcceptanceReport run_once() const {
        MultiTofSnapshotAcceptanceReport report;

        MultiTofSyncOptions strict_sync;
        strict_sync.enabled = true;
        MultiTofSnapshotBuildOptions strict_build;
        strict_build.enabled = true;
        MultiTofSnapshotBuilder strict_builder({}, strict_sync, strict_build);

        run_case("valid_synced_packet",
                 MultiTofSnapshotSampleData::valid_synced_packet(),
                 true,
                 strict_builder,
                 report,
                 true);
        run_case("invalid_sync_packet",
                 MultiTofSnapshotSampleData::invalid_sync_packet(),
                 false,
                 strict_builder,
                 report,
                 false);
        run_case("missing_front_legacy_primary_packet",
                 MultiTofSnapshotSampleData::missing_front_packet(),
                 false,
                 strict_builder,
                 report,
                 false);

        MultiTofContractOptions degraded_contract;
        degraded_contract.require_left = false;
        degraded_contract.min_required_tof_count = 2;
        MultiTofSyncOptions degraded_sync;
        degraded_sync.enabled = true;
        degraded_sync.degradation_mode =
            MultiTofDegradationMode::AllowMissingOne;
        degraded_sync.min_required_tof_count = 2;
        MultiTofSnapshotBuildOptions degraded_build;
        degraded_build.enabled = true;
        degraded_build.min_required_tof_count = 2;
        MultiTofSnapshotBuilder degraded_builder(
            degraded_contract,
            degraded_sync,
            degraded_build);
        run_case("missing_left_degraded_packet",
                 MultiTofSnapshotSampleData::missing_left_degraded_packet(),
                 true,
                 degraded_builder,
                 report,
                 false,
                 true);

        report.ok = report.failed.empty();
        report.summary = report.ok ? "multi_tof_snapshot_acceptance_passed"
                                   : "multi_tof_snapshot_acceptance_failed";
        return report;
    }

private:
    void run_case(
        const std::string &name,
        const MultiTofRawPacket &packet,
        bool expect_pass,
        const MultiTofSnapshotBuilder &builder,
        MultiTofSnapshotAcceptanceReport &report,
        bool save_valid,
        bool save_degraded = false) const {
        report.case_count++;
        const auto result = builder.build(packet, 100.200);
        if (save_valid) {
            report.valid_result = result;
        }
        if (save_degraded) {
            report.degraded_result = result;
        }
        if (result.ok == expect_pass) {
            report.passed_case_count++;
            report.passed.push_back(name);
        } else {
            report.failed_case_count++;
            report.failed.push_back(name);
        }
        report.warnings.insert(report.warnings.end(),
                               result.warnings.begin(),
                               result.warnings.end());
    }
};

} // namespace robot_slamd
