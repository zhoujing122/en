#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_checker.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_sample_data.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct MultiTofSyncAcceptanceReport {
    bool ok = false;
    MultiTofSyncResult valid_result;
    MultiTofSyncResult degraded_result;
    int case_count = 0;
    int passed_case_count = 0;
    int failed_case_count = 0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class MultiTofSyncAcceptanceRunner {
public:
    MultiTofSyncAcceptanceReport run_once() const {
        MultiTofSyncAcceptanceReport report;
        MultiTofContractOptions strict_contract;
        MultiTofSyncOptions strict_sync;
        strict_sync.enabled = true;
        MultiTofSyncChecker strict_checker(strict_contract, strict_sync);
        run_case("valid_synced_packet",
                 MultiTofSyncSampleData::valid_synced_packet(),
                 true,
                 strict_checker,
                 report,
                 true);
        run_case("front_left_sync_too_large_packet",
                 MultiTofSyncSampleData::front_left_sync_too_large_packet(),
                 false,
                 strict_checker,
                 report,
                 false);
        run_case("front_right_sync_too_large_packet",
                 MultiTofSyncSampleData::front_right_sync_too_large_packet(),
                 false,
                 strict_checker,
                 report,
                 false);
        run_case("left_right_sync_too_large_packet",
                 MultiTofSyncSampleData::left_right_sync_too_large_packet(),
                 false,
                 strict_checker,
                 report,
                 false);
        run_case("imu_sync_too_large_packet",
                 MultiTofSyncSampleData::imu_sync_too_large_packet(),
                 false,
                 strict_checker,
                 report,
                 false);
        run_case("wheel_sync_too_large_packet",
                 MultiTofSyncSampleData::wheel_sync_too_large_packet(),
                 false,
                 strict_checker,
                 report,
                 false);
        run_case("missing_imu_packet",
                 MultiTofSyncSampleData::missing_imu_packet(),
                 false,
                 strict_checker,
                 report,
                 false);
        run_case("missing_wheel_packet",
                 MultiTofSyncSampleData::missing_wheel_packet(),
                 false,
                 strict_checker,
                 report,
                 false);
        run_case("missing_left_require_all_packet",
                 MultiTofSyncSampleData::missing_left_degradable_packet(),
                 false,
                 strict_checker,
                 report,
                 false);

        MultiTofContractOptions degraded_contract;
        degraded_contract.require_left = false;
        degraded_contract.min_required_tof_count = 2;
        MultiTofSyncOptions degraded_sync;
        degraded_sync.enabled = true;
        degraded_sync.degradation_mode = MultiTofDegradationMode::AllowMissingOne;
        degraded_sync.min_required_tof_count = 2;
        MultiTofSyncChecker degraded_checker(degraded_contract, degraded_sync);
        run_case("missing_left_degradable_packet",
                 MultiTofSyncSampleData::missing_left_degradable_packet(),
                 true,
                 degraded_checker,
                 report,
                 false,
                 true);
        report.ok = report.failed.empty();
        report.summary = report.ok ? "multi_tof_sync_acceptance_passed"
                                   : "multi_tof_sync_acceptance_failed";
        return report;
    }

private:
    void run_case(
        const std::string &name,
        const MultiTofRawPacket &packet,
        bool expect_pass,
        const MultiTofSyncChecker &checker,
        MultiTofSyncAcceptanceReport &report,
        bool save_valid,
        bool save_degraded = false) const {
        report.case_count++;
        const auto result = checker.check_packet(packet, 100.200);
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
