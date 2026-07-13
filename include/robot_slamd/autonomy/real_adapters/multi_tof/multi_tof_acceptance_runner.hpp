#pragma once

#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_contract_checker.hpp"
#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sample_data.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct MultiTofAcceptanceReport {
    bool ok = false;
    MultiTofContractResult valid_result;
    int case_count = 0;
    int passed_case_count = 0;
    int failed_case_count = 0;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

class MultiTofAcceptanceRunner {
public:
    explicit MultiTofAcceptanceRunner(
        MultiTofContractOptions options = {})
        : options_(options) {}

    MultiTofAcceptanceReport run_once() const {
        MultiTofAcceptanceReport report;
        MultiTofContractChecker checker(options_);
        run_case("valid_three_tof_packet",
                 MultiTofSampleData::valid_three_tof_packet(),
                 true,
                 checker,
                 report);
        run_case("missing_left_packet",
                 MultiTofSampleData::missing_left_packet(),
                 false,
                 checker,
                 report);
        run_case("duplicate_frame_id_packet",
                 MultiTofSampleData::duplicate_frame_id_packet(),
                 false,
                 checker,
                 report);
        run_case("duplicate_mount_id_packet",
                 MultiTofSampleData::duplicate_mount_id_packet(),
                 false,
                 checker,
                 report);
        run_case("invalid_mount_yaw_packet",
                 MultiTofSampleData::invalid_mount_yaw_packet(),
                 false,
                 checker,
                 report);
        run_case("invalid_distance_packet",
                 MultiTofSampleData::invalid_distance_packet(),
                 false,
                 checker,
                 report);
        run_case("latency_too_high_packet",
                 MultiTofSampleData::latency_too_high_packet(),
                 false,
                 checker,
                 report);
        report.ok = report.failed.empty();
        report.summary = report.ok ? "multi_tof_acceptance_passed"
                                   : "multi_tof_acceptance_failed";
        return report;
    }

private:
    MultiTofContractOptions options_;

    void run_case(
        const std::string &name,
        const MultiTofRawPacket &packet,
        bool expect_pass,
        const MultiTofContractChecker &checker,
        MultiTofAcceptanceReport &report) const {
        report.case_count++;
        const auto result = checker.check_packet(packet, 100.0);
        if (expect_pass) {
            report.valid_result = result;
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
