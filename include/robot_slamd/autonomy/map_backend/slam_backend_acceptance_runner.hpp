#pragma once

#include "robot_slamd/autonomy/map_backend/slam_backend_binding.hpp"
#include "robot_slamd/autonomy/map_backend/slam_backend_contract_checker.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

struct SlamBackendAcceptanceRunnerOptions {
    double start_time_s = 100.0;
    bool require_ready = true;
    bool require_update_accepted = true;
    bool require_quality_valid = true;
    bool require_save_map = false;
};

struct SlamBackendAcceptanceRunnerResult {
    bool ok = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    SlamBackendUpdateResult input_check;
    SlamBackendUpdateResult update_check;
    RobotSlamMapQuality final_quality;
};

class SlamBackendAcceptanceRunner {
public:
    SlamBackendAcceptanceRunner(std::shared_ptr<SlamBackendBinding> backend,
                                SlamBackendContractChecker checker = {},
                                SlamBackendAcceptanceRunnerOptions options = {})
        : backend_(std::move(backend)),
          checker_(checker),
          options_(options) {}

    SlamBackendAcceptanceRunnerResult run_once(
        const RobotSlamSensorSnapshot &snapshot) {
        SlamBackendAcceptanceRunnerResult result;

        // Backend readiness.
        if (!backend_) {
            fail(result, "slam_backend_missing");
            return result;
        }
        if (options_.require_ready && !backend_->ready()) {
            fail(result, "slam_backend_not_ready");
            return result;
        }
        pass(result, "slam_backend_ready");

        // Input frame contract.
        SlamBackendInputFrame input;
        input.timestamp_s = options_.start_time_s;
        input.snapshot = snapshot;
        input.source = "slam_backend_acceptance";
        result.input_check = checker_.check_input_frame(input, options_.start_time_s);
        if (result.input_check.status != SlamBackendUpdateStatus::Accepted) {
            fail(result, "slam_backend_input_contract_failed");
            return result;
        }
        pass(result, "slam_backend_input_contract_passed");

        // Backend update contract.
        const auto update = backend_->update(input);
        result.update_check = checker_.check_update_result(update);
        if (options_.require_update_accepted &&
            result.update_check.status != SlamBackendUpdateStatus::Accepted) {
            fail(result, "slam_backend_update_contract_failed");
            return result;
        }
        pass(result, "slam_backend_update_contract_passed");

        // Quality contract.
        result.final_quality = backend_->latest_quality(options_.start_time_s);
        const auto quality_check = checker_.check_quality(result.final_quality);
        if (options_.require_quality_valid &&
            quality_check.status != SlamBackendUpdateStatus::Accepted) {
            fail(result, "slam_backend_quality_contract_failed");
            return result;
        }
        pass(result, "slam_backend_quality_contract_passed");

        // Optional save contract.
        if (options_.require_save_map) {
            const auto save = backend_->save_map("/tmp/slam_backend_acceptance_map");
            if (!save.ok) {
                fail(result, "slam_backend_save_failed");
                return result;
            }
            pass(result, "slam_backend_save_passed");
        }

        result.ok = result.failed.empty();
        return result;
    }

private:
    static void pass(SlamBackendAcceptanceRunnerResult &result,
                     const std::string &case_name) {
        result.passed.push_back(case_name);
    }

    static void fail(SlamBackendAcceptanceRunnerResult &result,
                     const std::string &case_name) {
        result.failed.push_back(case_name);
        result.ok = false;
    }

    std::shared_ptr<SlamBackendBinding> backend_;
    SlamBackendContractChecker checker_;
    SlamBackendAcceptanceRunnerOptions options_;
};

} // namespace robot_slamd
