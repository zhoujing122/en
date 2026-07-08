#pragma once

#include "robot_slamd/autonomy/contracts/real_adapter_contract_checker.hpp"
#include "robot_slamd/autonomy/contracts/real_adapter_readiness_checker.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robot_slamd {

struct RealAdapterAcceptanceRunnerOptions {
    double start_time_s = 100.0;
    bool expect_tof = true;
    bool expect_imu_or_wheel = true;
};

struct RealAdapterAcceptanceRunnerResult {
    bool ok = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    RealAdapterContractReport sensor_report;
    RealAdapterContractReport map_report;
    RealAdapterReadinessReport readiness_report;
};

class RealAdapterAcceptanceRunner {
public:
    RealAdapterAcceptanceRunner(std::shared_ptr<RobotSlamSensorPort> sensor_port,
                                std::shared_ptr<RobotSlamMotionPort> motion_port,
                                std::shared_ptr<RobotSlamMapPort> map_port,
                                std::shared_ptr<RobotSlamTimePort> time_port,
                                RealAdapterContractChecker contract_checker = {},
                                RealAdapterReadinessChecker readiness_checker = {},
                                RealAdapterAcceptanceRunnerOptions options = {})
        : sensor_port_(std::move(sensor_port)),
          motion_port_(std::move(motion_port)),
          map_port_(std::move(map_port)),
          time_port_(std::move(time_port)),
          contract_checker_(contract_checker),
          readiness_checker_(readiness_checker),
          options_(options) {}

    RealAdapterAcceptanceRunnerResult run_once() {
        RealAdapterAcceptanceRunnerResult result;
        const double now_s = time_port_ ? time_port_->now_s() : options_.start_time_s;

        // readiness ports
        result.readiness_report = readiness_checker_.check_ports(sensor_port_.get(),
                                                                 motion_port_.get(),
                                                                 map_port_.get(),
                                                                 time_port_.get());
        record(result, "readiness_ports", result.readiness_report.ok);

        // sensor snapshot contract
        if (sensor_port_ && sensor_port_->ready()) {
            const auto snapshot = sensor_port_->latest_snapshot(now_s);
            result.sensor_report = contract_checker_.check_sensor_snapshot(snapshot, now_s);
            record(result, "sensor_snapshot_contract", result.sensor_report.ok);
        } else {
            result.sensor_report.ok = false;
            result.sensor_report.readiness = RealAdapterReadiness::NotReady;
            record(result, "sensor_snapshot_contract", false);
        }

        // map quality contract
        if (map_port_ && map_port_->ready()) {
            result.map_report = contract_checker_.check_map_quality(map_port_->latest_quality(now_s));
            record(result, "map_quality_contract", result.map_report.ok);
        } else {
            result.map_report.ok = false;
            result.map_report.readiness = RealAdapterReadiness::NotReady;
            record(result, "map_quality_contract", false);
        }

        result.ok = result.failed.empty();
        return result;
    }

private:
    static void record(RealAdapterAcceptanceRunnerResult &result,
                       const std::string &case_name,
                       bool ok) {
        if (ok) {
            result.passed.push_back(case_name);
        } else {
            result.failed.push_back(case_name);
        }
    }

    std::shared_ptr<RobotSlamSensorPort> sensor_port_;
    std::shared_ptr<RobotSlamMotionPort> motion_port_;
    std::shared_ptr<RobotSlamMapPort> map_port_;
    std::shared_ptr<RobotSlamTimePort> time_port_;
    RealAdapterContractChecker contract_checker_;
    RealAdapterReadinessChecker readiness_checker_;
    RealAdapterAcceptanceRunnerOptions options_;
};

} // namespace robot_slamd
