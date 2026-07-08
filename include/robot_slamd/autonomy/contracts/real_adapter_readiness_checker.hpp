#pragma once

#include "robot_slamd/autonomy/contracts/real_adapter_contract_types.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_map_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_sensor_port.hpp"
#include "robot_slamd/autonomy/ports/robot_slam_time_port.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

struct RealAdapterReadinessOptions {
    bool require_sensor_port_ready = true;
    bool require_motion_port_ready = true;
    bool require_map_port_ready = true;
    bool require_time_port_ready = false;
    bool require_shadow_before_live = true;
};

struct RealAdapterReadinessReport {
    bool ok = false;
    RealAdapterReadiness readiness = RealAdapterReadiness::NotReady;
    std::vector<RealAdapterContractViolation> violations;
    std::vector<std::string> checklist;
};

class RealAdapterReadinessChecker {
public:
    RealAdapterReadinessChecker() = default;

    explicit RealAdapterReadinessChecker(RealAdapterReadinessOptions options)
        : options_(options) {}

    RealAdapterReadinessReport check_ports(const RobotSlamSensorPort *sensor_port,
                                           const RobotSlamMotionPort *motion_port,
                                           const RobotSlamMapPort *map_port,
                                           const RobotSlamTimePort *time_port) const {
        RealAdapterReadinessReport report;

        // required port presence and readiness
        check_required_port(report,
                            options_.require_sensor_port_ready,
                            sensor_port,
                            RealAdapterKind::SensorFusion,
                            "sensor_port_missing",
                            "sensor_port_not_ready");
        check_required_port(report,
                            options_.require_motion_port_ready,
                            motion_port,
                            RealAdapterKind::Motion,
                            "motion_port_missing",
                            "motion_port_not_ready");
        check_required_port(report,
                            options_.require_map_port_ready,
                            map_port,
                            RealAdapterKind::Map,
                            "map_port_missing",
                            "map_port_not_ready");
        if (options_.require_time_port_ready && !time_port) {
            add_error(report, RealAdapterKind::Time, "time_port_missing");
        }

        // live readiness checklist. M3-A1 only reaches ShadowReady.
        if (options_.require_shadow_before_live) {
            report.checklist.push_back("software_transport_shadow_acceptance_passed");
            report.checklist.push_back("direction_probe_lifted_required");
            report.checklist.push_back("stop_estop_ttl_required");
            report.checklist.push_back("forward_backward_disabled_until_ground_test");
        }

        report.ok = report.violations.empty();
        report.readiness = report.ok ? RealAdapterReadiness::ShadowReady
                                     : RealAdapterReadiness::NotReady;
        return report;
    }

private:
    template <typename PortT>
    static void check_required_port(RealAdapterReadinessReport &report,
                                    bool required,
                                    const PortT *port,
                                    RealAdapterKind kind,
                                    const std::string &missing_code,
                                    const std::string &not_ready_code) {
        if (!required) {
            return;
        }
        if (!port) {
            add_error(report, kind, missing_code);
            return;
        }
        if (!port->ready()) {
            add_error(report, kind, not_ready_code);
        }
    }

    static void add_error(RealAdapterReadinessReport &report,
                          RealAdapterKind kind,
                          const std::string &code) {
        RealAdapterContractViolation violation;
        violation.kind = kind;
        violation.severity = RealAdapterSeverity::Error;
        violation.code = code;
        violation.message = code;
        report.violations.push_back(violation);
    }

    RealAdapterReadinessOptions options_;
};

} // namespace robot_slamd
