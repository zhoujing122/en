#pragma once

#include "robot_slamd/autonomy/real_adapters/real_adapter_stub_types.hpp"

namespace robot_slamd {

class LiveHandoffReadinessChecker {
public:
    explicit LiveHandoffReadinessChecker(
        LiveHandoffReadinessOptions options = {})
        : options_(options) {}

    LiveHandoffReadinessReport evaluate(
        const RealAdapterStubStatus &sensor_status,
        const RealAdapterStubStatus &motion_status,
        const RealAdapterStubStatus &slam_backend_status,
        bool software_transport_acceptance_passed,
        bool e2e_prelive_passed,
        bool direction_probe_passed,
        bool stop_estop_ttl_passed,
        bool hardware_safety_confirmed) const {
        LiveHandoffReadinessReport report;

        // Configuration gate.
        if (!options_.enabled) {
            report.block_reason = LiveHandoffBlockReason::ConfigDisabled;
            report.failed.push_back("live_handoff_config_disabled");
            report.summary = "live_handoff_blocked";
            add_forward_backward_warning(report);
            return report;
        }

        // Real adapter gates.
        add_required_status(report,
                            options_.require_real_sensor_adapter,
                            sensor_status.ready,
                            "real_sensor_adapter_ready",
                            "real_sensor_adapter_not_ready",
                            LiveHandoffBlockReason::RealSensorAdapterMissing);
        add_required_status(report,
                            options_.require_real_motion_adapter,
                            motion_status.ready,
                            "real_motion_adapter_ready",
                            "real_motion_adapter_not_ready",
                            LiveHandoffBlockReason::RealMotionAdapterMissing);
        add_required_status(report,
                            options_.require_real_slam_backend,
                            slam_backend_status.ready,
                            "real_slam_backend_ready",
                            "real_slam_backend_not_ready",
                            LiveHandoffBlockReason::RealSlamBackendMissing);

        // Shadow and pre-live evidence gates.
        add_required_status(
            report,
            options_.require_software_transport_acceptance,
            software_transport_acceptance_passed,
            "software_transport_acceptance_passed",
            "software_transport_acceptance_missing",
            LiveHandoffBlockReason::SoftwareTransportAcceptanceMissing);
        add_required_status(report,
                            options_.require_e2e_prelive_pass,
                            e2e_prelive_passed,
                            "e2e_prelive_passed",
                            "e2e_prelive_missing",
                            LiveHandoffBlockReason::E2EPreLiveMissing);

        // Live preparation gates.
        add_required_status(report,
                            options_.require_direction_probe,
                            direction_probe_passed,
                            "direction_probe_passed",
                            "direction_probe_missing",
                            LiveHandoffBlockReason::DirectionProbeMissing);
        add_required_status(report,
                            options_.require_stop_estop_ttl,
                            stop_estop_ttl_passed,
                            "stop_estop_ttl_passed",
                            "stop_estop_ttl_missing",
                            LiveHandoffBlockReason::StopEstopTtlMissing);
        add_required_status(report,
                            options_.require_hardware_safety,
                            hardware_safety_confirmed,
                            "hardware_safety_confirmed",
                            "hardware_safety_missing",
                            LiveHandoffBlockReason::HardwareSafetyMissing);
        add_forward_backward_warning(report);

        if (!report.failed.empty()) {
            report.ok = false;
            report.state = LiveHandoffReadinessState::Blocked;
            report.summary = "live_handoff_blocked";
            return report;
        }

        // M3-B0 cannot claim live readiness. The best possible result is a
        // stub-only handoff state until real verification is added later.
        report.ok = false;
        report.state = LiveHandoffReadinessState::StubOnlyReady;
        report.block_reason = LiveHandoffBlockReason::RealHardwareNotVerified;
        report.warnings.push_back("real_hardware_not_verified");
        report.summary = "live_handoff_stub_only_ready";
        return report;
    }

private:
    static void set_first_block_reason(
        LiveHandoffReadinessReport &report,
        LiveHandoffBlockReason reason) {
        if (report.block_reason == LiveHandoffBlockReason::None) {
            report.block_reason = reason;
        }
    }

    static void add_required_status(
        LiveHandoffReadinessReport &report,
        bool required,
        bool passed,
        const std::string &passed_case,
        const std::string &failed_case,
        LiveHandoffBlockReason reason) {
        if (!required) {
            return;
        }
        if (passed) {
            report.passed.push_back(passed_case);
            return;
        }
        report.failed.push_back(failed_case);
        set_first_block_reason(report, reason);
    }

    void add_forward_backward_warning(
        LiveHandoffReadinessReport &report) const {
        if (!options_.allow_forward_backward) {
            report.warnings.push_back("forward_backward_remains_disabled");
        }
    }

    LiveHandoffReadinessOptions options_;
};

} // namespace robot_slamd
