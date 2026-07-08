#pragma once

#include <string>
#include <vector>

namespace robot_slamd {

enum class RealAdapterStubKind {
    Sensor,
    Motion,
    SlamBackend,
    Factory
};

enum class RealAdapterStubState {
    Disabled,
    StubOnly,
    WaitingForImplementation,
    Blocked,
    Fault
};

enum class LiveHandoffReadinessState {
    Blocked,
    StubOnlyReady,
    ShadowReady,
    LivePreflightBlocked,
    LiveReady
};

enum class LiveHandoffBlockReason {
    None,
    ConfigDisabled,
    RealSensorAdapterMissing,
    RealMotionAdapterMissing,
    RealSlamBackendMissing,
    SoftwareTransportAcceptanceMissing,
    E2EPreLiveMissing,
    DirectionProbeMissing,
    StopEstopTtlMissing,
    HardwareSafetyMissing,
    ForwardBackwardNotAllowed,
    RealHardwareNotVerified
};

struct RealAdapterStubStatus {
    RealAdapterStubKind kind = RealAdapterStubKind::Factory;
    RealAdapterStubState state = RealAdapterStubState::Disabled;
    bool ready = false;
    std::string name;
    std::string message;
};

struct RealAdapterFactoryOptions {
    bool enabled = false;
    bool create_sensor_stub = true;
    bool create_motion_stub = true;
    bool create_slam_backend_stub = true;
    bool allow_real_hardware_adapters = false;
    bool require_explicit_live_enable = true;
};

struct LiveHandoffReadinessOptions {
    bool enabled = false;
    bool require_real_sensor_adapter = true;
    bool require_real_motion_adapter = true;
    bool require_real_slam_backend = true;
    bool require_software_transport_acceptance = true;
    bool require_e2e_prelive_pass = true;
    bool require_direction_probe = true;
    bool require_stop_estop_ttl = true;
    bool require_hardware_safety = true;
    bool allow_forward_backward = false;
};

struct LiveHandoffReadinessReport {
    bool ok = false;
    LiveHandoffReadinessState state = LiveHandoffReadinessState::Blocked;
    LiveHandoffBlockReason block_reason = LiveHandoffBlockReason::None;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(RealAdapterStubKind kind) {
    switch (kind) {
    case RealAdapterStubKind::Sensor:
        return "Sensor";
    case RealAdapterStubKind::Motion:
        return "Motion";
    case RealAdapterStubKind::SlamBackend:
        return "SlamBackend";
    case RealAdapterStubKind::Factory:
        return "Factory";
    }
    return "Unknown";
}

inline std::string to_string(RealAdapterStubState state) {
    switch (state) {
    case RealAdapterStubState::Disabled:
        return "Disabled";
    case RealAdapterStubState::StubOnly:
        return "StubOnly";
    case RealAdapterStubState::WaitingForImplementation:
        return "WaitingForImplementation";
    case RealAdapterStubState::Blocked:
        return "Blocked";
    case RealAdapterStubState::Fault:
        return "Fault";
    }
    return "Unknown";
}

inline std::string to_string(LiveHandoffReadinessState state) {
    switch (state) {
    case LiveHandoffReadinessState::Blocked:
        return "Blocked";
    case LiveHandoffReadinessState::StubOnlyReady:
        return "StubOnlyReady";
    case LiveHandoffReadinessState::ShadowReady:
        return "ShadowReady";
    case LiveHandoffReadinessState::LivePreflightBlocked:
        return "LivePreflightBlocked";
    case LiveHandoffReadinessState::LiveReady:
        return "LiveReady";
    }
    return "Unknown";
}

inline std::string to_string(LiveHandoffBlockReason reason) {
    switch (reason) {
    case LiveHandoffBlockReason::None:
        return "None";
    case LiveHandoffBlockReason::ConfigDisabled:
        return "ConfigDisabled";
    case LiveHandoffBlockReason::RealSensorAdapterMissing:
        return "RealSensorAdapterMissing";
    case LiveHandoffBlockReason::RealMotionAdapterMissing:
        return "RealMotionAdapterMissing";
    case LiveHandoffBlockReason::RealSlamBackendMissing:
        return "RealSlamBackendMissing";
    case LiveHandoffBlockReason::SoftwareTransportAcceptanceMissing:
        return "SoftwareTransportAcceptanceMissing";
    case LiveHandoffBlockReason::E2EPreLiveMissing:
        return "E2EPreLiveMissing";
    case LiveHandoffBlockReason::DirectionProbeMissing:
        return "DirectionProbeMissing";
    case LiveHandoffBlockReason::StopEstopTtlMissing:
        return "StopEstopTtlMissing";
    case LiveHandoffBlockReason::HardwareSafetyMissing:
        return "HardwareSafetyMissing";
    case LiveHandoffBlockReason::ForwardBackwardNotAllowed:
        return "ForwardBackwardNotAllowed";
    case LiveHandoffBlockReason::RealHardwareNotVerified:
        return "RealHardwareNotVerified";
    }
    return "Unknown";
}

inline int real_adapter_stub_kind_id(RealAdapterStubKind kind) {
    return static_cast<int>(kind);
}

inline int real_adapter_stub_state_id(RealAdapterStubState state) {
    return static_cast<int>(state);
}

inline int live_handoff_readiness_state_id(
    LiveHandoffReadinessState state) {
    return static_cast<int>(state);
}

inline int live_handoff_block_reason_id(LiveHandoffBlockReason reason) {
    return static_cast<int>(reason);
}

} // namespace robot_slamd
