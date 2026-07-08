#pragma once

#include <string>
#include <vector>

namespace robot_slamd {

enum class RealAdapterKind {
    Tof,
    Imu,
    Wheel,
    SensorFusion,
    Map,
    Motion,
    Time
};

enum class RealAdapterSeverity {
    Info,
    Warning,
    Error,
    Fatal
};

enum class RealAdapterReadiness {
    NotReady,
    ShadowReady,
    LivePreflightReady,
    LiveBlocked
};

struct RealAdapterContractViolation {
    RealAdapterKind kind = RealAdapterKind::SensorFusion;
    RealAdapterSeverity severity = RealAdapterSeverity::Error;
    std::string code;
    std::string message;
};

struct RealAdapterContractReport {
    bool ok = false;
    RealAdapterReadiness readiness = RealAdapterReadiness::NotReady;
    std::vector<RealAdapterContractViolation> violations;
    std::vector<std::string> warnings;
};

struct TofAdapterContractOptions {
    bool required = true;
    int min_range_count = 3;
    int max_range_count = 4096;
    double min_range_m = 0.02;
    double max_range_m = 10.0;
    double max_frame_age_s = 0.50;
    double max_allowed_nan_ratio = 0.25;
    bool require_frame_id = true;
};

struct ImuAdapterContractOptions {
    bool required = true;
    double max_frame_age_s = 0.50;
    double max_abs_yaw_rate_rad_s = 20.0;
    double max_abs_acc_mps2 = 80.0;
};

struct WheelAdapterContractOptions {
    bool required_if_no_imu = true;
    double max_frame_age_s = 0.50;
    double max_abs_linear_mps = 2.0;
    double max_abs_angular_rad_s = 10.0;
};

struct MapAdapterContractOptions {
    bool required = true;
    double min_coverage_ratio = 0.0;
    double max_coverage_ratio = 1.0;
    double min_yaw_coverage_ratio = 0.0;
    double max_yaw_coverage_ratio = 1.0;
};

struct RealAdapterContractOptions {
    TofAdapterContractOptions tof;
    ImuAdapterContractOptions imu;
    WheelAdapterContractOptions wheel;
    MapAdapterContractOptions map;
    bool require_tof = true;
    bool require_imu_or_wheel = true;
    bool allow_wheel_without_imu = true;
    bool allow_empty_map_reason_when_good = true;
};

inline std::string to_string(RealAdapterKind kind) {
    switch (kind) {
    case RealAdapterKind::Tof:
        return "tof";
    case RealAdapterKind::Imu:
        return "imu";
    case RealAdapterKind::Wheel:
        return "wheel";
    case RealAdapterKind::SensorFusion:
        return "sensor_fusion";
    case RealAdapterKind::Map:
        return "map";
    case RealAdapterKind::Motion:
        return "motion";
    case RealAdapterKind::Time:
        return "time";
    }
    return "unknown";
}

inline std::string to_string(RealAdapterSeverity severity) {
    switch (severity) {
    case RealAdapterSeverity::Info:
        return "info";
    case RealAdapterSeverity::Warning:
        return "warning";
    case RealAdapterSeverity::Error:
        return "error";
    case RealAdapterSeverity::Fatal:
        return "fatal";
    }
    return "unknown";
}

inline std::string to_string(RealAdapterReadiness readiness) {
    switch (readiness) {
    case RealAdapterReadiness::NotReady:
        return "not_ready";
    case RealAdapterReadiness::ShadowReady:
        return "shadow_ready";
    case RealAdapterReadiness::LivePreflightReady:
        return "live_preflight_ready";
    case RealAdapterReadiness::LiveBlocked:
        return "live_blocked";
    }
    return "unknown";
}

inline int real_adapter_kind_id(RealAdapterKind kind) {
    return static_cast<int>(kind);
}

inline int real_adapter_severity_id(RealAdapterSeverity severity) {
    return static_cast<int>(severity);
}

inline int real_adapter_readiness_id(RealAdapterReadiness readiness) {
    return static_cast<int>(readiness);
}

} // namespace robot_slamd
