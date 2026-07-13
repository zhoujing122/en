#pragma once

#include <string>

namespace robot_slamd {

enum class SlamRuntimeMode {
    Legacy,
    SparseShadow
};

inline const char *to_string(SlamRuntimeMode mode) {
    switch (mode) {
    case SlamRuntimeMode::Legacy:
        return "legacy";
    case SlamRuntimeMode::SparseShadow:
        return "sparse_shadow";
    }
    return "unknown";
}

inline bool parse_slam_runtime_mode(const std::string &value,
                                    SlamRuntimeMode &mode) {
    if (value == "legacy") {
        mode = SlamRuntimeMode::Legacy;
        return true;
    }
    if (value == "sparse_shadow") {
        mode = SlamRuntimeMode::SparseShadow;
        return true;
    }
    return false;
}

inline SlamRuntimeMode resolved_slam_runtime_mode(const std::string &value) {
    SlamRuntimeMode mode = SlamRuntimeMode::Legacy;
    (void)parse_slam_runtime_mode(value, mode);
    return mode;
}

} // namespace robot_slamd
