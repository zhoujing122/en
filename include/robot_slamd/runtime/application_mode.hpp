#pragma once

#include <string>

namespace robot_slamd {

enum class SensorSource {
    Simulation,
    Replay,
    Real
};

enum class OperationMode {
    Mapping,
    Localization,
    Exploration
};

inline const char *to_string(SensorSource source) {
    switch (source) {
    case SensorSource::Simulation:
        return "simulation";
    case SensorSource::Replay:
        return "replay";
    case SensorSource::Real:
        return "real";
    }
    return "unknown";
}

inline const char *to_string(OperationMode operation) {
    switch (operation) {
    case OperationMode::Mapping:
        return "mapping";
    case OperationMode::Localization:
        return "localization";
    case OperationMode::Exploration:
        return "exploration";
    }
    return "unknown";
}

inline bool parse_sensor_source(const std::string &value,
                                SensorSource &source) {
    if (value == "simulation") {
        source = SensorSource::Simulation;
        return true;
    }
    if (value == "replay") {
        source = SensorSource::Replay;
        return true;
    }
    if (value == "real") {
        source = SensorSource::Real;
        return true;
    }
    return false;
}

inline bool parse_operation_mode(const std::string &value,
                                 OperationMode &operation) {
    if (value == "mapping") {
        operation = OperationMode::Mapping;
        return true;
    }
    if (value == "localization") {
        operation = OperationMode::Localization;
        return true;
    }
    if (value == "exploration") {
        operation = OperationMode::Exploration;
        return true;
    }
    return false;
}

} // namespace robot_slamd
