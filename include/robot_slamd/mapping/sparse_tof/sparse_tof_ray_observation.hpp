#pragma once

#include "robot_slamd/mapping/sparse_tof/scalar_tof_return_kind.hpp"

#include <cstdint>
#include <string>

namespace robot_slamd {

struct SparseTofRayObservation {
    ScalarTofReturnKind return_kind = ScalarTofReturnKind::Invalid;
    double sensor_origin_x_m = 0.0;
    double sensor_origin_y_m = 0.0;
    double ray_yaw_rad = 0.0;
    double measured_range_m = 0.0;
    double free_space_range_m = 0.0;
    std::uint8_t confidence = 0;
    std::uint64_t echo_tag_u48 = 0;
    double effective_timestamp_s = 0.0;
    bool protocol_valid = false;
    bool synchronized = false;
    bool usable_for_mapping = false;
    std::string frame_id;
    std::string reason;
};

struct SparseTofObservationBuildResult {
    bool ok = false;
    SparseTofRayObservation observation;
    std::string fault;
    std::string message;
};

} // namespace robot_slamd
