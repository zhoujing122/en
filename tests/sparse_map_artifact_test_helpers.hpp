#pragma once

#include "robot_slamd/mapping/sparse_tof/sparse_map_artifact.hpp"

#include <stdexcept>
#include <string>

namespace sparse_map_artifact_test {

inline void expect(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

inline robot_slamd::SparseMapArtifact artifact() {
    robot_slamd::SparseMapArtifact out;
    out.map_id = "unit_map";
    out.run_id = "unit_run";
    out.map_revision = 17;
    out.tof_extrinsics.front = {0.18, 0.01, 0.02};
    out.tof_extrinsics.left = {-0.02, 0.14, 1.55};
    out.tof_extrinsics.right = {-0.03, -0.15, -1.54};
    out.wheel_base_m = 0.20;
    out.wheel_radius_left_m = 0.04;
    out.wheel_radius_right_m = 0.041;
    out.encoder_ticks_per_rev = 2048;
    out.cells = {{{-2, -1}, -9}, {{3, -1}, 12}, {{0, 4}, -6}};
    return out;
}

inline robot_slamd::SparseMapArtifactLimits limits() {
    return {100, 1024U * 1024U};
}

} // namespace sparse_map_artifact_test
