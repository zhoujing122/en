#include "robot_slamd/localization/sparse_tof/sparse_tof_pose_correction_state.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    SparseTofPoseCorrectionState state;
    auto startup = state.reset_new_map();
    if (!startup.ok || state.initial_pose_mode() != InitialPoseMode::StartupZero) {
        return fail("StartupZero reset failed");
    }
    RobotPose2D odom;
    odom.x_m = 1.0;
    const auto map_pose = state.corrected_pose(odom);
    if (std::fabs(map_pose.x_m - 1.0) > 1e-9 || std::fabs(map_pose.yaw_rad) > 1e-9) {
        return fail("StartupZero should define identity map_from_odom");
    }

    RobotPose2D configured;
    configured.x_m = 2.0;
    configured.y_m = -1.0;
    configured.yaw_rad = 0.4;
    auto cfg = state.reset_configured_pose(configured);
    if (!cfg.ok || state.initial_pose_mode() != InitialPoseMode::ConfiguredPose) {
        return fail("ConfiguredPose reset failed");
    }
    const auto corrected = state.corrected_pose(RobotPose2D{});
    if (std::fabs(corrected.x_m - 2.0) > 1e-9 || std::fabs(corrected.y_m + 1.0) > 1e-9 || std::fabs(corrected.yaw_rad - 0.4) > 1e-9) {
        return fail("configured initial map pose not applied");
    }

    auto missing = state.reset_existing_map_without_pose();
    if (missing.ok || missing.fault != SparseTofPoseCorrectionFault::ExistingMapRequiresInitialPose) {
        return fail("existing map without explicit pose must be rejected");
    }

    configured.yaw_rad = std::numeric_limits<double>::quiet_NaN();
    auto bad = state.reset_configured_pose(configured);
    if (bad.ok || bad.fault != SparseTofPoseCorrectionFault::InvalidInitialPose) {
        return fail("non-finite initial pose accepted");
    }

    return 0;
}
