#include "robot_slamd/localization/sparse_tof/sparse_tof_pose_correction_state.hpp"

#include <cmath>
#include <iostream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    using namespace robot_slamd;
    SparseTofPoseCorrectionConfig cfg;
    cfg.maximum_translation_jump_m = 0.30;
    cfg.maximum_yaw_jump_rad = 0.20;
    cfg.maximum_history_count = 2;
    SparseTofPoseCorrectionState state(cfg);
    if (!state.reset_new_map().ok) return fail("reset failed");

    RobotPose2D odom;
    odom.x_m = 1.0;
    RobotPose2D matched;
    matched.x_m = 1.1;
    matched.yaw_rad = 0.1;
    const auto prepared = state.prepare_correction(odom, matched);
    if (!prepared.ok || !state.has_prepared_correction()) return fail("prepare failed");
    const auto before = state.corrected_pose(odom);
    if (std::fabs(before.x_m - 1.0) > 1e-9) return fail("prepare changed state before commit");
    const auto committed = state.commit_prepared_correction();
    if (!committed.ok || state.correction_count() != 1) return fail("commit failed");
    const auto after = state.corrected_pose(odom);
    if (std::fabs(after.x_m - matched.x_m) > 1e-9 || std::fabs(after.yaw_rad - matched.yaw_rad) > 1e-9) {
        return fail("committed correction not applied");
    }

    RobotPose2D huge = matched;
    huge.x_m += 2.0;
    const auto rejected = state.prepare_correction(odom, huge);
    if (rejected.ok || rejected.fault != SparseTofPoseCorrectionFault::TranslationJumpRejected) {
        return fail("large translation jump accepted");
    }
    const auto unchanged = state.corrected_pose(odom);
    if (std::fabs(unchanged.x_m - matched.x_m) > 1e-9) return fail("failed prepare changed pose");

    SparseTofPoseCorrectionConfig yaw_cfg;
    yaw_cfg.maximum_translation_jump_m = 10.0;
    yaw_cfg.maximum_yaw_jump_rad = 0.20;
    SparseTofPoseCorrectionState yaw_state(yaw_cfg);
    yaw_state.reset_new_map();
    RobotPose2D yaw_odom;
    RobotPose2D yaw_matched;
    yaw_matched.yaw_rad = 1.0;
    const auto yaw_rejected = yaw_state.prepare_correction(yaw_odom, yaw_matched);
    if (yaw_rejected.ok || yaw_rejected.fault != SparseTofPoseCorrectionFault::YawJumpRejected) {
        return fail("large yaw jump accepted");
    }

    return 0;
}
