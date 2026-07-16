#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_slam_pipeline.hpp"
#include "sparse_tof_d2_test_helpers.hpp"

namespace d2pipe {

inline robot_slamd::SparseTofLocalSlamConfig config() {
    robot_slamd::SparseTofLocalSlamConfig cfg;
    cfg.grid = d2test::grid_config();
    cfg.correction.maximum_translation_jump_m = 0.40;
    cfg.correction.maximum_yaw_jump_rad = 0.40;
    cfg.keyframe.minimum_snapshot_count = 2;
    cfg.keyframe.maximum_snapshot_count = 4;
    cfg.keyframe.minimum_valid_ray_count = 3;
    cfg.keyframe.minimum_hit_ray_count = 1;
    cfg.keyframe.minimum_translation_m = 0.0;
    cfg.keyframe.minimum_rotation_rad = 0.0;
    cfg.keyframe.minimum_duration_s = 0.0;
    cfg.keyframe.maximum_duration_s = 10.0;
    cfg.matcher = d2test::matcher_config();
    cfg.matcher.score.minimum_normalized_score = -0.2;
    cfg.matcher.score.minimum_score_margin = -1.0;
    cfg.matcher.score.minimum_observability_contrast = 0.0;
    cfg.matcher.maximum_correction_translation_m = 0.40;
    cfg.matcher.maximum_correction_yaw_rad = 0.40;
    return cfg;
}

inline robot_slamd::SparseTofLocalSlamInput input(
    double t,
    const robot_slamd::RobotPose2D &odom_pose,
    bool settled = true,
    bool publish = true) {
    robot_slamd::SparseTofLocalSlamInput in;
    in.now_s = t;
    in.odom_pose = odom_pose;
    in.has_odom_pose = true;
    in.observation = d2test::observation(t, odom_pose);
    in.has_multi_tof = true;
    in.motion_active = !settled;
    in.motion_settled = settled;
    in.mapping_publish_allowed = publish;
    return in;
}

inline bool bootstrap(robot_slamd::SparseTofLocalSlamPipeline &pipeline) {
    robot_slamd::RobotPose2D p0;
    auto a = pipeline.process(input(1.0, p0));
    p0.x_m = 0.05;
    auto b = pipeline.process(input(1.1, p0));
    return a.ok && b.ok && b.keyframe_bootstrapped && pipeline.grid().active_cell_count() > 0;
}

} // namespace d2pipe
