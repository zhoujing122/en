#include "robot_slamd/mapping/left_wall_line_estimator.hpp"

#include <cmath>

int main() {
    using namespace robot_slamd;
    LeftWallLineEstimatorConfig config;
    config.window_max_points = 10;
    config.min_fit_points = 5;
    config.min_baseline_m = 0.05;
    config.max_fit_rms_m = 0.03;
    config.outlier_min_threshold_m = 0.02;
    LeftWallLineEstimator estimator(config);
    std::vector<LeftWallPoint> points;
    for (std::size_t i = 0; i < 7; ++i) {
        points.push_back({0.1 * static_cast<double>(i), 0.6, 1.0 + i,
                          1.0, i, i + 1, 1});
    }
    points.push_back({0.35, 0.85, 9.0, 1.0, 8, 9, 1});
    const auto model = estimator.fit(points, RobotPose2D{0.0, 0.0, 0.0}, 1);
    return model.valid && model.inlier_point_count >= 6 &&
           model.baseline_m >= 0.49 && model.rms_residual_m < 1e-6 &&
           std::fabs(model.wall_heading_rad) < 1e-9 &&
           std::fabs(model.signed_base_to_wall_distance_m - 0.6) < 1e-9
               ? 0 : 1;
}
