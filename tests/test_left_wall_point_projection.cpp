#include "robot_slamd/mapping/left_wall_line_estimator.hpp"

#include <cmath>

int main() {
    using namespace robot_slamd;
    const MapPose2D base{RobotPose2D{1.0, 2.0, 0.2}};
    const RobotPose2D base_T_left{0.0, 0.08, 1.5707963267948966};
    const auto left = compose_robot_poses(base.map_T_base, base_T_left);
    const double distance_m = 0.15;
    const double x = left.x_m + std::cos(left.yaw_rad) * distance_m;
    const double y = left.y_m + std::sin(left.yaw_rad) * distance_m;
    return std::isfinite(x) && std::isfinite(y) &&
           std::fabs(x - (1.0 - std::sin(0.2) * 0.08 - std::sin(0.2) * distance_m)) < 1e-12 &&
           y > 2.20 ? 0 : 1;
}
