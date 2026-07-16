#include "robot_slamd/localization/sparse_tof/se2_pose_transform.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

int fail(const char *message) {
    std::cerr << message << "\n";
    return 1;
}

} // namespace

int main() {
    using namespace robot_slamd;

    RobotPose2D a;
    a.x_m = 1.0;
    a.y_m = 2.0;
    a.yaw_rad = 0.5;
    RobotPose2D identity;

    const auto composed = se2_compose(a, identity);
    if (!near(composed.x_m, a.x_m) || !near(composed.y_m, a.y_m) || !near(composed.yaw_rad, a.yaw_rad)) {
        return fail("identity compose changed pose");
    }

    const auto inv = se2_inverse(a);
    const auto back = se2_compose(a, inv);
    if (!near(back.x_m, 0.0, 1e-8) || !near(back.y_m, 0.0, 1e-8) || !near(back.yaw_rad, 0.0, 1e-8)) {
        return fail("inverse did not return identity");
    }

    RobotPose2D b;
    b.x_m = 1.2;
    b.y_m = 1.8;
    b.yaw_rad = -0.25;
    const auto between = se2_between(a, b);
    const auto recovered = se2_compose(a, between);
    if (!near(recovered.x_m, b.x_m, 1e-8) || !near(recovered.y_m, b.y_m, 1e-8) || !near(recovered.yaw_rad, b.yaw_rad, 1e-8)) {
        return fail("between did not recover target pose");
    }

    const double normalized = se2_normalize_yaw(4.0);
    if (normalized > 3.14159265358979323846 || normalized < -3.14159265358979323846) {
        return fail("yaw normalization out of range");
    }

    RobotPose2D bad = a;
    bad.x_m = std::numeric_limits<double>::quiet_NaN();
    if (se2_pose_finite(bad)) {
        return fail("non-finite pose accepted");
    }

    return 0;
}
