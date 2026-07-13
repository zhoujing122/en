#include "robot_slamd/runtime/sparse_slam_frames.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <type_traits>

namespace {

void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

void near(double a, double b, const char *message) {
    expect(std::fabs(a - b) < 1e-9, message);
}

} // namespace

int main() {
    using namespace robot_slamd;

    static_assert(!std::is_convertible<OdomPose2D, MapPose2D>::value,
                  "odom pose must not implicitly convert to map pose");
    static_assert(!std::is_convertible<MapPose2D, OdomPose2D>::value,
                  "map pose must not implicitly convert to odom pose");

    RobotPose2D odom_raw;
    odom_raw.x_m = 1.0;
    const OdomPose2D odom = make_odom_pose(odom_raw);
    const MapPose2D identity_result = compose(identity_map_from_odom(), odom);
    near(identity_result.map_T_base.x_m, 1.0, "identity x");
    near(identity_result.map_T_base.y_m, 0.0, "identity y");

    RobotPose2D transform_raw;
    transform_raw.x_m = 2.0;
    transform_raw.y_m = -1.0;
    const MapPose2D translated = compose(make_map_from_odom(transform_raw), odom);
    near(translated.map_T_base.x_m, 3.0, "translation x");
    near(translated.map_T_base.y_m, -1.0, "translation y");

    transform_raw = {};
    transform_raw.yaw_rad = 3.14159265358979323846 / 2.0;
    const MapPose2D rotated = compose(make_map_from_odom(transform_raw), odom);
    near(rotated.map_T_base.x_m, 0.0, "rotation x");
    near(rotated.map_T_base.y_m, 1.0, "rotation y");

    transform_raw.x_m = 2.0;
    transform_raw.y_m = 3.0;
    const auto transform = make_map_from_odom(transform_raw);
    const auto inverse_transform = inverse(transform);
    const auto back = compose(inverse_transform,
                              make_odom_pose(compose(transform, odom).map_T_base));
    near(back.map_T_base.x_m, odom.odom_T_base.x_m, "compose inverse x");
    near(back.map_T_base.y_m, odom.odom_T_base.y_m, "compose inverse y");

    RobotPose2D map_raw;
    map_raw.x_m = 2.0;
    map_raw.y_m = 3.0;
    map_raw.yaw_rad = 0.25;
    const auto derived = between(make_map_pose(map_raw), make_odom_pose(RobotPose2D{}));
    const auto derived_map = compose(derived, make_odom_pose(RobotPose2D{}));
    near(derived_map.map_T_base.x_m, 2.0, "between x");
    near(derived_map.map_T_base.y_m, 3.0, "between y");

    RobotPose2D wrap;
    wrap.yaw_rad = 4.0 * 3.14159265358979323846;
    expect(std::fabs(make_map_pose(wrap).map_T_base.yaw_rad) < 1e-9,
           "yaw wrap");

    wrap.yaw_rad = std::numeric_limits<double>::quiet_NaN();
    expect(!sparse_slam_frame_pose_valid(make_map_pose(wrap)),
           "nan pose invalid");

    const double delta =
        sparse_slam_shortest_yaw_delta(179.0 * 3.14159265358979323846 / 180.0,
                                       -179.0 * 3.14159265358979323846 / 180.0);
    near(delta, 2.0 * 3.14159265358979323846 / 180.0,
         "shortest yaw across pi");

    return 0;
}
