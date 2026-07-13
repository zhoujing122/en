#include "robot_slamd/runtime/timed_odom_pose_buffer.hpp"

#include <cmath>
#include <iostream>

namespace {
void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}
robot_slamd::OdomPose2D pose(double yaw_deg) {
    robot_slamd::RobotPose2D raw;
    raw.yaw_rad = yaw_deg * 3.14159265358979323846 / 180.0;
    return robot_slamd::make_odom_pose(raw);
}
} // namespace

int main() {
    using namespace robot_slamd;
    TimedOdomPoseBufferConfig cfg;
    cfg.max_interpolation_gap_s = 2.0;
    TimedOdomPoseBuffer buffer(cfg);
    expect(buffer.append(1.0, pose(179.0)).ok, "append 179");
    expect(buffer.append(2.0, pose(-179.0)).ok, "append -179");
    const auto mid = buffer.lookup(1.5);
    expect(mid.ok, "mid lookup");
    expect(std::fabs(std::fabs(mid.pose.odom_T_base.yaw_rad) -
                     3.14159265358979323846) < 1e-9,
           "shortest interpolation crosses pi");
    robot_slamd::RobotPose2D bad;
    bad.x_m = std::numeric_limits<double>::quiet_NaN();
    expect(!buffer.append(3.0, make_odom_pose(bad)).ok,
           "nan pose rejected");
    expect(!buffer.append(std::numeric_limits<double>::quiet_NaN(),
                          pose(0.0)).ok,
           "nan timestamp rejected");
    return 0;
}
