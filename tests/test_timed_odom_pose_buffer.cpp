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
void near(double a, double b, const char *message) {
    expect(std::fabs(a - b) < 1e-9, message);
}
robot_slamd::OdomPose2D pose(double x, double y, double yaw) {
    robot_slamd::RobotPose2D raw;
    raw.x_m = x;
    raw.y_m = y;
    raw.yaw_rad = yaw;
    return robot_slamd::make_odom_pose(raw);
}
} // namespace

int main() {
    using namespace robot_slamd;
    TimedOdomPoseBufferConfig cfg;
    cfg.capacity = 3;
    cfg.max_age_s = 10.0;
    cfg.max_interpolation_gap_s = 1.0;
    TimedOdomPoseBuffer buffer(cfg);
    expect(buffer.lookup(1.0).status == TimedPoseLookupStatus::Empty,
           "empty lookup");
    expect(buffer.append(1.0, pose(0.0, 0.0, 0.0)).ok, "append first");
    expect(!buffer.append(1.0, pose(0.0, 0.0, 0.0)).ok,
           "duplicate timestamp rejected");
    expect(!buffer.append(0.5, pose(0.0, 0.0, 0.0)).ok,
           "non-monotonic timestamp rejected");
    expect(buffer.append(2.0, pose(2.0, 4.0, 0.4)).ok, "append second");
    const auto exact = buffer.lookup(1.0);
    expect(exact.ok && exact.status == TimedPoseLookupStatus::Exact,
           "exact lookup");
    const auto mid = buffer.lookup(1.5);
    expect(mid.ok && mid.status == TimedPoseLookupStatus::Interpolated,
           "interpolated lookup");
    near(mid.pose.odom_T_base.x_m, 1.0, "interp x");
    near(mid.pose.odom_T_base.y_m, 2.0, "interp y");
    near(mid.pose.odom_T_base.yaw_rad, 0.2, "interp yaw");
    expect(buffer.lookup(0.9).status == TimedPoseLookupStatus::BeforeOldest,
           "before oldest rejected");
    expect(buffer.lookup(2.1).status == TimedPoseLookupStatus::AfterNewest,
           "after newest rejected");

    expect(buffer.append(3.0, pose(3.0, 0.0, 0.0)).ok, "append third");
    expect(buffer.append(4.0, pose(4.0, 0.0, 0.0)).ok, "capacity append");
    expect(buffer.size() == 3, "capacity bounded");
    expect(buffer.lookup(1.0).status == TimedPoseLookupStatus::BeforeOldest,
           "capacity evicts oldest");

    TimedOdomPoseBufferConfig gap_cfg;
    gap_cfg.max_interpolation_gap_s = 0.25;
    TimedOdomPoseBuffer gap_buffer(gap_cfg);
    expect(gap_buffer.append(1.0, pose(0.0, 0.0, 0.0)).ok, "gap first");
    expect(gap_buffer.append(2.0, pose(1.0, 0.0, 0.0)).ok, "gap second");
    expect(gap_buffer.lookup(1.5).status == TimedPoseLookupStatus::GapTooLarge,
           "gap too large rejected");

    TimedOdomPoseBufferConfig age_cfg;
    age_cfg.capacity = 10;
    age_cfg.max_age_s = 0.5;
    TimedOdomPoseBuffer age_buffer(age_cfg);
    expect(age_buffer.append(1.0, pose(0.0, 0.0, 0.0)).ok, "age first");
    expect(age_buffer.append(2.0, pose(1.0, 0.0, 0.0)).ok, "age second");
    expect(age_buffer.lookup(1.0).status == TimedPoseLookupStatus::BeforeOldest,
           "max age evicts oldest");
    return 0;
}
