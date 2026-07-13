#include "robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

robot_slamd::WheelOdomFrame wheel(double t,
                                  double linear,
                                  double angular,
                                  bool valid = true) {
    robot_slamd::WheelOdomFrame w;
    w.timestamp_s = t;
    w.linear_mps = linear;
    w.angular_rad_s = angular;
    w.valid = valid;
    return w;
}

robot_slamd::ImuFrame imu(double t, double yaw_rate) {
    robot_slamd::ImuFrame i;
    i.timestamp_s = t;
    i.yaw_rate_rad_s = yaw_rate;
    i.az_mps2 = 9.80665;
    return i;
}
}

int main() {
    using namespace robot_slamd;

    WheelImuDeadReckoningConfig cfg;
    cfg.maximum_dt_s = 0.20;
    cfg.timestamp_tolerance_s = 0.01;
    cfg.maximum_yaw_rate_disagreement_rad_s = 0.25;
    WheelImuDeadReckoning2D estimator(cfg);
    expect(estimator.config_valid(), "valid config");

    auto bad = cfg;
    bad.wheel_radius_m = 0.0;
    expect(!WheelImuDeadReckoning2D(bad).config_valid(),
           "invalid wheel radius");
    bad = cfg;
    bad.wheel_base_m = 0.0;
    expect(!WheelImuDeadReckoning2D(bad).config_valid(),
           "invalid wheel base");
    bad = cfg;
    bad.maximum_dt_s = std::numeric_limits<double>::quiet_NaN();
    expect(!WheelImuDeadReckoning2D(bad).config_valid(), "nan config");

    expect(estimator.reset().ok, "reset origin");
    expect(near(estimator.estimated_pose().x_m, 0.0), "reset x");
    RobotPose2D nonzero;
    nonzero.x_m = 1.0;
    nonzero.y_m = -2.0;
    nonzero.yaw_rad = 3.5;
    expect(estimator.reset(nonzero).ok, "reset nonzero");
    expect(estimator.estimated_pose().yaw_rad < 3.14159265358979323846,
           "reset yaw normalized");
    nonzero.x_m = std::numeric_limits<double>::quiet_NaN();
    expect(!estimator.reset(nonzero).ok, "reject nonfinite initial pose");

    expect(estimator.reset().ok, "reset for motion");
    auto r = estimator.update(wheel(1.0, 0.0, 0.0), imu(1.0, 0.0));
    expect(r.ok &&
               r.status == WheelImuDeadReckoningStatus::InitializedWithoutMotion,
           "first sample initializes without motion");
    expect(near(estimator.estimated_pose().x_m, 0.0), "first sample no x");

    r = estimator.update(wheel(1.1, 0.0, 0.0), imu(1.1, 0.0));
    expect(r.ok, "static update ok");
    expect(near(estimator.estimated_pose().x_m, 0.0), "static no x");
    expect(near(estimator.estimated_pose().yaw_rad, 0.0), "static no yaw");

    r = estimator.update(wheel(1.2, 1.0, 0.0), imu(1.2, 0.0));
    expect(r.ok, "forward update ok");
    expect(near(estimator.estimated_pose().x_m, 0.1), "forward x");

    r = estimator.update(wheel(1.3, -1.0, 0.0), imu(1.3, 0.0));
    expect(r.ok, "backward update ok");
    expect(near(estimator.estimated_pose().x_m, 0.0), "backward returns x");

    r = estimator.update(wheel(1.4, 0.0, 1.0), imu(1.4, 1.0));
    expect(r.ok, "left turn ok");
    expect(estimator.estimated_pose().yaw_rad > 0.09, "left turn yaw");

    r = estimator.update(wheel(1.5, 0.0, -1.0), imu(1.5, -1.0));
    expect(r.ok, "right turn ok");
    expect(near(estimator.estimated_pose().yaw_rad, 0.0), "right turn yaw");

    r = estimator.update(wheel(1.6, 1.0, 1.0), imu(1.6, 1.0));
    expect(r.ok, "curve update ok");
    expect(estimator.estimated_pose().x_m > 0.09 &&
               estimator.estimated_pose().y_m > 0.0,
           "curve pose changes");

    RobotPose2D wrap_pose;
    wrap_pose.yaw_rad = 3.13;
    expect(estimator.reset(wrap_pose).ok, "reset wrap pose");
    expect(estimator.update(wheel(2.0, 0.0, 0.0), imu(2.0, 0.0)).ok,
           "wrap baseline");
    expect(estimator.update(wheel(2.1, 0.0, 1.0), imu(2.1, 1.0)).ok,
           "wrap update");
    expect(estimator.estimated_pose().yaw_rad < 3.14159265358979323846,
           "yaw remains normalized");

    expect(estimator.reset().ok, "reset failure cases");
    expect(estimator.update(wheel(3.0, 0.0, 0.0), imu(3.0, 0.0)).ok,
           "failure baseline");
    const auto before = estimator.estimated_pose();
    expect(!estimator.update(wheel(3.0, 0.1, 0.0), imu(3.1, 0.0)).ok,
           "wheel timestamp not monotonic");
    expect(near(estimator.estimated_pose().x_m, before.x_m),
           "failure does not move pose");
    expect(!estimator.update(wheel(3.3, 0.1, 0.0), imu(3.3, 0.0)).ok,
           "dt too large");
    expect(!estimator.update(wheel(3.1, 0.1, 0.0, false), imu(3.1, 0.0)).ok,
           "wheel dropout");
    expect(!estimator.update(wheel(3.1, 10.0, 0.0), imu(3.1, 0.0)).ok,
           "wheel delta rejected");
    expect(!estimator.update(wheel(3.1, 0.0, 0.0), imu(3.2, 0.0)).ok,
           "timestamp mismatch");
    expect(!estimator.update(wheel(3.1, 0.0, 0.0), imu(3.1, 2.0)).ok,
           "yaw disagreement");
    expect(estimator.update(wheel(3.1, 0.0, 0.0), imu(3.1, 0.0)).ok,
           "valid input recovers after failures");

    WheelImuDeadReckoningConfig fallback_cfg = cfg;
    fallback_cfg.allow_imu_missing_wheel_yaw_fallback = true;
    fallback_cfg.enable_wheel_imu_yaw_consistency_gate = false;
    WheelImuDeadReckoning2D fallback(fallback_cfg);
    expect(fallback.reset().ok, "fallback reset");
    expect(fallback.update(wheel(4.0, 0.0, 0.0), imu(4.0, 0.0)).ok,
           "fallback baseline");
    expect(fallback.update(wheel(4.1, 0.0, 1.0), imu(4.1, 0.0), false).ok,
           "wheel yaw fallback");
    expect(fallback.estimated_pose().yaw_rad > 0.09, "fallback yaw");

    WheelImuDeadReckoning2D a(cfg);
    WheelImuDeadReckoning2D b(cfg);
    a.reset();
    b.reset();
    a.update(wheel(5.0, 0.0, 0.0), imu(5.0, 0.0));
    b.update(wheel(5.0, 0.0, 0.0), imu(5.0, 0.0));
    a.update(wheel(5.1, 0.2, 0.3), imu(5.1, 0.3));
    b.update(wheel(5.1, 0.2, 0.3), imu(5.1, 0.3));
    expect(near(a.estimated_pose().x_m, b.estimated_pose().x_m) &&
               near(a.estimated_pose().yaw_rad, b.estimated_pose().yaw_rad),
           "deterministic same samples");

    WheelImuDeadReckoning2D command_left(cfg);
    WheelImuDeadReckoning2D command_right(cfg);
    command_left.reset();
    command_right.reset();
    command_left.update(wheel(6.0, 0.0, 0.0), imu(6.0, 0.0));
    command_right.update(wheel(6.0, 0.0, 0.0), imu(6.0, 0.0));
    command_left.update(wheel(6.1, 0.0, 0.0), imu(6.1, 0.0));
    command_right.update(wheel(6.1, 0.0, 0.0), imu(6.1, 0.0));
    expect(near(command_left.estimated_pose().yaw_rad,
                command_right.estimated_pose().yaw_rad),
           "different command metadata cannot affect estimator");

    WheelImuDeadReckoning2D no_command(cfg);
    no_command.reset();
    no_command.update(wheel(7.0, 0.0, 0.0), imu(7.0, 0.0));
    no_command.update(wheel(7.1, 0.0, 1.0), imu(7.1, 1.0));
    expect(no_command.estimated_pose().yaw_rad > 0.09,
           "wheel imu turn works without command");

    return failures ? 1 : 0;
}
