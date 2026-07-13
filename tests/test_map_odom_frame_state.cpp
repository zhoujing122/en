#include "robot_slamd/runtime/sparse_slam_initialization.hpp"

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

} // namespace

int main() {
    using namespace robot_slamd;

    MapOdomFrameState state;
    expect(!state.initialized(), "starts uninitialized");
    auto startup = state.initialize_startup_zero();
    expect(startup.ok, "startup zero initializes");
    expect(state.initialized(), "state initialized");
    expect(startup.initial_yaw_defined_by_startup_frame,
           "startup yaw defined by frame");
    expect(!startup.initial_yaw_measured_by_imu, "imu does not measure yaw");

    RobotPose2D odom_raw;
    odom_raw.x_m = 1.0;
    const auto map_pose = state.map_pose_from_odom(make_odom_pose(odom_raw));
    near(map_pose.map_T_base.x_m, 1.0, "identity map x");
    near(map_pose.map_T_base.y_m, 0.0, "identity map y");

    MapOdomFrameState configured_state;
    RobotPose2D configured_raw;
    configured_raw.x_m = 2.5;
    configured_raw.y_m = -0.75;
    configured_raw.yaw_rad = 0.5;
    auto configured =
        configured_state.initialize_configured_pose(make_map_pose(configured_raw));
    expect(configured.ok, "configured pose initializes");
    expect(configured.configured_pose_used, "configured pose reported");
    const auto configured_map =
        configured_state.map_pose_from_odom(make_odom_pose(RobotPose2D{}));
    near(configured_map.map_T_base.x_m, 2.5, "configured x");
    near(configured_map.map_T_base.y_m, -0.75, "configured y");
    near(configured_map.map_T_base.yaw_rad, 0.5, "configured yaw");

    MapOdomFrameState transaction;
    expect(transaction.initialize_startup_zero().ok, "first init ok");
    SparseSlamInitializationRequest bad;
    bad.map_startup_mode = MapStartupMode::CreateNewMap;
    bad.initial_pose_mode = InitialPoseMode::ConfiguredPose;
    auto failed = transaction.initialize(bad);
    expect(!failed.ok, "bad configured pose rejected");
    expect(transaction.initialized(), "failed init leaves previous state");
    near(transaction.current_map_from_odom().map_T_odom.x_m, 0.0,
         "failed init does not change transform");

    return 0;
}
