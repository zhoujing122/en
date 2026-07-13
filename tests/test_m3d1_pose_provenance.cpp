#include "robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

std::string read_file(const char *path) {
    std::ifstream in(path);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

bool contains(const std::string &text, const char *needle) {
    return text.find(needle) != std::string::npos;
}

std::string repo_path(const char *relative_path) {
    std::string source_file = __FILE__;
    const std::string suffix = "tests/test_m3d1_pose_provenance.cpp";
    const auto pos = source_file.rfind(suffix);
    if (pos == std::string::npos) {
        return relative_path;
    }
    return source_file.substr(0, pos) + relative_path;
}

robot_slamd::WheelOdomFrame wheel(double timestamp_s,
                                  double linear_mps,
                                  double angular_rad_s) {
    robot_slamd::WheelOdomFrame frame;
    frame.timestamp_s = timestamp_s;
    frame.linear_mps = linear_mps;
    frame.angular_rad_s = angular_rad_s;
    frame.valid = true;
    return frame;
}

robot_slamd::ImuFrame imu(double timestamp_s, double yaw_rate_rad_s) {
    robot_slamd::ImuFrame frame;
    frame.timestamp_s = timestamp_s;
    frame.yaw_rate_rad_s = yaw_rate_rad_s;
    frame.az_mps2 = 9.80665;
    return frame;
}
} // namespace

int main() {
    const auto estimator_header = read_file(repo_path(
        "include/robot_slamd/autonomy/odometry/"
        "wheel_imu_dead_reckoning_2d.hpp").c_str());
    const auto backend_header = read_file(repo_path(
        "include/robot_slamd/autonomy/real_adapters/slam_backend/"
        "sparse_multi_tof_occupancy_backend.hpp").c_str());
    const auto closed_loop = read_file(repo_path(
        "tests/test_m3d1_closed_loop_occupancy.cpp").c_str());

    expect(!contains(estimator_header, "SimRobotPlant"),
           "estimator does not reference SimRobotPlant");
    expect(!contains(estimator_header, "FakeWorld2D"),
           "estimator does not reference FakeWorld2D");
    expect(!contains(estimator_header, "AlgorithmMotion"),
           "estimator API does not accept algorithm commands");
    expect(!contains(estimator_header, "SoftwareMotion"),
           "estimator API does not accept software commands");

    expect(!contains(backend_header, "sim_robot_plant.hpp"),
           "backend does not include sim robot plant");
    expect(!contains(backend_header, "fake_world_2d.hpp"),
           "backend does not include fake world");
    expect(!contains(backend_header, "SimRobotPlant"),
           "backend does not reference SimRobotPlant");
    expect(!contains(backend_header, "FakeWorld2D"),
           "backend does not reference FakeWorld2D");

    expect(contains(closed_loop, "WheelImuDeadReckoning2D"),
           "closed-loop test uses wheel/IMU estimator");
    expect(contains(closed_loop, "odometry.estimated_pose()"),
           "predicted pose comes from estimator");
    expect(contains(closed_loop, "odom_wheel.sample"),
           "closed-loop odometry samples wheel measurement");
    expect(contains(closed_loop, "odom_imu.sample"),
           "closed-loop odometry samples IMU measurement");
    expect(!contains(closed_loop, "m3d1_command_dead_reckoning_estimate"),
           "closed-loop no longer labels command-derived pose");
    expect(!contains(closed_loop, "estimate_yaw_rate"),
           "closed-loop has no oracle yaw-rate integrator");
    expect(!contains(closed_loop, "estimate_turn_active"),
           "closed-loop has no oracle turn latch");

    robot_slamd::WheelImuDeadReckoningConfig config;
    config.maximum_dt_s = 1.0;
    config.maximum_yaw_rate_disagreement_rad_s = 0.01;
    robot_slamd::WheelImuDeadReckoning2D estimator_a(config);
    robot_slamd::WheelImuDeadReckoning2D estimator_b(config);
    expect(estimator_a.reset().ok, "estimator_a reset");
    expect(estimator_b.reset().ok, "estimator_b reset");
    expect(estimator_a.update(wheel(1.0, 0.0, 0.0), imu(1.0, 0.0)).ok,
           "estimator_a baseline");
    expect(estimator_b.update(wheel(1.0, 0.0, 0.0), imu(1.0, 0.0)).ok,
           "estimator_b baseline");
    expect(estimator_a.update(wheel(1.1, 0.0, 0.5), imu(1.1, 0.5)).ok,
           "estimator_a wheel turn");
    expect(estimator_b.update(wheel(1.1, 0.0, 0.5), imu(1.1, 0.5)).ok,
           "estimator_b wheel turn");
    expect(std::fabs(estimator_a.estimated_pose().yaw_rad -
                     estimator_b.estimated_pose().yaw_rad) < 1e-12,
           "same wheel/IMU sequence gives same predicted pose");

    robot_slamd::WheelImuDeadReckoning2D stationary(config);
    expect(stationary.reset().ok, "stationary reset");
    expect(stationary.update(wheel(2.0, 0.0, 0.0), imu(2.0, 0.0)).ok,
           "stationary baseline");
    expect(stationary.update(wheel(2.1, 0.0, 0.0), imu(2.1, 0.0)).ok,
           "stationary wheel unchanged");
    expect(std::fabs(stationary.estimated_pose().yaw_rad) < 1e-12,
           "pose does not turn without Wheel/IMU motion");

    return failures ? 1 : 0;
}
