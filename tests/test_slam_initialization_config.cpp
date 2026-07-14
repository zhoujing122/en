#include "robot_slamd/config/config.hpp"

#include <fstream>
#include <iostream>
#include <limits>

namespace {

void expect(bool ok, const char *message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

bool rejects(robot_slamd::Config cfg) {
    try {
        robot_slamd::validate_config(cfg);
    } catch (const std::exception &) {
        return true;
    }
    return false;
}

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)),
                       std::istreambuf_iterator<char>());
}

} // namespace

int main() {
    robot_slamd::Config cfg;
    expect(cfg.sparse_slam_map_startup_mode == "create_new",
           "default map startup create_new");
    expect(cfg.sparse_slam_initial_pose_mode == "startup_zero",
           "default initial pose startup_zero");
    expect(cfg.sparse_slam_enable_atomic_local_slam_commit,
           "atomic local slam commit enabled");
    expect(cfg.sparse_slam_rejected_match_requires_discard,
           "rejected match requires explicit discard");
    robot_slamd::validate_config(cfg);

    cfg.sparse_slam_initial_pose_mode = "configured_pose";
    expect(rejects(cfg), "configured pose requires explicit flag");
    cfg.sparse_slam_has_configured_pose = true;
    cfg.sparse_slam_configured_pose_x_m = 1.0;
    cfg.sparse_slam_configured_pose_y_m = -2.0;
    cfg.sparse_slam_configured_pose_yaw_rad = 0.5;
    robot_slamd::validate_config(cfg);

    robot_slamd::Config bad;
    bad.sparse_slam_map_startup_mode = "new";
    expect(rejects(bad), "invalid startup mode rejects");
    bad = robot_slamd::Config{};
    bad.sparse_slam_initial_pose_mode = "ConfiguredPose";
    expect(rejects(bad), "invalid initial pose mode rejects");
    bad = robot_slamd::Config{};
    bad.sparse_slam_has_configured_pose = true;
    expect(rejects(bad), "startup zero with configured pose rejects");
    bad = robot_slamd::Config{};
    bad.sparse_slam_configured_pose_yaw_rad =
        std::numeric_limits<double>::quiet_NaN();
    expect(rejects(bad), "non-finite configured yaw rejects");
    bad = robot_slamd::Config{};
    bad.sparse_slam_max_keyframes = 0;
    expect(rejects(bad), "zero keyframe capacity rejects");
    bad = robot_slamd::Config{};
    bad.sparse_slam_max_cells_per_keyframe_transaction = 0;
    expect(rejects(bad), "zero transaction capacity rejects");
    bad = robot_slamd::Config{};
    bad.sparse_slam_rejected_match_requires_discard = false;
    expect(rejects(bad), "automatic recovery from rejected match rejects");
    bad = robot_slamd::Config{};
    bad.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    expect(rejects(bad), "formal sparse mode requires configured extrinsics");
    bad.sparse_slam_planar_tof_extrinsics_configured = true;
    bad.sparse_slam_front_tof_x_m = 0.12;
    bad.sparse_slam_left_tof_y_m = 0.09;
    bad.sparse_slam_right_tof_y_m = -0.09;
    robot_slamd::validate_config(bad);
    bad.sparse_slam_right_tof_yaw_rad =
        std::numeric_limits<double>::quiet_NaN();
    expect(rejects(bad), "non-finite planar extrinsic rejects");

    const std::string resolved = "/tmp/m3d2a0_slam_initialization_resolved.yaml";
    robot_slamd::write_resolved_config(cfg, resolved);
    const auto text = read_file(resolved);
    expect(text.find("sparse_slam:") != std::string::npos,
           "resolved sparse_slam section");
    expect(text.find("map_startup_mode: create_new") != std::string::npos,
           "resolved startup mode");
    expect(text.find("initial_pose_mode: configured_pose") != std::string::npos,
           "resolved initial pose mode");
    expect(text.find("has_configured_pose: true") != std::string::npos,
           "resolved configured flag");
    expect(text.find("enable_atomic_local_slam_commit: true") !=
               std::string::npos,
           "resolved atomic commit flag");
    expect(text.find("max_keyframes: 64") != std::string::npos,
           "resolved keyframe capacity");
    expect(text.find("planar_tof_extrinsics_configured:") !=
               std::string::npos &&
               text.find("front_tof_x_m:") != std::string::npos,
           "resolved planar extrinsic contract");

    return 0;
}
