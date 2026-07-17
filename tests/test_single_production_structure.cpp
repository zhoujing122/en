#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int failures = 0;

std::string read_file(const std::string &path) {
    std::ifstream input(path);
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
}

std::size_t count_of(const std::string &text, const std::string &needle) {
    std::size_t count = 0;
    std::size_t offset = 0;
    while ((offset = text.find(needle, offset)) != std::string::npos) {
        ++count;
        offset += needle.size();
    }
    return count;
}

void expect(bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        ++failures;
    }
}

void expect_absent(const std::string &text, const std::string &needle,
                   const std::string &message) {
    expect(text.find(needle) == std::string::npos, message);
}

} // namespace

int main() {
#ifndef ROBOT_SLAMD_SOURCE_DIR
    std::cerr << "FAIL: source directory was not provided\n";
    return 1;
#else
    const std::string root = ROBOT_SLAMD_SOURCE_DIR;
    const std::string main = read_file(root + "/src/main.cpp");
    const std::string app_entry = read_file(root + "/include/robot_slamd/app/app.hpp");
    const std::string application =
        read_file(root + "/include/robot_slamd/app/robot_slam_application.hpp");
    const std::string motion_boundary =
        read_file(root + "/include/robot_slamd/app/robot_slam_motion_boundary.hpp");
    const std::string core =
        read_file(root + "/include/robot_slamd/runtime/sparse_slam_runtime_core.hpp");
    const std::string simulation =
        read_file(root +
                  "/include/robot_slamd/simulation/application/"
                  "simulation_robot_slam_runner.hpp");
    const std::string simulation_sensor =
        read_file(root + "/include/robot_slamd/simulation/ports/sim_sensor_port.hpp");
    const std::string replay =
        read_file(root +
                  "/include/robot_slamd/autonomy/real_adapters/multi_tof/"
                  "multi_tof_replay_sensor_port.hpp");
    const std::string cmake = read_file(root + "/CMakeLists.txt");

    for (const auto &source : {main, app_entry}) {
        expect_absent(source, "SparseShadowRuntime",
                      "formal entry cannot name SparseShadowRuntime");
        expect_absent(source, "SlamRuntimeMode",
                      "formal entry cannot name SlamRuntimeMode");
        expect_absent(source, "Localizer",
                      "formal entry cannot construct Localizer");
        expect_absent(source, "ChunkedGrid",
                      "formal entry cannot construct ChunkedGrid");
        expect_absent(source, "TofPoseCorrector",
                      "formal entry cannot construct old correction");
        expect_absent(source, "SparseScanCollector",
                      "formal entry cannot construct old scan collector");
        expect_absent(source, "SparseScanYawMatcher",
                      "formal entry cannot construct old scan matcher");
        expect_absent(source, "RecoveryManager",
                      "formal entry cannot construct old recovery");
    }

    expect(count_of(main, "robot_slamd/app/app.hpp") == 1,
           "executable has one application composition include");
    expect(count_of(application, "SparseSlamRuntimeCore core_;") == 1,
           "Application has one SparseSlamRuntimeCore member");
    expect(count_of(application, "RobotSlamMotionBoundary> motion_boundary_") == 1,
           "Application has one motion boundary member");
    expect(count_of(core, "WheelImuDeadReckoning2D estimator_;") == 1,
           "Core has one wheel/IMU estimator");
    expect(count_of(core, "TimedOdomPoseBuffer pose_buffer_;") == 1,
           "Core has one timed pose buffer");
    expect(count_of(core, "MapOdomFrameState frame_state_;") == 1,
           "Core has one map/odom owner");
    expect(count_of(core, "SparseTofLocalMatcher local_matcher_;") == 1,
           "Core has one local matcher/scoring owner");
    expect(count_of(core, "SparseTofKeyframeManager keyframe_manager_;") == 1,
           "Core has one keyframe owner");
    expect(count_of(core,
                    "std::shared_ptr<SparseMultiTofOccupancyBackendBinding> "
                    "sparse_backend_;") == 1,
           "Core has one writable sparse map owner");

    expect_absent(simulation, "SparseSlamRuntimeCore core",
                  "simulation adapter does not construct a second Core");
    expect_absent(simulation, "WheelImuDeadReckoning2D",
                  "simulation adapter does not construct odometry");
    expect_absent(simulation, "SparseTofLocalMatcher",
                  "simulation adapter does not construct a matcher");
    expect_absent(simulation_sensor, "ground_truth",
                  "sensor adapter does not publish ground truth");
    expect_absent(core, "ground_truth",
                  "Core has no ground-truth input or state");
    expect_absent(core, "commanded_speed",
                  "Core has no commanded-speed odometry path");
    expect_absent(core, "SoftwareMotionCommand",
                  "Core does not consume motion commands as odometry");

    expect(count_of(simulation_sensor, "RobotSlamSensorSnapshot") > 0,
           "simulation publishes canonical sensor snapshots");
    expect(count_of(replay, "RobotSlamSensorSnapshot") > 0,
           "replay publishes canonical sensor snapshots");
    expect_absent(cmake, "SparseShadowRuntime",
                  "CMake has no SparseShadow runtime target");
    expect_absent(cmake, "SlamRuntimeMode",
                  "CMake has no legacy runtime mode target");
    expect_absent(cmake, "test_sparse_shadow",
                  "CMake has no SparseShadow tests");
    expect_absent(cmake, "test_slam_runtime_mode_config",
                  "CMake has no legacy mode behavior test");

    const std::string real_main_contract =
        "real sensor adapter unavailable; startup rejected fail-closed";
    expect(app_entry.find(real_main_contract) != std::string::npos,
           "real source is explicitly fail-closed at formal entry");
    expect_absent(app_entry, "LoopbackSoftwareMotion",
                  "formal real entry does not use loopback motion");
    expect_absent(app_entry, "FakeMotion",
                  "formal real entry does not use fake motion");
    expect_absent(motion_boundary, "WheelOdomFrame",
                  "motion boundary does not generate wheel odometry");

    return failures == 0 ? 0 : 1;
#endif
}
