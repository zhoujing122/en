#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {
std::string read(const std::string &path) {
    std::ifstream input(path);
    std::ostringstream content;
    content << input.rdbuf();
    return content.str();
}

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << "\n";
        std::exit(1);
    }
}
}

int main() {
    const std::string source = ROBOT_SLAMD_SOURCE_DIR;
    const auto controller = read(
        source + "/include/robot_slamd/exploration/"
                 "autonomous_exploration_controller.hpp");
    const auto planner = read(
        source + "/include/robot_slamd/exploration/"
                 "sparse_frontier_planner.hpp");
    const auto tracker = read(
        source + "/include/robot_slamd/exploration/safe_waypoint_tracker.hpp");
    const std::string algorithms = controller + planner + tracker;
    expect(!controller.empty() && !planner.empty() && !tracker.empty(),
           "algorithm sources are readable");
    for (const auto *forbidden : {"FakeWorld", "SimRobotPlant",
                                  "ground_truth", "simulation/"}) {
        expect(algorithms.find(forbidden) == std::string::npos,
               "exploration algorithms contain no simulation oracle");
    }
    expect(algorithms.find("current_map_pose") != std::string::npos &&
               algorithms.find("sparse_map_snapshot") != std::string::npos,
           "controller consumes estimated pose and sparse snapshot");
    return 0;
}
