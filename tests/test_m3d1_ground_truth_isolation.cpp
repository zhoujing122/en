#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

bool file_contains(const char *path, const char *needle) {
    std::ifstream in(path);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str().find(needle) != std::string::npos;
}
}

int main() {
    const char *backend =
        "include/robot_slamd/autonomy/real_adapters/slam_backend/"
        "sparse_multi_tof_occupancy_backend.hpp";
    expect(!file_contains(backend, "sim_robot_plant.hpp"),
           "backend does not include SimRobotPlant");
    expect(!file_contains(backend, "fake_world_2d.hpp"),
           "backend does not include FakeWorld2D");
    expect(!file_contains(backend, "SimRobotPlant"),
           "backend does not reference SimRobotPlant");
    expect(!file_contains(backend, "FakeWorld2D"),
           "backend does not reference FakeWorld2D");

    robot_slamd::SparseMultiTofOccupancyBackendBinding backend_instance;
    expect(!backend_instance.report().ground_truth_consumed,
           "runtime report ground truth false");
    expect(backend_instance.report().ground_truth_isolation_verified,
           "ground truth isolation marked verified");
    return failures ? 1 : 0;
}
