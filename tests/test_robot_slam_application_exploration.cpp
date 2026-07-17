#include "robot_slamd/app/app.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

std::string read_file(const std::string &path) {
    std::ifstream input(path);
    std::ostringstream content;
    content << input.rdbuf();
    return content.str();
}

} // namespace

int main() {
    const std::string config_path =
        std::string(ROBOT_SLAMD_SOURCE_DIR) +
        "/config/sparse_sim_autonomous_exploration.yaml";
    const std::string output_dir = "/tmp/en_m3e_formal_main_exploration";
    std::string program = "robot_slamd";
    std::string config_arg = "--config";
    std::string output_arg = "--output-dir";
    char *argv[] = {program.data(), config_arg.data(),
                    const_cast<char *>(config_path.c_str()), output_arg.data(),
                    const_cast<char *>(output_dir.c_str())};

    expect(robot_slamd::real_main(5, argv) == 0,
           "formal real_main simulation exploration smoke succeeds");
    const auto report = read_file(output_dir + "/latest/exploration_report.txt");
    expect(report.find("ok=1") != std::string::npos,
           "formal exploration report is successful");
    expect(report.find("state=completed") != std::string::npos &&
               report.find("termination_reason=no_reachable_frontier") !=
                   std::string::npos,
           "formal exploration keeps no-reachable-frontier termination");
    expect(report.find("application_core_step_count=") != std::string::npos &&
               report.find("runtime_core_constructed=1") != std::string::npos &&
               report.find("runtime_core_sparse_backend_constructed=1") !=
                   std::string::npos,
           "formal exploration report proves the unified Application/Core path");
    expect(report.find("ground_truth_isolation_verified=1") !=
               std::string::npos &&
               report.find("collision=0") != std::string::npos,
           "formal exploration keeps ground truth isolated and collision free");
    expect(report.find("completed_goals=0") == std::string::npos &&
               report.find("map_revision=") != std::string::npos &&
               report.find("sparse_map_cell_count=") != std::string::npos,
           "formal exploration grows frontier completion and sparse map state");
    return 0;
}
