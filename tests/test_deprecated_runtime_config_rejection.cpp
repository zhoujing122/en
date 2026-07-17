#include "robot_slamd/config/config.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; }
}
void expect_rejected(const std::string &path, const std::string &yaml,
                     const char *message) {
    std::ofstream(path) << yaml;
    try {
        (void)robot_slamd::load_config(path, "");
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    } catch (const std::exception &e) {
        expect(std::string(e.what()).find("deprecated_runtime_mode_field") !=
                   std::string::npos,
               "deprecated configuration reports a stable failure");
    }
}
}

int main() {
    expect_rejected("/tmp/robot_slamd_deprecated_runtime_mode_smoke.yaml",
                    "runtime:\n  slam_runtime_mode: legacy\n",
                    "runtime.slam_runtime_mode is rejected");
    expect_rejected("/tmp/robot_slamd_deprecated_slam_runtime_smoke.yaml",
                    "slam_runtime:\n  mode: sparse_shadow\n",
                    "slam_runtime.mode is rejected");
    return failures ? 1 : 0;
}
