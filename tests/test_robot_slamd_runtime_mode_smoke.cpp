#include "robot_slamd/app/app.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; }
}
std::string read_file(const std::string &path) {
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
std::string read_latest_link(const std::string &path) {
    char buf[4096] = {};
    const ssize_t n = readlink(path.c_str(), buf, sizeof(buf) - 1);
    if (n <= 0) return "";
    return std::string(buf, static_cast<std::size_t>(n));
}
}

int main() {
    const std::string root = "/tmp/m3d11_real_main_smoke";
    const std::string cfg_path = "/tmp/m3d11_real_main_smoke.yaml";
    {
        std::ofstream cfg(cfg_path);
        cfg << "slam_runtime:\n"
            << "  mode: sparse_shadow\n"
            << "  sparse_shadow_sensor_source: deterministic_simulation\n"
            << "logging:\n"
            << "  output_dir: " << root << "\n";
    }
    char arg0[] = "robot_slamd";
    char arg1[] = "--config";
    char arg2[128] = {};
    std::snprintf(arg2, sizeof(arg2), "%s", cfg_path.c_str());
    char arg3[] = "--duration-s";
    char arg4[] = "0.20";
    char *argv[] = {arg0, arg1, arg2, arg3, arg4};
    const int rc = robot_slamd::real_main(5, argv);
    expect(rc == 0, "real_main sparse shadow returns success");
    const std::string latest = read_latest_link(root + "/latest");
    expect(!latest.empty(), "latest run link exists");
    const auto report = read_file(latest + "/sparse_shadow_runtime_report.txt");
    expect(report.find("runtime_mode=sparse_shadow") != std::string::npos,
           "report proves sparse branch");
    expect(report.find("wheel_imu_estimator_constructed=true") != std::string::npos,
           "report proves estimator construction");
    expect(report.find("sparse_backend_constructed=true") != std::string::npos,
           "report proves sparse backend construction");
    expect(report.find("predicted_pose_handoff_count=0") == std::string::npos,
           "report proves pose handoff occurred");
    expect(report.find("native_multi_tof_consumed=true") != std::string::npos,
           "report proves native multi tof consumption");
    expect(report.find("legacy_projection_consumed=false") != std::string::npos,
           "report proves legacy projection not consumed");
    expect(report.find("old_chunked_grid_constructed=false") != std::string::npos,
           "report proves old grid not constructed");
    expect(report.find("old_matcher_constructed=false") != std::string::npos,
           "report proves old matcher not constructed");
    expect(report.find("old_correction_constructed=false") != std::string::npos,
           "report proves old correction not constructed");
    return failures ? 1 : 0;
}
