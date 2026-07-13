
#include "robot_slamd/app/app.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {
int failures = 0;
void expect(bool condition, const char *message) { if (!condition) { std::cerr << "FAIL: " << message << "\n"; failures++; } }
std::string read_file(const std::string &path) { std::ifstream in(path); return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()); }
std::string read_latest_link(const std::string &path) { char buf[4096] = {}; const ssize_t n = readlink(path.c_str(), buf, sizeof(buf) - 1); return n <= 0 ? std::string() : std::string(buf, static_cast<std::size_t>(n)); }
}

int main() {
    const std::string root = "/tmp/m3d2a0_real_main_smoke";
    const std::string cfg_path = "/tmp/m3d2a0_real_main_smoke.yaml";
    { std::ofstream cfg(cfg_path); cfg << "slam_runtime:\n  mode: sparse_shadow\n  sparse_shadow_sensor_source: deterministic_simulation\nlogging:\n  output_dir: " << root << "\n"; }
    char arg0[] = "robot_slamd"; char arg1[] = "--config"; char arg2[160] = {}; std::snprintf(arg2, sizeof(arg2), "%s", cfg_path.c_str()); char arg3[] = "--duration-s"; char arg4[] = "0.10"; char *argv[] = {arg0, arg1, arg2, arg3, arg4};
    const int rc = robot_slamd::real_main(5, argv);
    expect(rc == 0, "real_main sparse branch returns success");
    const std::string latest = read_latest_link(root + "/latest");
    const auto report = read_file(latest + "/sparse_shadow_runtime_report.txt");
    expect(report.find("runtime_core_constructed=true") != std::string::npos, "core report reached real_main");
    expect(report.find("measurement_time_pose_consumed=true") != std::string::npos, "measurement poses consumed in real_main");
    expect(report.find("single_pose_fallback_consumed=false") != std::string::npos, "fallback disabled in real_main");
    expect(report.find("matcher_attempt_count=0") != std::string::npos, "matcher not started");
    expect(report.find("keyframe_attempt_count=0") != std::string::npos, "keyframe not started");
    return failures ? 1 : 0;
}
