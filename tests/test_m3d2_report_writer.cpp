#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <iostream>
#include <sstream>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    robot_slamd::SparseTofLocalSlamPipeline pipeline(d2pipe::config());
    pipeline.reset_startup_zero();
    if (!d2pipe::bootstrap(pipeline)) return fail("bootstrap failed");
    const auto &report = pipeline.report();
    std::ostringstream out;
    out << "initial_yaw_measured_by_imu=" << report.initial_yaw_measured_by_imu << "\n";
    out << "native_multi_tof_consumed=" << report.native_multi_tof_consumed << "\n";
    out << "legacy_projection_consumed=" << report.legacy_projection_consumed << "\n";
    out << "self_matching_prevented=" << report.self_matching_prevented << "\n";
    const std::string text = out.str();
    if (text.find("initial_yaw_measured_by_imu=0") == std::string::npos ||
        text.find("native_multi_tof_consumed=1") == std::string::npos ||
        text.find("legacy_projection_consumed=0") == std::string::npos ||
        text.find("self_matching_prevented=1") == std::string::npos) {
        return fail("report text missing required M3-D2 fields");
    }
    return 0;
}
