#include "sparse_tof_d2_pipeline_helpers.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace { int fail(const char *m) { std::cerr << m << "\n"; return 1; } }

int main() {
    const std::string marker = "/tests/test_m3d2_ground_truth_isolation.cpp";
    const std::string this_file = __FILE__;
    const auto marker_pos = this_file.rfind(marker);
    if (marker_pos == std::string::npos) {
        return fail("cannot derive source root for isolation audit");
    }
    const std::string source_root = this_file.substr(0, marker_pos);
    const char *paths[] = {
        "include/robot_slamd/localization/sparse_tof/sparse_tof_local_scan_matcher.hpp",
        "include/robot_slamd/localization/sparse_tof/sparse_tof_pose_correction_state.hpp",
        "include/robot_slamd/localization/sparse_tof/sparse_tof_local_slam_pipeline.hpp",
        "include/robot_slamd/mapping/sparse_tof/sparse_tof_keyframe_manager.hpp"
    };
    for (const char *path : paths) {
        std::ifstream in(source_root + "/" + path);
        if (!in) {
            return fail("missing local SLAM source file for isolation audit");
        }
        std::stringstream buffer;
        buffer << in.rdbuf();
        const std::string text = buffer.str();
        if (text.find("SimRobotPlant") != std::string::npos || text.find("FakeWorld2D") != std::string::npos) {
            return fail("local SLAM algorithm includes simulation ground truth type");
        }
    }

    robot_slamd::SparseTofLocalSlamPipeline a(d2pipe::config());
    robot_slamd::SparseTofLocalSlamPipeline b(d2pipe::config());
    a.reset_startup_zero();
    b.reset_startup_zero();
    robot_slamd::RobotPose2D pose;
    auto out_a = a.process(d2pipe::input(7.0, pose, false, true));
    auto out_b = b.process(d2pipe::input(7.0, pose, false, true));
    if (out_a.corrected_map_pose.x_m != out_b.corrected_map_pose.x_m || out_a.message != out_b.message) {
        return fail("identical algorithm input produced different output");
    }
    if (a.report().matcher_used_ground_truth || a.report().matcher_used_fake_world || a.report().command_derived_pose_used) {
        return fail("report indicates ground truth or command-derived pose usage");
    }
    return 0;
}
