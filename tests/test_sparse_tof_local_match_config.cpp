#include "robot_slamd/config/config.hpp"
#include "robot_slamd/localization/sparse_tof/sparse_tof_local_match_input_validator.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

namespace {
void expect(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
bool rejects(robot_slamd::Config config) {
    try {
        robot_slamd::validate_config(config);
    } catch (...) {
        return true;
    }
    return false;
}
}

int main() {
    using namespace robot_slamd;
    Config config;
    validate_config(config);
    expect(config.sparse_slam_local_match_mode == "yaw_only",
           "config defaults yaw only");

    SparseTofLocalMatchConfig contract;
    expect(sparse_tof_local_match_config_valid(contract),
           "contract defaults valid");
    contract.mode = SparseTofLocalMatchMode::FullSE2;
    expect(sparse_tof_local_match_config_valid(contract),
           "full se2 contract expressible but not executed");

    Config invalid = config;
    invalid.sparse_slam_local_match_mode = "legacy";
    expect(rejects(invalid), "invalid mode string rejected");
    invalid = config;
    invalid.sparse_slam_local_match_fine_yaw_step_rad =
        invalid.sparse_slam_local_match_coarse_yaw_step_rad * 2.0;
    expect(rejects(invalid), "fine step greater than coarse rejected");
    invalid = config;
    invalid.sparse_slam_local_match_max_candidate_count = 100001;
    expect(rejects(invalid), "candidate hard cap enforced");
    invalid = config;
    invalid.sparse_slam_local_match_max_abs_translation_x_m = 0.1;
    expect(rejects(invalid), "yaw only translation ambiguity rejected");

    const std::string path = "/tmp/en_m3d2b0_match_config_resolved.yaml";
    write_resolved_config(config, path);
    std::ifstream in(path);
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    expect(text.find("local_match_mode: yaw_only") != std::string::npos,
           "resolved mode written");
    expect(text.find("local_match_max_candidate_count: 256") !=
               std::string::npos,
           "resolved candidate cap written");
    return 0;
}
