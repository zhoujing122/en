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
    invalid.sparse_slam_local_match_max_total_candidates = 1025;
    expect(rejects(invalid), "runtime candidate budget hard cap enforced");
    invalid = config;
    invalid.sparse_slam_local_match_max_coarse_candidates =
        invalid.sparse_slam_local_match_max_total_candidates + 1;
    expect(rejects(invalid), "coarse budget bounded by total");
    invalid = config;
    invalid.sparse_slam_local_match_max_unknown_ratio = 1.1;
    expect(rejects(invalid), "unknown ratio bounded");
    invalid = config;
    invalid.sparse_slam_local_match_free_contradiction_weight = 1.0;
    expect(rejects(invalid), "contradiction weight must penalize");
    invalid = config;
    invalid.sparse_slam_local_match_max_abs_translation_x_m = 0.1;
    expect(rejects(invalid), "yaw only translation ambiguity rejected");

    const std::string path = "/tmp/en_m3d2b1_match_config_resolved.yaml";
    write_resolved_config(config, path);
    std::ifstream in(path);
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    expect(text.find("local_match_mode: yaw_only") != std::string::npos,
           "resolved mode written");
    expect(text.find("local_match_max_candidate_count: 256") !=
               std::string::npos,
           "resolved candidate cap written");
    expect(text.find("local_match_max_total_candidates: 128") !=
               std::string::npos,
           "resolved runtime candidate budget written");
    expect(text.find("local_match_max_cells_per_ray: 256") !=
               std::string::npos,
           "resolved ray cell budget written");
    expect(text.find("local_match_runner_up_exclusion_yaw_rad:") !=
               std::string::npos,
           "resolved runner-up exclusion written");
    return 0;
}
