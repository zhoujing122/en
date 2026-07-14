#include "robot_slamd/runtime/sparse_shadow_runtime.hpp"
#include <stdexcept>
#include <string>

int main() {
    robot_slamd::SparseShadowRuntimeReport report;
    report.matcher_attempt_count = 1;
    report.matcher_execution_count = 1;
    report.matcher_evaluated_candidate_count = 17;
    report.matcher_status = "accepted_yaw_only";
    report.matcher_proposal_ready = true;
    report.map_from_odom_before_match = {0.1, -0.2, 0.3};
    report.map_from_odom_after_match = report.map_from_odom_before_match;
    const std::string text = robot_slamd::write_sparse_shadow_runtime_report(report);
    if (text.find("matcher_execution_count=1") == std::string::npos ||
        text.find("matcher_evaluated_candidate_count=17") == std::string::npos ||
        text.find("matcher_status=accepted_yaw_only") == std::string::npos ||
        text.find("map_from_odom_after_match_yaw_rad=0.3") == std::string::npos) {
        throw std::runtime_error("shadow report missing matcher fields");
    }
    return 0;
}
