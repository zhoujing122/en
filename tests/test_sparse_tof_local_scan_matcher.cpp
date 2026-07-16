#include "sparse_tof_d2_test_helpers.hpp"

#include <cmath>
#include <iostream>

namespace {

int fail(const char *message) {
    std::cerr << message << "\n";
    return 1;
}

} // namespace

int main() {
    using namespace robot_slamd;

    const auto map = d2test::reference_map();
    auto cfg = d2test::matcher_config();
    cfg.score.minimum_score_margin = -1.0;
    cfg.score.minimum_normalized_score = -0.2;
    SparseTofLocalScanMatcher matcher(cfg);
    if (!matcher.valid()) {
        return fail("matcher config should be valid");
    }

    RobotPose2D true_pose;
    RobotPose2D drifted = true_pose;
    drifted.yaw_rad = 0.08;
    const auto bundle = d2test::bundle_at(drifted);
    const auto result = matcher.match(map, bundle, RobotPose2D{});
    if (!result.accepted) {
        return fail("small yaw drift should be accepted");
    }
    if (std::fabs(result.best.map_from_odom.yaw_rad + 0.08) > 0.06) {
        return fail("matcher did not recover yaw drift direction");
    }
    if (result.candidate_count == 0 || !result.bounded_search_verified) {
        return fail("bounded candidate search was not reported");
    }

    RobotPose2D far = true_pose;
    far.yaw_rad = 0.5;
    auto strict_cfg = cfg;
    strict_cfg.score.minimum_normalized_score = 1.01;
    SparseTofLocalScanMatcher strict_matcher(strict_cfg);
    const auto rejected = strict_matcher.match(map, d2test::bundle_at(far), RobotPose2D{});
    if (rejected.accepted) {
        return fail("strict gate should reject out-of-window yaw drift");
    }

    return 0;
}
