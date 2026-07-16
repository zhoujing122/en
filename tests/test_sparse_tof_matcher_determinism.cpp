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
    SparseTofLocalScanMatcher matcher(cfg);
    RobotPose2D pose;
    pose.x_m = 0.04;
    pose.yaw_rad = -0.05;
    const auto bundle = d2test::bundle_at(pose);

    const auto a = matcher.match(map, bundle, RobotPose2D{});
    const auto b = matcher.match(map, bundle, RobotPose2D{});
    if (a.accepted != b.accepted || a.fault != b.fault || a.candidate_count != b.candidate_count) {
        return fail("matcher result status was not deterministic");
    }
    if (std::fabs(a.best.map_from_odom.x_m - b.best.map_from_odom.x_m) > 1e-12 ||
        std::fabs(a.best.map_from_odom.y_m - b.best.map_from_odom.y_m) > 1e-12 ||
        std::fabs(a.best.map_from_odom.yaw_rad - b.best.map_from_odom.yaw_rad) > 1e-12 ||
        a.best.raw_score != b.best.raw_score) {
        return fail("best candidate was not deterministic");
    }

    return 0;
}
