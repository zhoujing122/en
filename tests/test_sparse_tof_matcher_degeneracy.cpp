#include "sparse_tof_d2_test_helpers.hpp"

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
    RobotPose2D zero;
    const auto bundle = d2test::bundle_at(zero);

    auto margin_cfg = d2test::matcher_config();
    margin_cfg.score.minimum_score_margin = 0.90;
    SparseTofLocalScanMatcher margin_matcher(margin_cfg);
    const auto ambiguous = margin_matcher.match(map, bundle, zero);
    if (ambiguous.accepted || ambiguous.fault != SparseTofMatcherFault::ScoreMarginTooSmall) {
        return fail("high margin gate should reject ambiguous match");
    }

    auto obs_cfg = d2test::matcher_config();
    obs_cfg.score.minimum_score_margin = -1.0;
    obs_cfg.score.minimum_observability_contrast = 2.0;
    SparseTofLocalScanMatcher obs_matcher(obs_cfg);
    const auto unobservable = obs_matcher.match(map, bundle, zero);
    if (unobservable.accepted || unobservable.fault != SparseTofMatcherFault::YawUnobservable) {
        return fail("unobservable yaw should reject match");
    }

    SparseOccupancyGridSnapshot empty;
    empty.resolution_m = margin_cfg.grid.resolution_m;
    SparseTofLocalScanMatcher matcher(d2test::matcher_config());
    const auto no_ref = matcher.match(empty, bundle, zero);
    if (no_ref.accepted || no_ref.fault != SparseTofMatcherFault::ReferenceMapUnavailable) {
        return fail("empty reference map should not self-match");
    }

    return 0;
}
