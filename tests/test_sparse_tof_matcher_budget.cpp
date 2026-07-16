#include "sparse_tof_d2_test_helpers.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

int fail(const char *message) {
    std::cerr << message << "\n";
    return 1;
}

} // namespace

int main() {
    using namespace robot_slamd;

    auto budget_cfg = d2test::matcher_config();
    budget_cfg.search.maximum_candidate_count = 4;
    SparseTofLocalScanMatcher budget_matcher(budget_cfg);
    const auto budget_result = budget_matcher.match(d2test::reference_map(), d2test::bundle_at(RobotPose2D{}), RobotPose2D{});
    if (budget_result.accepted || budget_result.fault != SparseTofMatcherFault::CandidateBudgetExceeded) {
        return fail("candidate budget did not reject oversized search");
    }

    auto zero_step = d2test::matcher_config();
    zero_step.search.fine_yaw_step_rad = 0.0;
    if (SparseTofLocalScanMatcher(zero_step).valid()) {
        return fail("zero yaw step should be invalid");
    }

    auto nan_cfg = d2test::matcher_config();
    nan_cfg.search.coarse_yaw_step_rad = std::numeric_limits<double>::quiet_NaN();
    if (SparseTofLocalScanMatcher(nan_cfg).valid()) {
        return fail("NaN search config should be invalid");
    }

    return 0;
}
