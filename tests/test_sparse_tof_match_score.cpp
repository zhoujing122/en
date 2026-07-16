#include "sparse_tof_d2_test_helpers.hpp"

#include <cmath>
#include <iostream>
#include <vector>

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
    SparseTofLocalScanMatcher matcher(cfg);
    RobotPose2D zero;
    const auto bundle = d2test::bundle_at(zero);

    const auto perfect = matcher.score_candidate(map, bundle, zero);
    RobotPose2D shifted;
    shifted.x_m = 0.12;
    const auto off = matcher.score_candidate(map, bundle, shifted);
    if (!(perfect.normalized_score > off.normalized_score)) {
        return fail("perfect candidate did not outscore shifted candidate");
    }
    if (perfect.hit_endpoint_agreement_count == 0 || perfect.known_cell_count == 0) {
        return fail("hit endpoint or known cells were not counted");
    }

    SparseTofObservationBundle no_return_bundle;
    auto obs = d2test::observation(20.0, zero);
    obs.rays[0] = d2test::ray(ScalarTofReturnKind::NoReturn, zero, 0.0, 0.9);
    obs.rays[1] = d2test::ray(ScalarTofReturnKind::Invalid, zero, 1.5707963267948966, 0.7);
    obs.rays[2] = d2test::ray(ScalarTofReturnKind::Invalid, zero, -1.5707963267948966, 0.7);
    no_return_bundle.add(obs);
    const auto no_return_score = matcher.score_candidate(map, no_return_bundle, zero);
    if (no_return_score.evaluated_ray_count != 1) {
        return fail("NoReturn should evaluate one free-space ray");
    }

    SparseTofObservationBundle invalid_bundle;
    auto invalid_obs = d2test::observation(30.0, zero);
    for (auto &ray : invalid_obs.rays) {
        ray.return_kind = ScalarTofReturnKind::Invalid;
        ray.usable_for_mapping = false;
    }
    invalid_bundle.add(invalid_obs);
    const auto invalid_score = matcher.score_candidate(map, invalid_bundle, zero);
    if (invalid_score.evaluated_ray_count != 0 || invalid_score.raw_score != 0) {
        return fail("Invalid rays changed score");
    }

    SparseOccupancyGridSnapshot empty;
    empty.resolution_m = cfg.grid.resolution_m;
    const auto unknown_score = matcher.score_candidate(empty, bundle, zero);
    if (unknown_score.known_cell_count != 0 || unknown_score.normalized_score > 0.0) {
        return fail("unknown map produced known cells or positive score");
    }

    return 0;
}
