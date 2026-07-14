#include "sparse_tof_yaw_match_test_helpers.hpp"
#include <cmath>

int main() {
    using namespace yaw_match_test;
    auto frames = asymmetric_frames(true, false);
    auto with_no_return = input_from_frames(frames, 0.08);
    SparseTofLocalMatcher matcher;
    const auto constrained = matcher.match(with_no_return);
    expect(constrained.matcher_executed && constrained.evaluated_candidate_count > 0,
           "no-return scene evaluated");
    expect(constrained.best_metrics.no_return_ray_count > 0,
           "explicit no-return rays used");
    expect(constrained.best_metrics.used_ray_count ==
               constrained.best_metrics.hit_ray_count +
               constrained.best_metrics.no_return_ray_count,
           "used rays preserve return kinds");

    for (auto &frame : frames) {
        if (frame.right.return_kind == ScalarTofReturnKind::NoReturn) {
            frame.right.return_kind = ScalarTofReturnKind::Invalid;
            frame.right.frame.distance_m = 0.001;
            frame.right.frame.confidence = 0;
        }
    }
    auto without_no_return = input_from_frames(frames, 0.08);
    without_no_return.reference_map = with_no_return.reference_map;
    const auto unconstrained = matcher.match(without_no_return);
    expect(unconstrained.best_metrics.no_return_ray_count == 0,
           "invalid rays do not become no-return");
    expect(constrained.best_metrics.traversed_cell_count !=
               unconstrained.best_metrics.traversed_cell_count ||
               std::fabs(*constrained.best_score - *unconstrained.best_score) > 1e-12,
           "removing no-return changes constraints");

    auto ignored_frames = asymmetric_frames(false, true);
    auto ignored_input = input_from_frames(ignored_frames, 0.08);
    const auto ignored = matcher.match(ignored_input);
    expect(ignored.best_metrics.ignored_ray_count >= 2,
           "invalid and unspecified ignored");
    expect(ignored.best_metrics.used_ray_count ==
               ignored.best_metrics.hit_ray_count +
               ignored.best_metrics.no_return_ray_count,
           "ignored returns not scored");
    return 0;
}
