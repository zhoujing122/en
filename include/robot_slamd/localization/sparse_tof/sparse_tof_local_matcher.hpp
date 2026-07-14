#pragma once

#include "robot_slamd/localization/sparse_tof/sparse_tof_local_match_input_validator.hpp"

namespace robot_slamd {

class SparseTofLocalMatcher {
public:
    SparseTofLocalMatchResult match(
        const SparseTofLocalMatchInput &input) const {
        SparseTofLocalMatchResult result;
        result.bundle_id = input.bundle_id;
        result.reference_map_revision =
            input.expected_reference_map_revision;
        result.mode = input.config.mode;
        result.predicted_map_from_odom =
            input.predicted_map_from_odom;
        result.status = SparseTofLocalMatchStatus::NotImplemented;
        result.matcher_executed = false;
        result.accepted = false;
        result.evaluated_candidate_count = 0;
        result.reason = "sparse_tof_local_matcher_not_implemented_in_b0";
        return result;
    }
};

} // namespace robot_slamd
