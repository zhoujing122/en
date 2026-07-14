#include "sparse_tof_local_match_test_helpers.hpp"

#include <limits>

int main() {
    using namespace robot_slamd;
    using namespace local_match_test;
    SparseTofLocalMatchInputValidator validator;

    auto input = valid_input();
    auto ready = validator.validate(input);
    expect(ready.ready &&
               ready.status == SparseTofLocalMatchInputStatus::Ready,
           "valid input ready");
    expect(validator.validate(input).reason == ready.reason,
           "validation deterministic");

    auto missing_bundle = input;
    missing_bundle.frozen_bundle.reset();
    expect(validator.validate(missing_bundle).status ==
               SparseTofLocalMatchInputStatus::BundleMissing,
           "missing bundle rejected");

    auto not_frozen = input;
    not_frozen.frozen_bundle =
        std::make_shared<const FrozenMultiTofObservationBundle>();
    expect(validator.validate(not_frozen).status ==
               SparseTofLocalMatchInputStatus::BundleNotFrozen,
           "not frozen bundle rejected");

    auto missing_map = input;
    missing_map.reference_map.reset();
    expect(validator.validate(missing_map).status ==
               SparseTofLocalMatchInputStatus::ReferenceMapMissing,
           "missing map rejected");

    auto revision = input;
    revision.expected_reference_map_revision = 4;
    expect(validator.validate(revision).status ==
               SparseTofLocalMatchInputStatus::ReferenceRevisionMismatch,
           "revision mismatch rejected");

    auto prediction = input;
    prediction.predicted_map_from_odom.map_T_odom.x_m =
        std::numeric_limits<double>::quiet_NaN();
    expect(validator.validate(prediction).status ==
               SparseTofLocalMatchInputStatus::PredictionInvalid,
           "invalid prediction rejected");

    auto timestamp = input;
    timestamp.match_request_timestamp_s = 1.0;
    expect(validator.validate(timestamp).status ==
               SparseTofLocalMatchInputStatus::TimestampInvalid,
           "early request timestamp rejected");

    auto sparse = input;
    sparse.config.min_reference_cells = 20;
    sparse.config.min_reference_occupied_cells = 1;
    sparse.config.min_reference_free_cells = 1;
    expect(validator.validate(sparse).status ==
               SparseTofLocalMatchInputStatus::ReferenceMapTooSparse,
           "sparse reference rejected");
    return 0;
}
