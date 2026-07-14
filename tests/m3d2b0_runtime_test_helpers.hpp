#pragma once

#include "m3d2a1_runtime_test_helpers.hpp"

namespace m3d2b0_test {

using namespace robot_slamd;
using m3d2a1_test::expect;
using m3d2a1_test::request;
using m3d2a1_test::snapshot;

inline void freeze_ready_bundle(SparseSlamRuntimeCore &core) {
    core.step(snapshot(0.2, 0.0), 0.2);
    core.step(request(snapshot(0.3, 0.05),
                      ActiveObservationControl::BeginCollection));
    core.step(request(snapshot(0.4, 0.0),
                      ActiveObservationControl::EndMotionAndWaitForSettle));
    core.step(snapshot(0.51, 0.0), 0.51);
    core.step(snapshot(0.62, 0.0), 0.62);
}

} // namespace m3d2b0_test
