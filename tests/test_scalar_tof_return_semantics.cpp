#include "robot_slamd/mapping/sparse_tof/sparse_tof_observation_builder.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::ScalarTofSnapshotFrame frame(int distance_mm,
                                          int confidence,
                                          double timestamp_s = 100.0) {
    robot_slamd::ScalarTofSnapshotFrame out;
    out.distance_mm = static_cast<std::uint16_t>(distance_mm);
    out.distance_m = static_cast<double>(distance_mm) / 1000.0;
    out.confidence = static_cast<std::uint8_t>(confidence);
    out.echo_tag_u48 = 0x112233445566ULL;
    out.effective_timestamp_s = timestamp_s;
    out.protocol_valid = distance_mm >= 20 && distance_mm <= 12000 &&
                         confidence <= 100;
    out.usable_for_mapping = distance_mm >= 50 && distance_mm <= 12000 &&
                             confidence >= 70 && confidence <= 100;
    out.full_fov_rad = 1.6 * 3.14159265358979323846 / 180.0;
    out.frame_id = "tof_front_frame";
    out.source = "test";
    return out;
}

robot_slamd::SparseTofObservationBuildResult build(
    const robot_slamd::ScalarTofSnapshotFrame &tof,
    robot_slamd::ScalarTofReturnKind explicit_kind =
        robot_slamd::ScalarTofReturnKind::Unspecified,
    double now_s = 100.0,
    bool sync = true) {
    robot_slamd::SparseTofObservationBuildInput input;
    input.frame = tof;
    input.explicit_return_kind = explicit_kind;
    input.now_s = now_s;
    input.synchronized = sync;
    return robot_slamd::SparseTofObservationBuilder{}.build(input);
}
}

int main() {
    using namespace robot_slamd;
    expect(build(frame(1000, 100)).observation.return_kind ==
               ScalarTofReturnKind::Hit,
           "high confidence distance derives hit");
    expect(build(frame(1000, 0)).observation.return_kind ==
               ScalarTofReturnKind::Invalid,
           "confidence zero invalid");
    expect(build(frame(1000, 69)).observation.return_kind ==
               ScalarTofReturnKind::Invalid,
           "confidence 69 invalid");
    expect(build(frame(1000, 70)).observation.return_kind ==
               ScalarTofReturnKind::Hit,
           "confidence 70 hit");
    expect(build(frame(1000, 100)).observation.return_kind ==
               ScalarTofReturnKind::Hit,
           "confidence 100 hit");
    expect(build(frame(1000, 101)).observation.return_kind ==
               ScalarTofReturnKind::Invalid,
           "confidence 101 invalid");
    expect(build(frame(19, 100)).observation.return_kind ==
               ScalarTofReturnKind::Invalid,
           "distance below protocol invalid");
    expect(build(frame(12001, 100)).observation.return_kind ==
               ScalarTofReturnKind::Invalid,
           "distance above protocol invalid");
    expect(build(frame(40, 100)).observation.return_kind ==
               ScalarTofReturnKind::Invalid,
           "distance below mapping invalid");
    expect(build(frame(12000, 100)).observation.return_kind ==
               ScalarTofReturnKind::Hit,
           "distance 12000 not automatic no return");
    expect(build(frame(12000, 100), ScalarTofReturnKind::NoReturn)
                   .observation.return_kind == ScalarTofReturnKind::NoReturn,
           "explicit no return with confidence maps to no return");
    expect(build(frame(12000, 0), ScalarTofReturnKind::NoReturn)
                   .observation.return_kind == ScalarTofReturnKind::Invalid,
           "explicit no return still requires confidence");
    expect(build(frame(1000, 100), ScalarTofReturnKind::Unspecified,
                 100.0, false)
                   .observation.return_kind == ScalarTofReturnKind::Invalid,
           "sync failure invalid");
    expect(build(frame(1000, 100, 101.0), ScalarTofReturnKind::Unspecified,
                 100.0)
                   .observation.return_kind == ScalarTofReturnKind::Invalid,
           "future timestamp invalid");
    expect(build(frame(1000, 100, 99.0), ScalarTofReturnKind::Unspecified,
                 100.0)
                   .observation.return_kind == ScalarTofReturnKind::Invalid,
           "stale timestamp invalid");

    auto a = build(frame(1000, 100));
    auto b = build(frame(1000, 100));
    expect(a.observation.return_kind == b.observation.return_kind &&
               std::fabs(a.observation.measured_range_m -
                         b.observation.measured_range_m) < 1e-12,
           "same input deterministic");

    auto echo_a = frame(1000, 100);
    auto echo_b = echo_a;
    echo_b.echo_tag_u48 = 0xABCDEFULL;
    expect(build(echo_a).observation.return_kind ==
               build(echo_b).observation.return_kind,
           "echo tag does not affect return kind");
    return failures ? 1 : 0;
}
