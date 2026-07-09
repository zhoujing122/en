#include "robot_slamd/autonomy/fake_relocalization/fake_relocalization_types.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    expect(to_string(FakeRelocalizationStatus::Accepted) == "accepted", "status string");
    expect(to_string(FakeRelocalizationFault::MissingTof) == "missing_tof", "fault string");
    expect(to_string(FakeRelocalizationConfidenceBand::High) == "high", "band string");
    expect(fake_relocalization_status_id(FakeRelocalizationStatus::Rejected) == 4, "status id");
    expect(fake_relocalization_fault_id(FakeRelocalizationFault::PoseWritebackForbidden) == 9, "fault id");
    expect(fake_relocalization_confidence_band_id(FakeRelocalizationConfidenceBand::Medium) == 2, "band id");
    FakeRelocalizationOptions options;
    expect(!options.enabled, "default disabled");
    expect(!options.allow_pose_writeback, "writeback false");
    FakeRelocalizationResult result;
    expect(!result.ok, "default result not ok");
    return failures ? 1 : 0;
}
