#include "robot_slamd/autonomy/fake_map/fake_map_artifact_types.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    expect(to_string(FakeMapArtifactStatus::Built) == "built", "status string");
    expect(to_string(FakeMapArtifactFault::QualityNotGood) == "quality_not_good", "fault string");
    expect(fake_map_artifact_status_id(FakeMapArtifactStatus::Saved) == 2, "status id");
    expect(fake_map_artifact_fault_id(FakeMapArtifactFault::SaveDisabled) == 4, "fault id");
    FakeMapArtifact artifact;
    expect(artifact.status == FakeMapArtifactStatus::Empty, "default status");
    FakeMapStorageResult result;
    expect(!result.ok, "default result not ok");
    return failures ? 1 : 0;
}
