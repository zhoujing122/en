#include "robot_slamd/autonomy/product_acceptance/fake_to_real_adapter_replacement_manifest.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
static bool contains(const std::string &text, const std::string &needle) { return text.find(needle) != std::string::npos; }
int main() {
    using namespace robot_slamd;
    FakeToRealAdapterReplacementManifestBuilder builder;
    const auto manifest = builder.build_default_manifest();
    const auto text = builder.write_text(manifest);
    expect(manifest.valid, "manifest valid");
    expect(contains(text, "RealSensorReplayPort"), "sensor replacement");
    expect(contains(text, "FullAutonomousSlamFakeMotionPort"), "motion replacement");
    expect(contains(text, "DeterministicSlamBackendBinding"), "slam replacement");
    expect(contains(text, "FakeMapStorage"), "map replacement");
    expect(contains(text, "FakeRelocalizationBinding"), "relocalization replacement");
    expect(contains(text, "Fake pose candidate"), "pose candidate replacement");
    return failures ? 1 : 0;
}
