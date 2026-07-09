#include "robot_slamd/autonomy/product_acceptance/fake_autonomous_slam_product_acceptance_runner.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
static robot_slamd::FakeAutonomousSlamProductAcceptanceReport run(robot_slamd::FakeAutonomousSlamProductAcceptanceOptions options) {
    robot_slamd::FakeAutonomousSlamProductAcceptanceRunner runner(options);
    return runner.run_once();
}
int main() {
    using namespace robot_slamd;
    FakeAutonomousSlamProductAcceptanceOptions options;
    expect(!run(options).ok, "disabled rejected");
    options.enabled = true; options.force_mapping_failure = true;
    expect(run(options).block_reason == FakeAutonomousSlamProductAcceptanceBlockReason::MappingPipelineFailed, "mapping fail");
    options = {}; options.enabled = true; options.force_fake_map_missing = true;
    expect(run(options).block_reason == FakeAutonomousSlamProductAcceptanceBlockReason::FakeMapMissing, "map missing");
    options = {}; options.enabled = true; options.force_relocalization_failure = true;
    expect(run(options).block_reason == FakeAutonomousSlamProductAcceptanceBlockReason::RelocalizationFailed, "relocalization fail");
    options = {}; options.enabled = true; options.force_readiness_blocked = true;
    expect(run(options).block_reason == FakeAutonomousSlamProductAcceptanceBlockReason::RelocalizationReadinessBlocked, "readiness blocked");
    options = {}; options.enabled = true; options.force_manifest_invalid = true;
    expect(run(options).block_reason == FakeAutonomousSlamProductAcceptanceBlockReason::AdapterManifestInvalid, "manifest invalid");
    return failures ? 1 : 0;
}
