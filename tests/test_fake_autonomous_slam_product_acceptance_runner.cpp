#include "robot_slamd/autonomy/product_acceptance/fake_autonomous_slam_product_acceptance_runner.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FakeAutonomousSlamProductAcceptanceOptions options; options.enabled = true;
    FakeAutonomousSlamProductAcceptanceRunner runner(options);
    const auto report = runner.run_once();
    expect(report.ok, "acceptance ok");
    expect(report.mapping_report.ok, "mapping ok");
    expect(report.mapping_report.fake_map_built, "map built");
    expect(report.mapping_report.fake_map_saved, "map saved");
    expect(report.relocalization_report.ok, "relocalization ok");
    expect(report.readiness_result.ok, "readiness ok");
    expect(report.adapter_manifest_text.find("valid: true") != std::string::npos, "manifest valid");
    expect(!report.relocalization_report.pose_writeback_attempted, "no pose writeback");
    expect(report.mapping_report.failed.empty(), "no forward backward failure");
    expect(report.summary == "fake_autonomous_slam_product_acceptance_passed", "summary");
    return failures ? 1 : 0;
}
