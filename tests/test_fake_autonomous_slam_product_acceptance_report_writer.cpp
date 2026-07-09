#include "robot_slamd/autonomy/product_acceptance/fake_autonomous_slam_product_acceptance_report_writer.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } bool has(const std::string &s, const std::string &n) { return s.find(n) != std::string::npos; } }
int main() {
    using namespace robot_slamd;
    FakeAutonomousSlamProductAcceptanceOptions options; options.enabled = true;
    FakeAutonomousSlamProductAcceptanceRunner runner(options);
    const auto report = runner.run_once();
    FakeAutonomousSlamProductAcceptanceReportWriter writer;
    const auto text = writer.write_text_report(report);
    expect(has(text, "Fake Autonomous SLAM Product Acceptance Report"), "title");
    expect(has(text, "mapping pipeline summary"), "mapping summary");
    expect(has(text, "relocalization confidence"), "confidence");
    expect(has(text, "relocalization readiness summary"), "readiness");
    expect(has(text, "adapter replacement manifest"), "manifest");
    expect(has(text, "does not prove real hardware readiness"), "hardware disclaimer");
    expect(has(text, "No pose writeback is performed"), "writeback disclaimer");
    return failures ? 1 : 0;
}
