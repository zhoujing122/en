#include "robot_slamd/autonomy/fake_map/fake_map_report_writer.hpp"
#include <iostream>
namespace { int failures = 0; void expect(bool c, const char *m) { if (!c) { std::cerr << "FAIL: " << m << "\n"; failures++; } } }
int main() {
    using namespace robot_slamd;
    FakeMapStorageResult result;
    result.ok = true;
    result.artifact.status = FakeMapArtifactStatus::Saved;
    result.artifact.metadata.map_id = "fake_map_a";
    result.artifact.metadata.coverage_ratio = 1.0;
    result.artifact.serialized_text = "map_id=fake_map_a\ncoverage=1\n";
    const auto text = FakeMapReportWriter{}.write_text_report(result);
    expect(text.find("Fake Map Artifact Report") != std::string::npos, "title");
    expect(text.find("fake_map_a") != std::string::npos, "map id");
    expect(text.find("coverage") != std::string::npos, "coverage");
    expect(text.find("metadata only") != std::string::npos, "metadata disclaimer");
    expect(text.find("No real map file is written.") != std::string::npos, "file disclaimer");
    return failures ? 1 : 0;
}
