#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_snapshot_report_writer.hpp"

#include <iostream>
#include <string>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}
}

int main() {
    using namespace robot_slamd;
    const auto report = MultiTofSnapshotAcceptanceRunner().run_once();
    const auto text = MultiTofSnapshotReportWriter().write_text_report(report);
    expect(text.find("Multi-ToF Snapshot Report") != std::string::npos,
           "contains title");
    expect(text.find("has_multi_tof") != std::string::npos,
           "contains has_multi_tof");
    expect(text.find("synchronized_time_s") != std::string::npos,
           "contains synchronized time");
    expect(text.find("valid_tof_count") != std::string::npos,
           "contains valid count");
    expect(text.find("front_present") != std::string::npos,
           "contains front");
    expect(text.find("left_present") != std::string::npos,
           "contains left");
    expect(text.find("right_present") != std::string::npos,
           "contains right");
    expect(text.find("legacy_primary") != std::string::npos,
           "contains legacy primary");
    expect(text.find("does not read real hardware") != std::string::npos,
           "contains no hardware disclaimer");
    return failures ? 1 : 0;
}
