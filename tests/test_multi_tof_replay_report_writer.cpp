#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_replay_report_writer.hpp"

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
    const auto report = MultiTofReplayAcceptanceRunner().run_once();
    const auto text = MultiTofReplayReportWriter().write_text_report(report);
    expect(text.find("Multi-ToF Replay Report") != std::string::npos,
           "contains title");
    expect(text.find("packet_count") != std::string::npos,
           "contains packet count");
    expect(text.find("invalid_record_count") != std::string::npos,
           "contains invalid count");
    expect(text.find("snapshot_has_multi_tof") != std::string::npos,
           "contains has_multi_tof");
    expect(text.find("valid_tof_count") != std::string::npos,
           "contains valid count");
    expect(text.find("No real ToF/IMU/Wheel hardware is read") != std::string::npos,
           "contains no hardware disclaimer");
    expect(text.find("deferred to M3-C4") != std::string::npos,
           "contains m3c4 deferred");
    return failures ? 1 : 0;
}
