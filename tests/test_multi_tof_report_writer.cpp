#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_report_writer.hpp"

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
    const auto report = MultiTofAcceptanceRunner().run_once();
    const auto text = MultiTofReportWriter().write_text_report(report);
    expect(text.find("Multi-ToF Raw Data Contract Report") != std::string::npos, "title");
    expect(text.find("valid_tof_count") != std::string::npos, "valid count");
    expect(text.find("front") != std::string::npos, "front");
    expect(text.find("left") != std::string::npos, "left");
    expect(text.find("right") != std::string::npos, "right");
    expect(text.find("front yaw=0, left yaw=+90, right yaw=-90") != std::string::npos, "yaw disclaimer");
    expect(text.find("3-ToF pairwise sync is deferred to M3-C1") != std::string::npos, "sync deferred");
    expect(text.find("No real ToF/IMU/Wheel device is read") != std::string::npos, "no hardware");
    return failures ? 1 : 0;
}
