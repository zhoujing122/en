#include "robot_slamd/autonomy/real_adapters/multi_tof/multi_tof_sync_report_writer.hpp"

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
    MultiTofSyncOptions options;
    options.enabled = true;
    const auto text = MultiTofSyncReportWriter().write_text_report(MultiTofSyncAcceptanceRunner().run_once(), options);
    expect(text.find("Multi-ToF Sync Report") != std::string::npos, "title");
    expect(text.find("front-left dt") != std::string::npos, "front-left dt");
    expect(text.find("front-right dt") != std::string::npos, "front-right dt");
    expect(text.find("left-right dt") != std::string::npos, "left-right dt");
    expect(text.find("multi-tof imu dt") != std::string::npos, "imu dt");
    expect(text.find("multi-tof wheel dt") != std::string::npos, "wheel dt");
    expect(text.find("time reference") != std::string::npos, "time reference");
    expect(text.find("degradation mode") != std::string::npos, "degradation mode");
    expect(text.find("This stage does not read real hardware") != std::string::npos, "no hardware");
    return failures ? 1 : 0;
}
