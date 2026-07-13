#include "robot_slamd/simulation/full_pipeline/m3d0_report_writer.hpp"

#include <iostream>

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
    M3D0SimulationReport report;
    report.ok = true;
    report.closed_loop_verified = true;
    report.simulated_duration_s = 1.0;
    report.physics_step_count = 50;
    const auto text = M3D0ReportWriter{}.write_text(report);
    expect(text.find("deterministic closed-loop 2D simulation") != std::string::npos,
           "title present");
    expect(text.find("ground truth is used only by simulation sensors and acceptance tests") != std::string::npos,
           "ground truth boundary");
    expect(text.find("Coordinator and deterministic backend do not read ground truth") != std::string::npos,
           "backend boundary");
    expect(text.find("native multi-ToF SLAM backend not implemented") != std::string::npos,
           "native backend limitation");
    expect(text.find("real hardware not accessed") != std::string::npos,
           "real hardware boundary");
    expect(text.find("real motion not attempted") != std::string::npos,
           "real motion boundary");
    return failures ? 1 : 0;
}
