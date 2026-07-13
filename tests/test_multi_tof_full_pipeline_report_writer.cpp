#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_report_writer.hpp"
#include "robot_slamd/autonomy/full_pipeline/multi_tof/multi_tof_full_pipeline_runner.hpp"

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
    const auto report = MultiTofFullPipelineRunner().run_once(
        MultiTofFullPipelineScenarioBuilder().build(
            MultiTofFullPipelineScenarioKind::NormalRequireAll));
    const auto text = MultiTofFullPipelineReportWriter().write_text(report);
    expect(text.find("scalar three-ToF replay") != std::string::npos,
           "mentions scalar three-ToF replay");
    expect(text.find("RobotSlamSensorPort") != std::string::npos,
           "mentions RobotSlamSensorPort");
    expect(text.find("Coordinator") != std::string::npos,
           "mentions Coordinator");
    expect(text.find("legacy scalar projection") != std::string::npos,
           "mentions legacy scalar projection");
    expect(text.find("deterministic backend") != std::string::npos,
           "mentions deterministic backend");
    expect(text.find("native_fusion=not implemented") != std::string::npos,
           "native fusion not implemented");
    expect(text.find("motion=fake motion only") != std::string::npos,
           "fake motion only");
    expect(text.find("real_hardware=not accessed") != std::string::npos,
           "real hardware not accessed");
    expect(text.find("pose_writeback=not attempted") != std::string::npos,
           "pose writeback not attempted");
    expect(text.find("native_multi_tof_backend_consumption=0") != std::string::npos,
           "native backend false");
    expect(text.find("real_hardware_accessed=0") != std::string::npos,
           "hardware flag false");
    expect(text.find("real_pose_writeback_attempted=0") != std::string::npos,
           "pose flag false");
    expect(text.find("warning=multi_tof_pipeline_uses_legacy_scalar_projection_for_deterministic_backend") != std::string::npos,
           "legacy warning");
    expect(text.find("real mapping completed") == std::string::npos,
           "does not claim real mapping");
    expect(text.find("native multi-ToF fusion completed") == std::string::npos,
           "does not claim native fusion");
    expect(text.find("autonomous exploration completed") == std::string::npos,
           "does not claim exploration");
    return failures ? 1 : 0;
}
