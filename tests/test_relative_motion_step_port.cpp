#include "robot_slamd/autonomy/real_adapters/real_relative_motion_step_port.hpp"
#include "robot_slamd/replay/replay_relative_motion_step_port.hpp"
#include "robot_slamd/simulation/ports/sim_relative_motion_step_port.hpp"

#include <memory>
#include <string>

int main() {
    using namespace robot_slamd;
    if (static_cast<int>(RelativeMotionStepAction::Right) != 1 ||
        static_cast<int>(RelativeMotionStepAction::Left) != 2 ||
        std::string(to_string(RelativeMotionStepAction::Right)) != "RIGHT" ||
        std::string(to_string(RelativeMotionStepAction::Left)) != "LEFT") return 1;
    auto plant = std::make_shared<SimRobotPlant>();
    if (!plant->reset({}, 0.0)) return 1;
    SimRelativeMotionStepPort sim(plant);
    RelativeMotionStepCommand command{42, RelativeMotionStepAction::Forward, 20, 12, 3000};
    if (sim.submit(command, 0.0).state != RelativeMotionStepState::Accepted) return 1;
    RelativeMotionStepResult result;
    for (int i = 1; i <= 200; ++i) {
        const double now = i * 0.02;
        if (!sim.advance(0.02, now)) return 1;
        result = sim.poll(now);
        if (result.state == RelativeMotionStepState::Done) break;
    }
    if (result.state != RelativeMotionStepState::Done || result.command_id != 42 ||
        result.actual_distance_mm <= 0.0) return 1;
    ReplayNoMotionStepPort replay;
    if (replay.submit(command, 0.0).state != RelativeMotionStepState::Fault) return 1;
    RealRelativeMotionStepPort real;
    if (real.ready() || real.submit(command, 0.0).state != RelativeMotionStepState::Fault) return 1;
    if (sim.emergency_stop(4.0).state != RelativeMotionStepState::EstopLatched || sim.ready()) return 1;
    if (sim.clear_estop(4.1).state != RelativeMotionStepState::Done || !sim.ready()) return 1;
    return 0;
}
