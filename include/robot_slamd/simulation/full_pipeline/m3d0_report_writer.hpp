#pragma once

#include "robot_slamd/simulation/full_pipeline/m3d0_scenario_types.hpp"

#include <sstream>

namespace robot_slamd {

class M3D0ReportWriter {
public:
    std::string write_text(const M3D0SimulationReport &report) const {
        std::ostringstream out;
        out << "M3-D0 deterministic closed-loop 2D simulation\n";
        out << "ok=" << (report.ok ? "true" : "false") << "\n";
        out << "duration_s=" << report.simulated_duration_s << "\n";
        out << "physics_steps=" << report.physics_step_count << "\n";
        out << "closed_loop_verified=" << (report.closed_loop_verified ? "true" : "false") << "\n";
        out << "ground truth is used only by simulation sensors and acceptance tests\n";
        out << "Coordinator and deterministic backend do not read ground truth\n";
        out << "native multi-ToF SLAM backend not implemented\n";
        out << "occupancy mapping not implemented\n";
        out << "scan matching not implemented\n";
        out << "Frontier/A* not implemented\n";
        out << "real hardware not accessed\n";
        out << "real motion not attempted\n";
        return out.str();
    }
};

} // namespace robot_slamd
