#pragma once

#include "robot_slamd/simulation/full_pipeline/m3d0_scenario_types.hpp"

namespace robot_slamd {

class M3D0ScenarioBuilder {
public:
    static M3D0Scenario closed_loop_active_scan() {
        M3D0Scenario scenario;
        scenario.kind = M3D0ScenarioKind::ClosedLoopActiveScan;
        scenario.name = "closed_loop_active_scan";
        scenario.run_coordinator = true;
        scenario.fixed_dt_s = 0.02;
        scenario.max_duration_s = 4.0;
        scenario.map_scans_to_good = 2;
        scenario.initial_pose.x_m = -0.45;
        scenario.initial_pose.y_m = -0.20;
        scenario.initial_pose.yaw_rad = 0.20;
        scenario.motion_config.allow_translation = false;
        scenario.plant_config.collision_check_enabled = false;
        scenario.sensor_config.sync_options.enabled = true;
        scenario.sensor_config.build_options.enabled = true;
        return scenario;
    }

    static M3D0Scenario no_command_static() {
        M3D0Scenario scenario = closed_loop_active_scan();
        scenario.kind = M3D0ScenarioKind::NoCommandStatic;
        scenario.name = "no_command_static";
        scenario.run_coordinator = false;
        scenario.max_duration_s = 5.0;
        return scenario;
    }

    static M3D0Scenario forward_translation() {
        M3D0Scenario scenario = no_command_static();
        scenario.kind = M3D0ScenarioKind::ForwardTranslationSimulationOnly;
        scenario.name = "forward_translation_simulation_only";
        scenario.motion_config.allow_translation = true;
        return scenario;
    }
};

} // namespace robot_slamd
