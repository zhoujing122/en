#pragma once

#include "robot_slamd/autonomy/autonomous_slam_types.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"
#include "robot_slamd/simulation/ports/sim_sensor_port.hpp"

#include <string>
#include <vector>

namespace robot_slamd {

enum class M3D0ScenarioKind {
    NoCommandStatic,
    TurnLeftChangesSensors,
    TurnRightDirection,
    ForwardTranslationSimulationOnly,
    BackwardTranslationSimulationOnly,
    StopInterruptsMotion,
    EmergencyStop,
    CollisionPrevention,
    SensorTiming,
    DeterministicRepeatability,
    ClosedLoopActiveScan
};

struct M3D0Scenario {
    M3D0ScenarioKind kind = M3D0ScenarioKind::ClosedLoopActiveScan;
    double start_time_s = 100.0;
    double fixed_dt_s = 0.02;
    double max_duration_s = 5.0;
    RobotPose2D initial_pose;
    SimRobotPlantConfig plant_config;
    SimMotionPortConfig motion_config;
    SimSensorPortConfig sensor_config;
    int map_scans_to_good = 2;
    bool run_coordinator = false;
    std::string name = "m3d0_closed_loop";
};

struct M3D0SimulationReport {
    bool ok = false;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    double simulated_duration_s = 0.0;
    double fixed_dt_s = 0.0;
    int physics_step_count = 0;
    int coordinator_step_count = 0;
    int motion_command_count = 0;
    int active_scan_command_count = 0;
    int turn_left_command_count = 0;
    int turn_right_command_count = 0;
    int forward_command_count = 0;
    int backward_command_count = 0;
    int stop_command_count = 0;
    int emergency_stop_count = 0;
    int sensor_publish_count = 0;
    int wheel_sample_count = 0;
    int imu_sample_count = 0;
    int front_tof_sample_count = 0;
    int left_tof_sample_count = 0;
    int right_tof_sample_count = 0;
    RobotPose2D initial_ground_truth_pose;
    RobotPose2D final_ground_truth_pose;
    double traveled_distance_m = 0.0;
    double accumulated_abs_yaw_rad = 0.0;
    bool pose_changed = false;
    bool wheel_changed = false;
    bool imu_changed = false;
    bool tof_changed = false;
    bool non_instant_motion_verified = false;
    bool motion_settle_verified = false;
    bool sensor_timestamp_verified = false;
    bool collision_prevention_verified = false;
    bool deterministic_repeatability_verified = false;
    bool closed_loop_verified = false;
    bool ground_truth_isolation_verified = false;
    bool simulation_translation_enabled = false;
    bool forward_seen = false;
    bool backward_seen = false;
    bool real_hardware_accessed = false;
    bool real_motion_attempted = false;
    bool real_map_write_attempted = false;
    bool real_pose_writeback_attempted = false;
    std::vector<std::string> passed;
    std::vector<std::string> failed;
    std::vector<std::string> warnings;
    std::string summary;
};

inline std::string to_string(M3D0ScenarioKind kind) {
    switch (kind) {
    case M3D0ScenarioKind::NoCommandStatic:
        return "no_command_static";
    case M3D0ScenarioKind::TurnLeftChangesSensors:
        return "turn_left_changes_sensors";
    case M3D0ScenarioKind::TurnRightDirection:
        return "turn_right_direction";
    case M3D0ScenarioKind::ForwardTranslationSimulationOnly:
        return "forward_translation_simulation_only";
    case M3D0ScenarioKind::BackwardTranslationSimulationOnly:
        return "backward_translation_simulation_only";
    case M3D0ScenarioKind::StopInterruptsMotion:
        return "stop_interrupts_motion";
    case M3D0ScenarioKind::EmergencyStop:
        return "emergency_stop";
    case M3D0ScenarioKind::CollisionPrevention:
        return "collision_prevention";
    case M3D0ScenarioKind::SensorTiming:
        return "sensor_timing";
    case M3D0ScenarioKind::DeterministicRepeatability:
        return "deterministic_repeatability";
    case M3D0ScenarioKind::ClosedLoopActiveScan:
        return "closed_loop_active_scan";
    }
    return "unknown";
}

} // namespace robot_slamd
