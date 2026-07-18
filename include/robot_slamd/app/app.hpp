#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/app/robot_slam_application_adapters.hpp"
#include "robot_slamd/config/config.hpp"
#include "robot_slamd/runtime/application_mode.hpp"
#include "robot_slamd/simulation/exploration/m3e_exploration_runner.hpp"
#include "robot_slamd/simulation/application/simulation_robot_slam_runner.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "robot_slamd/replay/stop_go_replay_runner.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace robot_slamd {

inline int real_main(int argc, char **argv) {
    std::string config_path;
    std::string output_override;
    double duration_s = -1.0;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (argument == "--duration-s" && i + 1 < argc) {
            duration_s = std::strtod(argv[++i], nullptr);
        } else if (argument == "--output-dir" && i + 1 < argc) {
            output_override = argv[++i];
        } else {
            std::cerr
                << "usage: robot_slamd --config config.yaml "
                   "[--duration-s N] [--output-dir PATH]\n";
            return 2;
        }
    }
    if (config_path.empty()) {
        std::cerr << "--config is required\n";
        return 2;
    }

    Config config = load_config(config_path, output_override);
    config.output_dir = absolute_path(config.output_dir);
    validate_config(config);

    SensorSource source = SensorSource::Simulation;
    OperationMode operation = OperationMode::Mapping;
    if (!parse_sensor_source(config.runtime_sensor_source, source) ||
        !parse_operation_mode(config.runtime_operation, operation)) {
        std::cerr << "runtime source/operation parsing failed\n";
        return 2;
    }

    std::string error;
    const std::string run_root = config.output_dir + "/runs";
    if (!mkdir_p(run_root)) {
        std::cerr << "failed to create output_dir: " << run_root << "\n";
        return 1;
    }
    std::string run_dir;
    if (!create_unique_run_dir(run_root, run_dir, error)) {
        std::cerr << "failed to create run_dir under " << run_root << ": "
                  << error << "\n";
        return 1;
    }
    unlink((config.output_dir + "/latest").c_str());
    if (symlink(run_dir.c_str(),
                (config.output_dir + "/latest").c_str()) != 0) {
        std::ofstream(config.output_dir + "/latest.txt") << run_dir << "\n";
    }
    write_resolved_config(config, run_dir + "/config.resolved.yaml");
    std::ifstream input(config_path, std::ios::binary);
    std::ofstream(run_dir + "/config.input.yaml", std::ios::binary)
        << input.rdbuf();

    if (source == SensorSource::Real) {
        std::cerr
            << "real sensor adapter unavailable; startup rejected fail-closed\n";
        return 1;
    }
    if (source == SensorSource::Replay) {
        if (operation == OperationMode::StopGoWallMapping) {
            if (config.runtime_replay_path.empty()) {
                std::cerr << "stop-go replay requires runtime.replay_path\n";
                return 1;
            }
            const auto report = StopGoReplayRunner{}.run(
                config, config.runtime_replay_path, run_dir);
            std::cout << "stop_go_replay"
                      << " completed_steps=" << report.completed_steps
                      << " stable_samples=" << report.stable_samples
                      << " map_commits=" << report.map_commits
                      << " map_revision=" << report.map_revision
                      << " map_cells=" << report.map_cells
                      << " left_wall_observed_cells=" << report.left_wall_observed_cells
                      << " final_map_checksum=" << report.final_map_checksum
                      << " termination_reason=" << report.termination_reason
                      << " run_dir=" << run_dir << "\n";
            return report.ok ? 0 : 1;
        }
        if (operation == OperationMode::Exploration) {
            std::cerr << "replay exploration requires a motion adapter\n";
            return 1;
        }
        const auto adapter = build_replay_sensor_adapter(config);
        if (!adapter.ok) {
            std::cerr << adapter.reason << "\n";
            return 1;
        }
        RobotSlamApplication application(
            config, source, operation, adapter.sensor, nullptr);
        const auto initialized = application.initialize();
        if (!initialized.ok) {
            std::cerr << initialized.reason << "\n";
            return 1;
        }

        const double hz = config.tof_read_hz > 0.0 ? config.tof_read_hz : 1.0;
        const double dt = 1.0 / hz;
        const std::size_t replay_steps =
            duration_s > 0.0
                ? static_cast<std::size_t>(std::ceil(duration_s * hz)) + 1U
                : static_cast<std::size_t>(adapter.sensor->packet_count()) + 1U;
        double now_s = adapter.sensor->first_packet_time_s();
        for (std::size_t step = 0; step < replay_steps; ++step) {
            const auto result = application.step(now_s);
            if (adapter.sensor->replay_completed()) {
                break;
            }
            if (adapter.sensor->replay_faulted() || !result.core_step_executed) {
                std::cerr << result.reason << "\n";
                return 1;
            }
            now_s += dt;
        }
        std::cout << "robot_slamd application sensor_source="
                  << to_string(source) << " operation=" << to_string(operation)
                  << " replay_steps=" << application.report().core_step_count
                  << " run_dir=" << run_dir << "\n";
        return 0;
    }
    if (operation == OperationMode::StopGoWallMapping) {
        if (source != SensorSource::Simulation) {
            std::cerr << "stop-go mapping currently supports simulation only; real/replay are fail-closed\n";
            return 1;
        }
        const auto report = StopGoStraightWallSimulationRunner{}.run(config, run_dir);
        std::cout << "stop_go_formal"
                  << " completed_steps=" << report.completed_steps
                  << " commands_completed=" << report.commands_completed
                  << " stable_samples=" << report.stable_samples
                  << " map_commits=" << report.map_commits
                  << " map_revision=" << report.map_revision
                  << " map_cells=" << report.map_cells
                  << " left_wall_observed_cells=" << report.left_wall_observed_cells
                  << " collision_count=" << report.collision_count
                  << " map_writes_while_moving=" << report.map_writes_while_moving
                  << " command_speed_used_as_odometry="
                  << (report.command_speed_used_as_odometry ? "true" : "false")
                  << " ground_truth_used_by_algorithm="
                  << (report.ground_truth_used_by_algorithm ? "true" : "false")
                  << " final_map_checksum=" << report.final_map_checksum
                  << " termination_reason=" << report.termination_reason
                  << " run_dir=" << run_dir << "\n";
        return report.ok ? 0 : 1;
    }
    if (operation != OperationMode::Exploration) {
        const auto report =
            SimulationRobotSlamRunner{}.run(config, run_dir, duration_s);
        if (!report.ok) {
            std::cerr << report.reason << "\n";
            return 1;
        }
        std::cout << "robot_slamd application sensor_source="
                  << to_string(source) << " operation=" << to_string(operation)
                  << " simulation_steps=" << report.simulation_steps
                  << " map_revision=" << report.map_revision_after
                  << " map_cells=" << report.map_cell_count
                  << " run_dir=" << run_dir << "\n";
        return 0;
    }

    const auto report =
        M3EExplorationRunner{}.run(config, run_dir, duration_s);
    if (!report.ok) {
        std::cerr << report.termination_reason << "\n";
        return 1;
    }
    std::cout << "robot_slamd application sensor_source="
              << to_string(source) << " operation=" << to_string(operation)
              << " simulation_steps=" << report.simulation_steps
              << " completed_goals=" << report.exploration.completed_goals
              << " map_revision=" << report.exploration.map_revision_end
              << " map_cells=" << report.exploration.known_cells_end
              << " collision=" << (report.collision ? 1 : 0)
              << " run_dir=" << run_dir << "\n";
    return 0;
}

} // namespace robot_slamd
