#pragma once

#include "robot_slamd/config/config.hpp"
#include "robot_slamd/runtime/application_mode.hpp"
#include "robot_slamd/simulation/exploration/m3e_exploration_runner.hpp"

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
        std::cerr
            << "replay Application runner requires a canonical replay adapter\n";
        return 1;
    }
    if (operation != OperationMode::Exploration) {
        std::cerr
            << "simulation mapping/localization runner is not yet composed\n";
        return 1;
    }

    const auto report =
        M3EExplorationRunner{}.run(config, run_dir, duration_s);
    if (!report.ok) {
        std::cerr << report.termination_reason << "\n";
        return 1;
    }
    std::cout << "robot_slamd application sensor_source="
              << to_string(source) << " operation=" << to_string(operation)
              << " run_dir=" << run_dir << "\n";
    return 0;
}

} // namespace robot_slamd
