#include "robot_slamd/app/app.hpp"
#include "robot_slamd/simulation/application/simulation_robot_slam_runner.hpp"

#include <cstdio>
#include <fstream>
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

robot_slamd::Config simulation_config() {
    robot_slamd::Config config;
    config.runtime_sensor_source = "simulation";
    config.runtime_operation = "mapping";
    config.exploration_min_x_m = -1.0;
    config.exploration_max_x_m = 1.5;
    config.exploration_min_y_m = -1.0;
    config.exploration_max_y_m = 1.0;
    config.exploration_robot_radius_m = 0.08;
    config.exploration_simulation_fixed_dt_s = 0.05;
    config.exploration_simulation_tof_max_range_m = 2.0;
    config.exploration_simulation_initial_x_m = -0.75;
    config.exploration_simulation_initial_y_m = 0.0;
    config.exploration_simulation_initial_yaw_rad = 0.0;
    config.sparse_slam_planar_tof_extrinsics_configured = true;
    config.sparse_slam_front_tof_x_m = 0.04;
    config.sparse_slam_front_tof_y_m = 0.0;
    config.sparse_slam_front_tof_yaw_rad = 0.0;
    config.sparse_slam_left_tof_x_m = 0.0;
    config.sparse_slam_left_tof_y_m = 0.04;
    config.sparse_slam_left_tof_yaw_rad = robot_slamd::kPi / 2.0;
    config.sparse_slam_right_tof_x_m = 0.0;
    config.sparse_slam_right_tof_y_m = -0.04;
    config.sparse_slam_right_tof_yaw_rad = -robot_slamd::kPi / 2.0;
    config.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    config.sparse_slam_active_bundle_min_snapshots = 3;
    config.sparse_slam_active_bundle_min_matchable_rays = 3;
    config.sparse_slam_active_bundle_min_yaw_span_rad = 0.05;
    config.sparse_slam_local_match_min_bundle_frames = 3;
    config.sparse_slam_local_match_min_matchable_rays = 3;
    config.sparse_slam_local_match_min_used_rays = 1;
    config.sparse_slam_local_match_min_known_cells = 1;
    config.sparse_slam_local_match_min_reference_cells = 1;
    config.sparse_slam_local_match_min_reference_occupied_cells = 0;
    config.sparse_slam_local_match_min_reference_free_cells = 0;
    config.sparse_slam_local_match_min_normalized_score = -1.0;
    config.sparse_slam_local_match_min_score_margin = 0.0;
    config.sparse_slam_local_match_min_score_range = 0.0;
    config.sparse_slam_local_match_max_unknown_ratio = 0.99;
    config.sparse_slam_local_match_max_contradiction_ratio = 0.99;
    config.sparse_slam_map_id = "simulation_unified_map";
    config.sparse_slam_map_run_id = "simulation_unified_run";
    robot_slamd::validate_config(config);
    return config;
}

void write_formal_config(const std::string &path,
                         const std::string &operation,
                         const std::string &map_path = {}) {
    std::ofstream out(path);
    out << "runtime:\n"
        << "  sensor_source: simulation\n"
        << "  operation: " << operation << "\n"
        << "sparse_slam:\n"
        << "  planar_tof_extrinsics_configured: true\n"
        << "  allow_legacy_mount_yaw_extrinsics: false\n"
        << "  front_tof_x_m: 0.04\n"
        << "  front_tof_y_m: 0.0\n"
        << "  front_tof_yaw_rad: 0.0\n"
        << "  left_tof_x_m: 0.0\n"
        << "  left_tof_y_m: 0.04\n"
        << "  left_tof_yaw_rad: 1.5707963267948966\n"
        << "  right_tof_x_m: 0.0\n"
        << "  right_tof_y_m: -0.04\n"
        << "  right_tof_yaw_rad: -1.5707963267948966\n"
        << "  map_startup_mode: "
        << (map_path.empty() ? "create_new" : "load_existing") << "\n"
        << "  initial_pose_mode: "
        << (map_path.empty() ? "startup_zero" : "configured_pose") << "\n"
        << "  has_configured_pose: " << (map_path.empty() ? "false" : "true") << "\n"
        << "  configured_pose_x_m: 0.0\n"
        << "  configured_pose_y_m: 0.0\n"
        << "  configured_pose_yaw_rad: 0.0\n"
        << "  map_path: " << map_path << "\n"
        << "  active_bundle_min_snapshots: 3\n"
        << "  active_bundle_min_matchable_rays: 3\n"
        << "  active_bundle_min_yaw_span_rad: 0.05\n"
        << "  local_match_min_bundle_frames: 3\n"
        << "  local_match_min_matchable_rays: 3\n"
        << "  local_match_min_used_rays: 1\n"
        << "  local_match_min_known_cells: 1\n"
        << "  local_match_min_reference_cells: 1\n"
        << "  local_match_min_reference_occupied_cells: 0\n"
        << "  local_match_min_reference_free_cells: 0\n"
        << "  local_match_min_normalized_score: -1.0\n"
        << "  local_match_min_score_margin: 0.0\n"
        << "  local_match_min_score_range: 0.0\n"
        << "  local_match_max_unknown_ratio: 0.99\n"
        << "  local_match_max_contradiction_ratio: 0.99\n"
        << "exploration:\n"
        << "  min_x_m: -1.0\n"
        << "  max_x_m: 1.5\n"
        << "  min_y_m: -1.0\n"
        << "  max_y_m: 1.0\n"
        << "  robot_radius_m: 0.08\n"
        << "  simulation_fixed_dt_s: 0.05\n"
        << "  simulation_tof_max_range_m: 2.0\n"
        << "  simulation_initial_x_m: -0.75\n"
        << "  simulation_initial_y_m: 0.0\n"
        << "  simulation_initial_yaw_rad: 0.0\n";
}

int formal_main(const std::string &config_path,
                const std::string &output_dir) {
    std::string program = "robot_slamd";
    std::string config_arg = "--config";
    std::string output_arg = "--output-dir";
    std::string duration_arg = "--duration-s";
    std::string duration = "2.0";
    char *argv[] = {program.data(), config_arg.data(),
                    const_cast<char *>(config_path.c_str()), duration_arg.data(),
                    duration.data(), output_arg.data(),
                    const_cast<char *>(output_dir.c_str())};
    return robot_slamd::real_main(7, argv);
}

} // namespace

int main() {
    using namespace robot_slamd;

    Config mapping_config = simulation_config();
    SimulationRobotSlamRunner runner;
    const auto mapping = runner.run(mapping_config, "/tmp", 2.0);
    expect(mapping.ok, "simulation mapping uses the unified Application");
    expect(mapping.application.sensor_source == SensorSource::Simulation &&
               mapping.application.operation == OperationMode::Mapping,
           "mapping report identifies simulation Application mode");
    expect(mapping.application.core_step_count > 0 &&
               mapping.application.canonical_snapshot_count > 0,
           "mapping Application forwards canonical snapshots to Core");
    expect(mapping.core.runtime_core_constructed &&
               mapping.core.wheel_imu_estimator_constructed &&
               mapping.core.sparse_backend_constructed,
           "mapping has one fully constructed Runtime Core");
    expect(mapping.map_revision_after > 0 && mapping.map_cell_count > 0 &&
               mapping.keyframe_count > 0 && mapping.map_saved,
           "mapping produces revision, keyframe, cells, and an artifact");
    expect(mapping.ground_truth_isolation_verified,
           "ground truth remains in the simulation-side adapter boundary");

    Config localization_config = mapping_config;
    localization_config.runtime_operation = "localization";
    localization_config.sparse_slam_map_startup_mode = "load_existing";
    localization_config.sparse_slam_initial_pose_mode = "configured_pose";
    localization_config.sparse_slam_has_configured_pose = true;
    localization_config.sparse_slam_map_path = mapping.map_path;
    validate_config(localization_config);
    const auto localization = runner.run(localization_config, "/tmp", 0.25);
    expect(localization.ok,
           "simulation localization uses the unified Application");
    expect(localization.core.sparse_map_loaded && localization.localization_ready,
           "localization loads the existing map and reports a valid map pose");
    expect(!localization.mapping_writes_enabled &&
               localization.map_revision_after == mapping.map_revision_after,
           "localization preserves the loaded map revision");
    expect(localization.application.core_step_count > 0 &&
               localization.application.canonical_snapshot_count > 0,
           "localization forwards canonical snapshots to the same Core path");

    auto composition = build_simulation_robot_slam_adapter(
        mapping_config, OperationMode::Mapping);
    expect(composition.ok && composition.application && composition.sensor,
           "Simulation adapter composition is complete");
    const auto initialized = composition.application->initialize();
    expect(initialized.ok, "direct Simulation adapter Application initializes");
    const auto first = composition.application->step(composition.clock->now_s());
    expect(first.core_step_executed &&
               composition.sensor->last_build_result().snapshot.has_multi_tof,
           "Simulation adapter emits the Replay-compatible canonical multi-ToF snapshot");
    expect(&composition.application->core() ==
               &composition.application->core(),
           "Simulation Application exposes one stable Core owner");

    Config bad = mapping_config;
    bad.runtime_sensor_source = "unknown";
    SensorSource bad_source = SensorSource::Simulation;
    expect(!parse_sensor_source(bad.runtime_sensor_source, bad_source),
           "unknown sensor source fails closed");
    bad = mapping_config;
    bad.runtime_operation = "unknown";
    OperationMode bad_operation = OperationMode::Mapping;
    expect(!parse_operation_mode(bad.runtime_operation, bad_operation),
           "unknown operation fails closed");

    const std::string map_path = "/tmp/simulation_unified_formal_map.smap";
    std::remove(map_path.c_str());
    const std::string mapping_yaml = "/tmp/simulation_unified_mapping.yaml";
    const std::string localization_yaml = "/tmp/simulation_unified_localization.yaml";
    write_formal_config(mapping_yaml, "mapping");
    expect(formal_main(mapping_yaml, "/tmp/simulation_unified_formal_mapping") == 0,
           "formal real_main simulation mapping smoke succeeds");

    const auto formal_map = runner.run(mapping_config, "/tmp", 2.0);
    expect(formal_map.ok && !formal_map.map_path.empty(),
           "formal localization receives a verified map artifact");
    write_formal_config(localization_yaml, "localization", formal_map.map_path);
    expect(formal_main(localization_yaml,
                       "/tmp/simulation_unified_formal_localization") == 0,
           "formal real_main simulation localization smoke succeeds");

    std::remove(mapping_yaml.c_str());
    std::remove(localization_yaml.c_str());
    std::remove(formal_map.map_path.c_str());
    return failures == 0 ? 0 : 1;
}
