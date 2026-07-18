#include "robot_slamd/replay/stop_go_replay_runner.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"

#include <filesystem>

static robot_slamd::Config config() {
    robot_slamd::Config c;
    c.runtime_sensor_source = "simulation";
    c.runtime_operation = "stop_go_wall_mapping";
    c.stop_go_mapping_enabled = true;
    c.sparse_slam_planar_tof_extrinsics_configured = true;
    c.sparse_slam_allow_legacy_mount_yaw_extrinsics = false;
    c.sparse_slam_front_tof_x_m = 0.10;
    c.sparse_slam_left_tof_y_m = 0.08;
    c.sparse_slam_left_tof_yaw_rad = 1.5707963267948966;
    c.sparse_slam_right_tof_y_m = -0.08;
    c.sparse_slam_right_tof_yaw_rad = -1.5707963267948966;
    c.exploration_simulation_initial_x_m = -1.2;
    c.exploration_simulation_initial_y_m = 0.0;
    c.exploration_simulation_initial_yaw_rad = 0.0;
    c.sparse_slam_map_run_id = "replay_stop_go_test";
    return c;
}

int main() {
    using namespace robot_slamd;
    const std::string source_dir = "/tmp/p2_stop_go_replay_source";
    const std::string replay_dir = "/tmp/p2_stop_go_replay_output";
    std::filesystem::create_directories(source_dir);
    std::filesystem::create_directories(replay_dir);
    std::filesystem::create_directories(replay_dir + "/one");
    std::filesystem::create_directories(replay_dir + "/two");
    const auto simulation = StopGoStraightWallSimulationRunner{}.run(config(), source_dir);
    if (!simulation.ok || simulation.replay_records.empty()) return 1;
    const std::string replay_path = source_dir + "/stop_go_mapping.replay";
    const auto first = StopGoReplayRunner{}.run(config(), replay_path, replay_dir + "/one");
    const auto second = StopGoReplayRunner{}.run(config(), replay_path, replay_dir + "/two");
    return first.ok && second.ok && first.completed_steps == 20 &&
                   first.map_commits == 21 && first.final_map_checksum != 0 &&
                   first.final_map_checksum == second.final_map_checksum
               ? 0 : 1;
}
