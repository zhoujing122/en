#include "robot_slamd/autonomy/autonomous_slam_coordinator.hpp"
#include "robot_slamd/autonomy/odometry/wheel_imu_dead_reckoning_2d.hpp"
#include "robot_slamd/autonomy/real_adapters/slam_backend/sparse_multi_tof_occupancy_backend.hpp"
#include "robot_slamd/simulation/core/sim_clock.hpp"
#include "robot_slamd/simulation/ports/sim_motion_port.hpp"
#include "robot_slamd/simulation/ports/sim_sensor_port.hpp"
#include "robot_slamd/simulation/sensors/sim_imu.hpp"
#include "robot_slamd/simulation/sensors/sim_wheel_encoder.hpp"
#include "robot_slamd/simulation/world/fake_world_2d.hpp"

#include <cmath>
#include <iostream>
#include <memory>

namespace {
int failures = 0;
void expect(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        failures++;
    }
}

robot_slamd::WheelOdomFrame make_wheel_frame(
    const robot_slamd::SimWheelEncoderSample &sample) {
    robot_slamd::WheelOdomFrame frame;
    frame.timestamp_s = sample.raw.timing.estimated_sample_time_s;
    frame.linear_mps = sample.raw.linear_velocity_mps;
    frame.angular_rad_s = sample.raw.angular_velocity_rad_s;
    frame.valid = sample.raw.valid;
    return frame;
}

robot_slamd::ImuFrame make_imu_frame(
    const robot_slamd::SimImuSample &sample) {
    robot_slamd::ImuFrame frame;
    frame.timestamp_s = sample.raw.timestamp_s;
    frame.yaw_rate_rad_s = sample.raw.yaw_rate_rad_s;
    frame.ax_mps2 = sample.raw.accel_x_mps2;
    frame.ay_mps2 = sample.raw.accel_y_mps2;
    frame.az_mps2 = sample.raw.accel_z_mps2;
    return frame;
}

class EstimatedPoseMapPort final : public robot_slamd::RobotSlamMapPort {
public:
    explicit EstimatedPoseMapPort(
        std::shared_ptr<robot_slamd::SparseMultiTofOccupancyBackendBinding> backend)
        : backend_(std::move(backend)) {}

    void set_estimated_pose(const robot_slamd::RobotPose2D &pose) {
        estimated_pose_ = pose;
    }

    bool ready() const override {
        return backend_ && backend_->ready();
    }

    bool integrate_sensor_snapshot(
        const robot_slamd::RobotSlamSensorSnapshot &snapshot,
        double now_s) override {
        robot_slamd::SlamBackendInputFrame input;
        input.timestamp_s = now_s;
        input.snapshot = snapshot;
        input.has_predicted_pose = true;
        input.predicted_pose = estimated_pose_;
        input.source = "m3d1_wheel_imu_dead_reckoning_estimate";
        const auto result = backend_->update(input);
        return result.status == robot_slamd::SlamBackendUpdateStatus::Accepted &&
               result.map_updated;
    }

    robot_slamd::RobotSlamMapQuality latest_quality(double now_s) const override {
        return backend_->latest_quality(now_s);
    }

    bool save_map(const std::string &output_path) override {
        (void)output_path;
        return false;
    }

    std::string status() const override {
        return backend_ ? backend_->status() : "m3d1_backend_missing";
    }

private:
    std::shared_ptr<robot_slamd::SparseMultiTofOccupancyBackendBinding> backend_;
    robot_slamd::RobotPose2D estimated_pose_;
};
}

int main() {
    using namespace robot_slamd;
    const double dt = 0.02;
    auto clock = std::make_shared<SimClock>(100.0);
    auto world = std::make_shared<FakeWorld2D>();
    world->add_axis_aligned_room(-2.0, -1.5, 2.0, 1.5);
    world->add_axis_aligned_obstacle(0.7, -0.2, 0.9, 0.4);

    SimRobotPlantConfig plant_config;
    plant_config.max_angular_speed_rad_s = 4.0;
    plant_config.max_angular_accel_rad_s2 = 8.0;
    auto plant = std::make_shared<SimRobotPlant>(plant_config);
    RobotPose2D initial_pose;
    initial_pose.x_m = -0.4;
    initial_pose.y_m = -0.2;
    expect(plant->reset(initial_pose, clock->now_s()), "plant reset");

    SimMotionPortConfig motion_config;
    motion_config.max_angular_speed_rad_s = 4.0;
    auto motion = std::make_shared<SimMotionPort>(plant, motion_config);
    auto sensor = std::make_shared<SimSensorPort>(clock, world, plant);
    SimWheelEncoder odom_wheel;
    SimImu odom_imu;
    WheelImuDeadReckoningConfig odometry_config;
    odometry_config.wheel_radius_m = plant_config.wheel_radius_m;
    odometry_config.wheel_base_m = plant_config.wheel_base_m;
    odometry_config.maximum_dt_s = 0.10;
    odometry_config.timestamp_tolerance_s = 0.01;
    odometry_config.maximum_abs_wheel_linear_mps =
        plant_config.max_linear_speed_mps + 0.1;
    odometry_config.maximum_abs_wheel_angular_rad_s =
        plant_config.max_angular_speed_rad_s + 0.1;
    odometry_config.maximum_abs_imu_yaw_rate_rad_s =
        plant_config.max_angular_speed_rad_s + 0.1;
    odometry_config.maximum_yaw_rate_disagreement_rad_s = 0.05;
    WheelImuDeadReckoning2D odometry(odometry_config);
    expect(odometry.reset().ok, "odometry reset");

    SparseMultiTofOccupancyBackendOptions backend_options;
    backend_options.grid.resolution_m = 0.10;
    backend_options.minimum_accepted_snapshots_for_good_quality = 2;
    backend_options.minimum_valid_rays_for_good_quality = 6;
    backend_options.minimum_observed_cells_for_good_quality = 10;
    backend_options.minimum_angular_bins_for_good_quality = 2;
    auto backend =
        std::make_shared<SparseMultiTofOccupancyBackendBinding>(backend_options);
    auto map = std::make_shared<EstimatedPoseMapPort>(backend);

    AutonomousSlamCoordinatorOptions options;
    options.enabled = true;
    options.max_iterations = 120;
    options.motion_settle_timeout_s = 3.0;
    options.active_scan_speed = 0.05;
    options.active_scan_duration_s = 0.50;
    AutonomousSlamCoordinator coordinator(sensor, motion, map, options);

    int skipped_waiting_publish = 0;
    int backend_updates_before_wait = -1;
    bool backend_updated_while_waiting = false;
    bool saw_motion_command = false;
    bool saw_waiting_motion = false;
    bool saw_settle_then_publish = false;
    int publish_count = 0;
    bool start = true;

    for (int step = 0; step < 240; ++step) {
        const double now = clock->now_s();
        motion->update(now);
        const auto wheel_sample = odom_wheel.sample(plant->state(), now);
        const auto imu_sample = odom_imu.sample(plant->state(), now);
        const auto odom_result = odometry.update(
            make_wheel_frame(wheel_sample),
            make_imu_frame(imu_sample),
            imu_sample.fresh);
        expect(odom_result.ok, "odometry update from Wheel/IMU");
        map->set_estimated_pose(odometry.estimated_pose());

        const bool phase_allows_sensor =
            coordinator.state() == AutonomousSlamState::Idle ||
            coordinator.state() == AutonomousSlamState::WaitingForSensors ||
            coordinator.state() == AutonomousSlamState::Initializing ||
            coordinator.state() == AutonomousSlamState::Mapping ||
            coordinator.state() == AutonomousSlamState::IntegratingScan;

        AutonomousSlamStepInput input;
        input.now_s = now;
        input.start_requested = start;
        start = false;
        input.motion_feedback = motion->latest_feedback(now);
        if (phase_allows_sensor) {
            input.sensors = sensor->latest_snapshot(now);
            if (snapshot_has_any_payload(input.sensors)) {
                publish_count++;
                if (saw_waiting_motion) {
                    saw_settle_then_publish = true;
                }
            }
        } else {
            skipped_waiting_publish++;
            if (saw_motion_command && backend_updates_before_wait >= 0 &&
                backend->report().accepted_snapshot_count !=
                    backend_updates_before_wait) {
                backend_updated_while_waiting = true;
            }
        }

        const auto output = coordinator.step(input);
        if (output.command_sent) {
            saw_motion_command = true;
            backend_updates_before_wait =
                backend->report().accepted_snapshot_count;
        }
        if (coordinator.state() == AutonomousSlamState::WaitingMotionSettle) {
            saw_waiting_motion = true;
        }

        plant->step(dt, now + dt, world.get());
        expect(clock->advance(dt), "clock advance");
        if (coordinator.state() == AutonomousSlamState::Completed) {
            break;
        }
    }

    const auto report = backend->report();
    expect(saw_motion_command, "coordinator issued active scan command");
    expect(plant->state().pose.yaw_rad != initial_pose.yaw_rad,
           "plant actually turned");
    expect(report.native_multi_tof_backend_consumption,
           "native backend consumed multi-ToF");
    expect(!report.legacy_projection_consumed, "legacy projection not consumed");
    expect(report.accepted_snapshot_count >= 2, "settle after new map update");
    expect(report.active_cell_count > 0, "occupancy cells observed");
    expect(report.hit_ray_count >= 6, "three rays across snapshots");
    expect(report.angular_coverage_bins >= 2, "active scan changed direction bins");
    expect(skipped_waiting_publish > 0, "waiting motion skipped SLAM publish");
    expect(!backend_updated_while_waiting,
           "backend not updated while waiting for settle");
    expect(saw_settle_then_publish, "publish resumed after settle");
    expect(publish_count >= 2, "multiple sensor publishes");
    expect(!report.ground_truth_consumed, "backend did not consume ground truth");
    expect(!report.real_hardware_accessed && !report.real_motion_attempted,
           "no real hardware or motion");
    return failures ? 1 : 0;
}
