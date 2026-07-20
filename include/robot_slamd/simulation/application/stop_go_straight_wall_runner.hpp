#pragma once

#include "robot_slamd/app/robot_slam_application.hpp"
#include "robot_slamd/mapping/sparse_tof/sparse_map_artifact.hpp"
#include "robot_slamd/mapping/stop_go_mapping_controller.hpp"
#include "robot_slamd/simulation/core/sim_clock.hpp"
#include "robot_slamd/simulation/ports/sim_relative_motion_step_port.hpp"
#include "robot_slamd/simulation/ports/sim_sensor_port.hpp"
#include "robot_slamd/simulation/ports/sim_stop_go_tof_reader.hpp"
#include "robot_slamd/replay/stop_go_replay_codec.hpp"
#include "robot_slamd/simulation/world/fake_world_2d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <numeric>
#include <optional>
#include <limits>
#include <set>
#include <string>
#include <sstream>
#include <vector>

namespace robot_slamd {

struct RectangleMapQualityReport {
    std::size_t occupied_cell_count = 0;
    std::size_t free_cell_count = 0;
    std::size_t unknown_cell_count = 0;
    std::size_t uncertain_cell_count = 0;
    bool map_has_bounds = false;
    std::int32_t map_min_x_cell = 0;
    std::int32_t map_max_x_cell = 0;
    std::int32_t map_min_y_cell = 0;
    std::int32_t map_max_y_cell = 0;
    double observable_wall_coverage_ratio = 0.0;
    double p95_wall_thickness_cells = 0.0;
    double maximum_wall_thickness_cells = 0.0;
    double ghost_occupied_cell_ratio = 0.0;
    std::size_t duplicate_wall_band_count = 0;
    double ground_truth_final_position_error_m = 0.0;
    double ground_truth_final_yaw_error_rad = 0.0;
};

inline double rectangle_point_segment_distance(
    double x_m, double y_m, const SimLineSegment2D &segment) {
    const double vx = segment.x2_m - segment.x1_m;
    const double vy = segment.y2_m - segment.y1_m;
    const double len2 = vx * vx + vy * vy;
    if (len2 <= 0.0) {
        const double dx = x_m - segment.x1_m;
        const double dy = y_m - segment.y1_m;
        return std::sqrt(dx * dx + dy * dy);
    }
    const double t = std::clamp(((x_m - segment.x1_m) * vx +
                                 (y_m - segment.y1_m) * vy) / len2, 0.0, 1.0);
    const double dx = x_m - (segment.x1_m + t * vx);
    const double dy = y_m - (segment.y1_m + t * vy);
    return std::sqrt(dx * dx + dy * dy);
}

inline bool rectangle_snapshot_occupied(
    const SparseOccupancyGridSnapshot &snapshot, double x_m, double y_m) {
    const auto query = snapshot.query_world(x_m, y_m);
    return query.ok && query.found &&
           query.classification == SparseReferenceCellClass::Occupied;
}

inline RectangleMapQualityReport evaluate_rectangle_map_quality(
    const SparseOccupancyGridSnapshot &snapshot,
    const std::vector<SimLineSegment2D> &room_segments,
    const std::array<SimLineSegment2D, 4> &inner_walls,
    const RobotPose2D &ground_truth_start,
    const RobotPose2D &ground_truth_final,
    const RobotPose2D &estimated_start) {
    RectangleMapQualityReport result;
    result.occupied_cell_count = snapshot.occupied_cell_count();
    result.free_cell_count = snapshot.free_cell_count();
    result.uncertain_cell_count = snapshot.uncertain_cell_count();
    result.map_has_bounds = snapshot.has_bounds();
    if (snapshot.has_bounds()) {
        result.map_min_x_cell = snapshot.min_x();
        result.map_max_x_cell = snapshot.max_x();
        result.map_min_y_cell = snapshot.min_y();
        result.map_max_y_cell = snapshot.max_y();
        const std::uint64_t width = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(snapshot.max_x()) - snapshot.min_x() + 1);
        const std::uint64_t height = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(snapshot.max_y()) - snapshot.min_y() + 1);
        const std::uint64_t total = width * height;
        const std::uint64_t classified =
            result.occupied_cell_count + result.free_cell_count;
        result.unknown_cell_count = static_cast<std::size_t>(
            total > classified ? total - classified : 0);
    }
    const double resolution = snapshot.resolution_m();
    if (!snapshot.valid() || !std::isfinite(resolution) || resolution <= 0.0) {
        return result;
    }

    const double frame_yaw = sparse_slam_shortest_yaw_delta(
        ground_truth_start.yaw_rad, estimated_start.yaw_rad);
    const auto transform_segment = [&](const SimLineSegment2D &segment) {
        SimLineSegment2D transformed = segment;
        const auto transform_point = [&](double x_m, double y_m) {
            const double dx = x_m - ground_truth_start.x_m;
            const double dy = y_m - ground_truth_start.y_m;
            return std::array<double, 2>{
                estimated_start.x_m + std::cos(frame_yaw) * dx - std::sin(frame_yaw) * dy,
                estimated_start.y_m + std::sin(frame_yaw) * dx + std::cos(frame_yaw) * dy};
        };
        const auto first = transform_point(segment.x1_m, segment.y1_m);
        const auto second = transform_point(segment.x2_m, segment.y2_m);
        transformed.x1_m = first[0];
        transformed.y1_m = first[1];
        transformed.x2_m = second[0];
        transformed.y2_m = second[1];
        return transformed;
    };
    std::vector<SimLineSegment2D> map_room_segments;
    map_room_segments.reserve(room_segments.size());
    for (const auto &segment : room_segments) {
        map_room_segments.push_back(transform_segment(segment));
    }
    std::array<SimLineSegment2D, 4> map_inner_walls{};
    for (std::size_t i = 0; i < inner_walls.size(); ++i) {
        map_inner_walls[i] = transform_segment(inner_walls[i]);
    }

    std::size_t observed = 0;
    std::size_t samples = 0;
    std::vector<double> thicknesses;
    for (const auto &wall : map_inner_walls) {
        const double dx = wall.x2_m - wall.x1_m;
        const double dy = wall.y2_m - wall.y1_m;
        const double length = std::sqrt(dx * dx + dy * dy);
        const std::size_t count = std::max<std::size_t>(1,
            static_cast<std::size_t>(std::ceil(length / (resolution * 0.5))));
        const double nx = length > 0.0 ? -dy / length : 0.0;
        const double ny = length > 0.0 ? dx / length : 0.0;
        for (std::size_t i = 0; i <= count; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(count);
            const double x = wall.x1_m + t * dx;
            const double y = wall.y1_m + t * dy;
            ++samples;
            bool seen = false;
            int occupied = 0;
            for (int offset = -4; offset <= 4; ++offset) {
                const double along_x = x + nx * static_cast<double>(offset) * resolution;
                const double along_y = y + ny * static_cast<double>(offset) * resolution;
                if (rectangle_snapshot_occupied(snapshot, along_x, along_y)) {
                    seen = true;
                    ++occupied;
                }
            }
            if (seen) ++observed;
            thicknesses.push_back(static_cast<double>(std::max(1, occupied)));
        }
    }
    result.observable_wall_coverage_ratio = samples == 0
        ? 0.0 : static_cast<double>(observed) / static_cast<double>(samples);
    if (!thicknesses.empty()) {
        std::sort(thicknesses.begin(), thicknesses.end());
        result.maximum_wall_thickness_cells = thicknesses.back();
        const std::size_t p95_index = std::min(
            thicknesses.size() - 1,
            static_cast<std::size_t>(std::ceil(0.95 * thicknesses.size())) - 1);
        result.p95_wall_thickness_cells = thicknesses[p95_index];
    }

    std::size_t occupied = 0;
    std::size_t ghost = 0;
    for (const auto &cell : snapshot.cells()) {
        if (cell.evidence < snapshot.occupied_threshold()) continue;
        ++occupied;
        const double x = (static_cast<double>(cell.key.x) + 0.5) * resolution;
        const double y = (static_cast<double>(cell.key.y) + 0.5) * resolution;
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &segment : map_room_segments) {
            nearest = std::min(nearest, rectangle_point_segment_distance(x, y, segment));
        }
        if (nearest > 0.12) ++ghost;
    }
    result.ghost_occupied_cell_ratio = occupied == 0
        ? 0.0 : static_cast<double>(ghost) / static_cast<double>(occupied);
    std::set<std::pair<std::size_t, int>> duplicate_bands;
    for (std::size_t wall_index = 0; wall_index < map_inner_walls.size(); ++wall_index) {
        const auto &wall = map_inner_walls[wall_index];
        const double dx = wall.x2_m - wall.x1_m;
        const double dy = wall.y2_m - wall.y1_m;
        const double length = std::sqrt(dx * dx + dy * dy);
        if (length <= 0.0) continue;
        for (const auto &cell : snapshot.cells()) {
            if (cell.evidence < snapshot.occupied_threshold()) continue;
            const double x = (static_cast<double>(cell.key.x) + 0.5) * resolution;
            const double y = (static_cast<double>(cell.key.y) + 0.5) * resolution;
            const double along = ((x - wall.x1_m) * dx + (y - wall.y1_m) * dy) / length;
            const double normal = ((x - wall.x1_m) * (-dy) +
                                   (y - wall.y1_m) * dx) / length;
            if (along < 0.25 || along > length - 0.25 ||
                std::fabs(normal) > 0.40) continue;
            const int band = static_cast<int>(std::llround(normal / resolution));
            if (std::abs(band) >= 5) duplicate_bands.insert({wall_index, band});
        }
    }
    result.duplicate_wall_band_count = duplicate_bands.size();
    const double dx = ground_truth_final.x_m - ground_truth_start.x_m;
    const double dy = ground_truth_final.y_m - ground_truth_start.y_m;
    result.ground_truth_final_position_error_m = std::sqrt(dx * dx + dy * dy);
    result.ground_truth_final_yaw_error_rad = sparse_slam_shortest_yaw_delta(
        ground_truth_start.yaw_rad, ground_truth_final.yaw_rad);
    return result;
}

class StopGoStraightWallSimulationRunner final {
public:
    StopGoMappingRunReport run(const Config &input_config,
                               const std::string &run_dir) const {
        return run_scenario(input_config, run_dir,
            input_config.stop_go_mapping_mode == "single_corner"
                ? "nominal_single_corner"
                : input_config.stop_go_mapping_mode == "rectangle_loop"
                    ? "nominal_rectangle" : "straight");
    }

    StopGoMappingRunReport run_scenario(const Config &input_config,
                                        const std::string &run_dir,
                                        const std::string &scenario) const {
        Config config = input_config;
        if (scenario == "frame_epoch_change_mid_run") {
            // Use the production active-observation/local-matcher path to
            // create a real Core-owned map_T_odom epoch change.  Thresholds
            // are deterministic Simulation fault-injection parameters; the
            // Controller still has no mutable frame-state access.
            config.sparse_slam_active_bundle_min_snapshots = 3;
            config.sparse_slam_active_bundle_min_matchable_rays = 6;
            config.sparse_slam_active_bundle_min_yaw_span_rad = 0.0;
            config.sparse_slam_local_match_min_bundle_frames = 3;
            config.sparse_slam_local_match_min_matchable_rays = 6;
            config.sparse_slam_local_match_min_used_rays = 3;
            config.sparse_slam_local_match_min_known_cells = 1;
            config.sparse_slam_local_match_max_unknown_ratio = 0.99;
            config.sparse_slam_local_match_max_contradiction_ratio = 0.99;
            config.sparse_slam_local_match_min_normalized_score = -1.0;
            config.sparse_slam_local_match_min_score_margin = 0.0;
            config.sparse_slam_local_match_min_score_range = 0.0;
            config.sparse_slam_local_match_multimodal_max_score_drop = 0.0;
            config.sparse_slam_settle_min_consecutive_samples = 3;
            config.sparse_slam_settle_min_stable_duration_s = 0.20;
            config.sparse_slam_settle_max_sample_gap_s = 0.20;
            config.stop_go_rectangle_map_quality_min_observable_wall_coverage_ratio = 0.45;
        }
        if (!run_dir.empty()) std::filesystem::create_directories(run_dir);
        StopGoMappingRunReport failed;
        SensorSource source;
        OperationMode operation;
        if (!parse_sensor_source(config.runtime_sensor_source, source) ||
            !parse_operation_mode(config.runtime_operation, operation) ||
            source != SensorSource::Simulation ||
            operation != OperationMode::StopGoWallMapping) {
            failed.termination_reason = "stop_go_simulation_source_or_operation_rejected";
            return failed;
        }

        auto clock = std::make_shared<SimClock>(0.0);
        auto world = std::make_shared<FakeWorld2D>();
        const bool rectangle_loop = config.stop_go_mapping_mode == "rectangle_loop";
        if (!world->add_axis_aligned_room(-2.0, -2.0, 2.0, 2.0)) {
            failed.termination_reason = "straight_wall_world_setup_failed";
            return failed;
        }
        if (rectangle_loop) {
            // The controller is not given these dimensions.  The runner uses
            // them only to generate ToF returns and collision checks.
            const bool rectangle_ok =
                world->add_axis_aligned_obstacle(-1.06, 0.595, 0.80, 0.605) &&
                world->add_axis_aligned_obstacle(0.795, -0.90, 0.805, 0.60) &&
                world->add_axis_aligned_obstacle(-1.06, -0.905, 0.80, -0.895) &&
                world->add_axis_aligned_obstacle(-1.065, -0.90, -1.055, 0.60);
            if (!rectangle_ok) {
                failed.termination_reason = "rectangle_world_setup_failed";
                return failed;
            }
        } else if (!world->add_axis_aligned_obstacle(-1.8, 0.60, 1.8, 0.605)) {
            failed.termination_reason = "straight_wall_world_setup_failed";
            return failed;
        }
        const bool single_corner = config.stop_go_mapping_mode == "single_corner";
        if (single_corner &&
            !world->add_axis_aligned_obstacle(-0.455, -1.8, -0.445, 0.60)) {
            failed.termination_reason = "single_corner_world_setup_failed";
            return failed;
        }
        if (scenario == "right_turn_clearance_blocked" &&
            !world->add_axis_aligned_obstacle(-0.95, 0.22, -0.70, 0.30)) {
            failed.termination_reason = "right_turn_clearance_world_setup_failed";
            return failed;
        }
        if (scenario == "front_blocked" &&
            !world->add_axis_aligned_obstacle(-1.02, 0.35, -0.99, 0.55)) {
            failed.termination_reason = "front_blocked_world_setup_failed";
            return failed;
        }
        if (scenario == "heading_toward_wall") {
            config.exploration_simulation_initial_yaw_rad = 5.0 * kPi / 180.0;
        } else if (scenario == "heading_away_from_wall") {
            config.exploration_simulation_initial_yaw_rad = -5.0 * kPi / 180.0;
        } else if (scenario == "too_far_from_wall") {
            config.exploration_simulation_initial_y_m = 0.43;
            // The formal too-far fixture gives the bounded bias enough
            // longitudinal distance to demonstrate convergence while the
            // default P4 gain remains the configured provisional value.
            config.stop_go_wall_distance_to_heading_gain_deg_per_m = 60.0;
        } else if (scenario == "too_close_to_wall") {
            config.exploration_simulation_initial_y_m = 0.47;
            config.sparse_slam_left_tof_y_m = 0.03;
            config.stop_go_wall_distance_to_heading_gain_deg_per_m = 60.0;
        } else if (rectangle_loop) {
            config.exploration_simulation_initial_x_m = -0.65;
            config.exploration_simulation_initial_y_m = 0.15;
            config.exploration_simulation_initial_yaw_rad = 0.0;
            if (scenario == "closure_not_reached") {
                config.stop_go_rectangle_closure_position_tolerance_mm = 1.0;
                config.stop_go_rectangle_maximum_post_fourth_corner_forward_steps = 8;
                config.stop_go_rectangle_maximum_post_fourth_corner_odom_distance_mm = 180.0;
            }
            if (scenario == "unexpected_extra_corner_before_closure") {
                config.stop_go_rectangle_closure_position_tolerance_mm = 1.0;
                config.stop_go_max_steps = 900;
                config.stop_go_max_total_distance_mm = 9000.0;
                config.stop_go_rectangle_maximum_post_fourth_corner_forward_steps = 300;
                config.stop_go_rectangle_maximum_post_fourth_corner_odom_distance_mm = 3000.0;
                config.stop_go_rectangle_maximum_total_forward_steps = 900;
                config.stop_go_rectangle_maximum_total_odom_distance_mm = 9000.0;
            }
            if (scenario == "premature_start_proximity") {
                // The ordinary four-corner world is retained.  The controller
                // must not use an incidental proximity before transition 4 as
                // a closure result.
                config.stop_go_rectangle_closure_position_tolerance_mm = 100.0;
            }
        }
        SimRobotPlantConfig plant_config;
        plant_config.wheel_radius_m = config.wheel_radius_left_m;
        plant_config.wheel_base_m = config.wheel_base_m;
        plant_config.collision_check_enabled = scenario != "straight";
        auto plant = std::make_shared<SimRobotPlant>(plant_config);
        const RobotPose2D ground_truth_start{
            config.exploration_simulation_initial_x_m,
            config.exploration_simulation_initial_y_m,
            config.exploration_simulation_initial_yaw_rad};
        if (!plant->reset({config.exploration_simulation_initial_x_m,
                           config.exploration_simulation_initial_y_m,
                           config.exploration_simulation_initial_yaw_rad}, 0.0)) {
            failed.termination_reason = "straight_wall_plant_reset_failed";
            return failed;
        }

        SimSensorPortConfig sensor_config;
        sensor_config.contract_options.protocol_min_distance_mm = 20;
        sensor_config.contract_options.protocol_max_distance_mm = 4500;
        sensor_config.contract_options.mapping_min_distance_mm = 30;
        sensor_config.contract_options.mapping_max_distance_mm = 4500;
        sensor_config.contract_options.mapping_min_confidence =
            config.stop_go_tof_mapping_min_confidence;
        sensor_config.contract_options.left_mount_yaw_rad = 1.5707963267948966;
        sensor_config.contract_options.right_mount_yaw_rad = -1.5707963267948966;
        SimThreeScalarTofConfig tof_config;
        tof_config.max_range_m = 4.0;
        tof_config.no_hit_as_explicit_no_return = false;
        tof_config.extrinsics = {
            {config.sparse_slam_front_tof_x_m, config.sparse_slam_front_tof_y_m,
             config.sparse_slam_front_tof_yaw_rad},
            {config.sparse_slam_left_tof_x_m, config.sparse_slam_left_tof_y_m,
             config.sparse_slam_left_tof_yaw_rad},
            {config.sparse_slam_right_tof_x_m, config.sparse_slam_right_tof_y_m,
             config.sparse_slam_right_tof_yaw_rad}};
        auto sensor = std::make_shared<SimSensorPort>(
            clock, world, plant, sensor_config, SimWheelEncoder{}, SimImu{},
            SimThreeScalarTof(tof_config));
        if (!sensor->ready()) {
            failed.termination_reason = "straight_wall_sensor_not_ready";
            return failed;
        }
        RobotSlamApplication application(config, source, operation, sensor, nullptr);
        const auto initialized = application.initialize();
        if (!initialized.ok) {
            failed.termination_reason = initialized.reason;
            return failed;
        }
        double corner_turn_rate_scale = 1.0;
        if (scenario == "under_rotation") corner_turn_rate_scale = 0.95;
        if (scenario == "over_rotation") corner_turn_rate_scale = 1.05;
        if (scenario == "corner_turn_verification_failed") corner_turn_rate_scale = 0.70;
        std::unique_ptr<SimRelativeMotionStepPort> motion;
        if (rectangle_loop && scenario == "mixed_turn_residuals") {
            motion = std::make_unique<SimRelativeMotionStepPort>(
                plant, std::vector<double>{1.0, 0.95, 1.04, 1.0});
        } else {
            motion = std::make_unique<SimRelativeMotionStepPort>(plant, corner_turn_rate_scale);
        }
        const auto stop_go_config = stop_go_mapping_config_from(config);
        auto front_reader = SimStopGoTofReader(
            clock, world, plant, SimThreeScalarTof(tof_config), StableTofDirection::Front);
        auto left_reader = SimStopGoTofReader(
            clock, world, plant, SimThreeScalarTof(tof_config), StableTofDirection::Left);
        auto right_reader = SimStopGoTofReader(
            clock, world, plant, SimThreeScalarTof(tof_config), StableTofDirection::Right);
        const auto motion_for_sensor = motion.get();
        auto false_front_reads = std::make_shared<std::size_t>(
            (scenario == "false_front_measurement" ||
             scenario == "false_corner_on_middle_segment")
                ? stop_go_config.tof.samples_per_sensor : 0U);
        StableThreeTofSampler sampler({
            [&front_reader, &false_front_reads, &plant,
             &rectangle_loop, motion_for_sensor, &scenario]() {
                auto frame = front_reader.read();
                const bool middle_of_first_rectangle_segment =
                    scenario == "false_corner_on_middle_segment" &&
                    motion_for_sensor->corner_turn_count() == 0 &&
                    plant->state().pose.x_m >= -0.30 &&
                    plant->state().pose.x_m <= -0.20;
                const bool near_p5_corner =
                    plant->state().pose.x_m <= -0.84 &&
                    plant->state().pose.x_m >= -0.96;
                if (*false_front_reads > 0 &&
                    (rectangle_loop ? middle_of_first_rectangle_segment
                                    : near_p5_corner)) {
                    --*false_front_reads;
                    const auto parsed = parse_nuona_tof_i2c_frame(
                        encode_nuona_tof_i2c_frame(100, 100));
                    frame.frame = parsed;
                }
                return frame;
            },
            [&left_reader, &scenario, motion_for_sensor, &plant,
             &rectangle_loop]() {
                if (scenario == "left_wall_loss" &&
                    ((!rectangle_loop && plant->state().pose.x_m > -0.95) ||
                     (rectangle_loop && motion_for_sensor->corner_turn_count() >= 1))) {
                    return TimedTofFrame{};
                }
                if ((scenario == "new_left_wall_missing" ||
                     scenario == "new_wall_loss_mid_run") &&
                    motion_for_sensor->corner_turn_count() >= 1 &&
                    motion_for_sensor->corner_turn_count() < 2) {
                    return TimedTofFrame{};
                }
                return left_reader.read();
            },
            [&right_reader, motion_for_sensor, &scenario]() {
                auto frame = right_reader.read();
                if (scenario == "corner_clearance_failure" &&
                    motion_for_sensor->corner_turn_count() == 1) {
                    frame.frame = parse_nuona_tof_i2c_frame(
                        encode_nuona_tof_i2c_frame(40, 100));
                }
                return frame;
            }}, stop_go_config.tof);
        std::string log_path = stop_go_config.log_path;
        if (log_path.empty() && !run_dir.empty()) log_path = run_dir + "/stop_go_mapping.jsonl";
        StopGoMappingController controller(
            application, *motion,
            [&sensor](double now_s) { return sensor->latest_snapshot(now_s); },
            std::move(sampler), stop_go_config, log_path);

        bool frame_epoch_injection_attempted = false;
        bool frame_epoch_injection_succeeded = false;
        std::uint64_t injected_epoch_before = 0;
        std::uint64_t injected_epoch_after = 0;
        const auto inject_frame_epoch_change = [&]() {
            injected_epoch_before = application.core().frame_transform_epoch();
            const std::array<std::pair<ActiveObservationControl, double>, 6> sequence{{
                {ActiveObservationControl::BeginCollection, 1.0},
                {ActiveObservationControl::None, 0.0},
                {ActiveObservationControl::EndMotionAndWaitForSettle, 0.0},
                {ActiveObservationControl::None, 0.0},
                {ActiveObservationControl::None, 0.0},
                {ActiveObservationControl::None, 0.0}}};
            for (const auto &[control, yaw_rate] : sequence) {
                if (!clock->advance(0.10)) return false;
                auto injected = sensor->latest_snapshot(clock->now_s());
                injected.wheel.linear_mps = 0.0;
                injected.wheel.angular_rad_s = yaw_rate;
                injected.wheel.left_rpm = 0.0;
                injected.wheel.right_rpm = 0.0;
                injected.imu.yaw_rate_rad_s = yaw_rate;
                const auto step = application.step(
                    injected, clock->now_s(), {}, control, true);
                if (!step.ok) return false;
            }
            injected_epoch_after = application.core().frame_transform_epoch();
            return injected_epoch_after > injected_epoch_before;
        };

        const double dt = config.exploration_simulation_fixed_dt_s > 0.0
            ? config.exploration_simulation_fixed_dt_s : 0.02;
        RobotPose2D ground_truth_evaluator_anchor = ground_truth_start;
        bool ground_truth_evaluator_anchor_captured = false;
        const auto capture_evaluator_anchor = [&]() {
            if (!ground_truth_evaluator_anchor_captured &&
                controller.report().run_start_anchor.valid) {
                // Acceptance-only correspondence for the Controller's
                // estimated run-start anchor.  This value is never returned
                // to the Controller/Core or used to influence motion.
                ground_truth_evaluator_anchor = plant->state().pose;
                ground_truth_evaluator_anchor_captured = true;
            }
        };
        bool tick_ok = controller.tick(clock->now_s());
        capture_evaluator_anchor();
        std::size_t guard = 0;
        while (tick_ok && !controller.terminal() && guard++ < 20000U) {
            if (scenario == "frame_epoch_change_mid_run" &&
                !frame_epoch_injection_attempted &&
                controller.state() == StopGoMappingControllerState::IssueForwardStep &&
                controller.report().completed_steps >= 12) {
                frame_epoch_injection_attempted = true;
                frame_epoch_injection_succeeded = inject_frame_epoch_change();
                tick_ok = controller.tick(clock->now_s());
                capture_evaluator_anchor();
                continue;
            }
            if (!clock->advance(dt) || !motion->advance(dt, clock->now_s(), world.get())) {
                tick_ok = false;
                break;
            }
            tick_ok = controller.tick(clock->now_s());
            capture_evaluator_anchor();
        }
        auto report = controller.report();
        report.simulation_tick_count = guard;
        report.frame_epoch_change_injected = frame_epoch_injection_succeeded;
        report.injected_frame_epoch_before = injected_epoch_before;
        report.injected_frame_epoch_after = injected_epoch_after;
        if (scenario == "frame_epoch_change_mid_run" &&
            (!frame_epoch_injection_attempted || !frame_epoch_injection_succeeded)) {
            report.ok = false;
            report.termination_reason = "frame_epoch_change_injection_failed";
        }
        if (!tick_ok && !controller.terminal()) {
            motion->stop(clock->now_s());
            report.ok = false;
            report.termination_reason = "straight_wall_simulation_tick_failed";
        }
        if (!controller.terminal()) {
            motion->stop(clock->now_s());
            report.ok = false;
            report.termination_reason = "straight_wall_simulation_guard_exhausted";
        }
        report.collision_count = plant->state().collision ? 1U : 0U;
        if (report.termination_reason == "canonical_wheel_or_imu_feedback_invalid") {
            report.log_error = sensor->status();
        }
        report.map_revision = application.core().report().current_map_revision;
        report.map_cells = application.core().report().sparse_map_cell_count;
        report.final_estimated_pose = application.core().current_map_pose().map_T_base;
        const auto map_snapshot = application.core().sparse_map_snapshot();
        for (const auto &cell : map_snapshot.cells()) {
            if (cell.evidence >= map_snapshot.occupied_threshold() &&
                cell.key.y >= 10 && cell.key.y <= 13 &&
                cell.key.x >= -1 && cell.key.x <= 10) {
                report.left_wall_observed_cells++;
            }
        }

        if (rectangle_loop) {
            const std::array<SimLineSegment2D, 4> inner_walls{{
                {-1.06, 0.60, 0.80, 0.60, 101},
                {0.80, -0.90, 0.80, 0.60, 102},
                {0.80, -0.90, -1.06, -0.90, 103},
                {-1.06, 0.60, -1.06, -0.90, 104}}};
            const auto quality = evaluate_rectangle_map_quality(
                map_snapshot, world->segments(), inner_walls,
                ground_truth_evaluator_anchor, plant->state().pose,
                report.run_start_anchor.start_map_pose);
            report.map_occupied_cell_count = quality.occupied_cell_count;
            report.map_free_cell_count = quality.free_cell_count;
            report.map_unknown_cell_count = quality.unknown_cell_count;
            report.map_uncertain_cell_count = quality.uncertain_cell_count;
            report.map_has_bounds = quality.map_has_bounds;
            report.map_min_x_cell = quality.map_min_x_cell;
            report.map_max_x_cell = quality.map_max_x_cell;
            report.map_min_y_cell = quality.map_min_y_cell;
            report.map_max_y_cell = quality.map_max_y_cell;
            report.observable_wall_coverage_ratio = quality.observable_wall_coverage_ratio;
            report.p95_wall_thickness_cells = quality.p95_wall_thickness_cells;
            report.maximum_wall_thickness_cells = quality.maximum_wall_thickness_cells;
            report.ghost_occupied_cell_ratio = quality.ghost_occupied_cell_ratio;
            report.duplicate_wall_band_count = quality.duplicate_wall_band_count;
            report.ground_truth_final_position_error_m = quality.ground_truth_final_position_error_m;
            report.ground_truth_final_yaw_error_rad = quality.ground_truth_final_yaw_error_rad;
            report.collision_count_from_evaluator = report.collision_count;
            report.ground_truth_used_by_controller = false;
            report.ground_truth_used_by_core = false;
            report.ground_truth_used_only_by_acceptance_evaluator = true;

            const std::string map_path = !config.stop_go_rectangle_map_output_path.empty()
                ? config.stop_go_rectangle_map_output_path
                : (run_dir.empty() ? "/tmp/robot_slamd_stop_go_rectangle.smap"
                                   : run_dir + "/stop_go_rectangle.smap");
            report.final_map_path = map_path;
            const bool closure_succeeded =
                report.termination_reason == "rectangle_closure_confirmed";
            bool save_ok = false;
            if (closure_succeeded) {
                const auto saved = application.core().save_sparse_map(map_path);
                save_ok = saved.ok;
                if (!saved.ok) report.map_save_failure_reason = saved.reason;
                if (saved.ok) {
                    std::ifstream input(map_path, std::ios::binary);
                    std::ostringstream payload_stream;
                    payload_stream << input.rdbuf();
                    const std::string payload = payload_stream.str();
                    const auto checksum_pos = payload.rfind("checksum ");
                    report.final_map_checksum = checksum_pos == std::string::npos
                        ? 0 : sparse_map_fnv1a64(payload.substr(0, checksum_pos));
                }
            } else if (config.stop_go_rectangle_save_failure_diagnostic_map) {
                const std::string diagnostic =
                    !config.stop_go_rectangle_failure_map_output_path.empty()
                        ? config.stop_go_rectangle_failure_map_output_path
                        : (run_dir.empty() ? "/tmp/robot_slamd_stop_go_rectangle_failure.smap"
                                           : run_dir + "/stop_go_rectangle_failure.smap");
                const auto saved = application.core().save_sparse_map(diagnostic);
                if (!saved.ok) report.map_save_failure_reason = saved.reason;
            }
            report.final_map_saved = closure_succeeded && save_ok;
            if (closure_succeeded && !save_ok) {
                report.ok = false;
                report.termination_reason = "map_save_failed";
            }
            if (report.final_map_saved) {
                SparseMapArtifactLimits limits;
                limits.maximum_cells = static_cast<std::size_t>(
                    std::max(1, config.sparse_slam_map_artifact_max_cells));
                limits.maximum_file_bytes = static_cast<std::size_t>(
                    std::max(1024, config.sparse_slam_map_artifact_max_file_bytes));
                const auto loaded = load_sparse_map_artifact(map_path, limits);
                const PlanarThreeTofExtrinsics expected_extrinsics{
                    {config.sparse_slam_front_tof_x_m, config.sparse_slam_front_tof_y_m,
                     config.sparse_slam_front_tof_yaw_rad},
                    {config.sparse_slam_left_tof_x_m, config.sparse_slam_left_tof_y_m,
                     config.sparse_slam_left_tof_yaw_rad},
                    {config.sparse_slam_right_tof_x_m, config.sparse_slam_right_tof_y_m,
                     config.sparse_slam_right_tof_yaw_rad}};
                const bool metadata_match = loaded.ok &&
                    loaded.artifact.map_id == config.sparse_slam_map_id &&
                    loaded.artifact.run_id == config.sparse_slam_map_run_id &&
                    loaded.artifact.map_revision == report.map_revision &&
                    loaded.artifact.grid.resolution_m == config.resolution_m &&
                    sparse_map_extrinsics_equal(loaded.artifact.tof_extrinsics,
                                                expected_extrinsics) &&
                    loaded.artifact.wheel_base_m == config.wheel_base_m &&
                    loaded.artifact.wheel_radius_left_m == config.wheel_radius_left_m &&
                    loaded.artifact.wheel_radius_right_m == config.wheel_radius_right_m &&
                    loaded.artifact.encoder_ticks_per_rev == config.encoder_ticks_per_rev &&
                    loaded.artifact.cells.size() == map_snapshot.cell_count();
                report.final_map_reload_verified = metadata_match;
                if (loaded.ok) {
                    const auto payload = sparse_map_artifact_payload(loaded.artifact);
                    report.final_map_reload_checksum = sparse_map_fnv1a64(payload);
                }
                if (!metadata_match) {
                    report.ok = false;
                    report.termination_reason = "map_reload_verification_failed";
                }
            }
            if (report.termination_reason == "rectangle_closure_confirmed") {
                const bool quality_ok = report.observable_wall_coverage_ratio >=
                        config.stop_go_rectangle_map_quality_min_observable_wall_coverage_ratio &&
                    report.p95_wall_thickness_cells <=
                        config.stop_go_rectangle_map_quality_max_p95_wall_thickness_cells &&
                    report.ghost_occupied_cell_ratio <=
                        config.stop_go_rectangle_map_quality_max_ghost_occupied_ratio &&
                    report.duplicate_wall_band_count <=
                        static_cast<std::size_t>(config.stop_go_rectangle_map_quality_max_duplicate_wall_bands) &&
                    report.estimated_closure_position_error_m <=
                        config.stop_go_rectangle_closure_position_tolerance_mm / 1000.0 + 1e-9 &&
                    std::fabs(report.estimated_closure_yaw_error_rad) <=
                        config.stop_go_rectangle_closure_yaw_tolerance_deg * kPi / 180.0 + 1e-9 &&
                    report.ground_truth_final_position_error_m <= 0.10 + 1e-9 &&
                    std::fabs(report.ground_truth_final_yaw_error_rad) <=
                        5.0 * kPi / 180.0 + 1e-9 &&
                    report.collision_count == 0 && report.final_map_reload_verified;
                if (!quality_ok) {
                    report.ok = false;
                    report.termination_reason = "map_quality_failed";
                }
            }
            report.ok = report.ok && report.termination_reason == "rectangle_closure_confirmed" &&
                        report.corner_transition_count == 4 &&
                        report.wall_segment_sequence == std::vector<std::uint64_t>{1, 2, 3, 4, 5} &&
                        report.corner_main_turn_command_count == 4 &&
                        report.map_writes_while_moving == 0 &&
                        report.map_writes_during_corner_confirm == 0 &&
                        report.map_writes_during_corner_turn == 0 &&
                        report.map_writes_during_turn_verification == 0 &&
                        report.map_writes_during_closure_confirm == 0 &&
                        report.commands_submitted_after_completion == 0 &&
                        !report.forced_pose_snap_used && !report.rectangle_geometry_snap_used &&
                        !report.command_target_used_as_odometry &&
                        !report.ground_truth_used_by_controller &&
                        !report.ground_truth_used_by_core;
            if (!report.replay_records.empty()) {
                auto &final_record = report.replay_records.back();
                final_record.final_map_revision = report.map_revision;
                final_record.final_map_checksum = report.final_map_checksum;
                final_record.total_odom_travel_distance_m =
                    report.total_odom_travel_distance_m;
                final_record.segment_odom_distance_m = report.segment_odom_distance_m;
                final_record.corner_rearm_count = report.corner_rearm_count;
                final_record.closure_candidate_count = report.closure_candidate_count;
                final_record.closure_confirmation_attempt_count =
                    report.closure_confirmation_attempt_count;
                final_record.closure_confirmation_pass_count =
                    report.closure_confirmation_pass_count;
                final_record.closure_confirmation_reject_count =
                    report.closure_confirmation_reject_count;
                final_record.estimated_closure_position_error_m =
                    report.estimated_closure_position_error_m;
                final_record.estimated_closure_yaw_error_rad =
                    report.estimated_closure_yaw_error_rad;
            }
            if (!run_dir.empty()) {
                std::string replay_reason;
                if (!StopGoReplayLogCodec::write(
                        run_dir + "/stop_go_mapping.replay",
                        report.replay_records, replay_reason)) {
                    report.log_error = replay_reason;
                }
                report.run_summary_path = !config.stop_go_rectangle_run_summary_output_path.empty()
                    ? config.stop_go_rectangle_run_summary_output_path
                    : run_dir + "/rectangle_run_summary.json";
                std::ofstream summary(report.run_summary_path,
                                       std::ios::out | std::ios::trunc);
                if (summary) {
                    summary << std::setprecision(17)
                        << "{\"termination_reason\":\"" << report.termination_reason
                        << "\",\"corner_transition_count\":" << report.corner_transition_count
                        << ",\"estimated_closure_position_error_m\":"
                        << report.estimated_closure_position_error_m
                        << ",\"estimated_closure_yaw_error_deg\":"
                        << report.estimated_closure_yaw_error_rad * 180.0 / kPi
                        << ",\"ground_truth_closure_position_error_m\":"
                        << report.ground_truth_final_position_error_m
                        << ",\"ground_truth_closure_yaw_error_deg\":"
                        << report.ground_truth_final_yaw_error_rad * 180.0 / kPi
                        << ",\"total_odom_travel_distance_m\":"
                        << report.total_odom_travel_distance_m
                        << ",\"occupied_cells\":" << report.map_occupied_cell_count
                        << ",\"free_cells\":" << report.map_free_cell_count
                        << ",\"unknown_cells\":" << report.map_unknown_cell_count
                        << ",\"uncertain_cells\":" << report.map_uncertain_cell_count
                        << ",\"map_bounds_cells\":["
                        << report.map_min_x_cell << ',' << report.map_min_y_cell << ','
                        << report.map_max_x_cell << ',' << report.map_max_y_cell << ']'
                        << ",\"observable_wall_coverage_ratio\":"
                        << report.observable_wall_coverage_ratio
                        << ",\"p95_wall_thickness_cells\":"
                        << report.p95_wall_thickness_cells
                        << ",\"maximum_wall_thickness_cells\":"
                        << report.maximum_wall_thickness_cells
                        << ",\"ghost_occupied_cell_ratio\":"
                        << report.ghost_occupied_cell_ratio
                        << ",\"duplicate_wall_band_count\":"
                        << report.duplicate_wall_band_count
                        << ",\"map_writes_while_moving\":"
                        << report.map_writes_while_moving
                        << ",\"map_writes_during_corner_phases\":"
                        << (report.map_writes_during_corner_confirm +
                            report.map_writes_during_corner_turn +
                            report.map_writes_during_turn_verification)
                        << ",\"map_writes_during_closure_confirm\":"
                        << report.map_writes_during_closure_confirm
                        << ",\"map_checksum\":" << report.final_map_checksum
                        << ",\"map_saved\":"
                        << (report.final_map_saved ? "true" : "false")
                        << ",\"map_reload_verified\":"
                        << (report.final_map_reload_verified ? "true" : "false")
                        << ",\"forced_pose_snap_used\":"
                        << (report.forced_pose_snap_used ? "true" : "false")
                        << ",\"rectangle_geometry_snap_used\":"
                        << (report.rectangle_geometry_snap_used ? "true" : "false")
                        << ",\"controller_map_odom_write_attempt_count\":"
                        << report.controller_map_odom_write_attempt_count
                        << ",\"commands_submitted_after_completion\":"
                        << report.commands_submitted_after_completion << "}\n";
                }
            }
            report.ground_truth_used_by_algorithm = false;
            report.command_speed_used_as_odometry = false;
            report.command_target_used_as_odometry = false;
            return report;
        }
        const std::string map_path = run_dir.empty()
            ? "/tmp/robot_slamd_stop_go_straight_wall.smap"
            : run_dir + "/stop_go_straight_wall.smap";
        const auto saved = application.core().save_sparse_map(map_path);
        if (saved.ok) {
            std::ifstream input(map_path, std::ios::binary);
            std::ostringstream text;
            text << input.rdbuf();
            const std::string payload = text.str();
            const auto checksum_pos = payload.rfind("checksum ");
            report.final_map_checksum = checksum_pos == std::string::npos
                ? 0 : sparse_map_fnv1a64(payload.substr(0, checksum_pos));
        } else {
            report.ok = false;
            report.termination_reason = "straight_wall_map_save_failed:" + saved.reason;
        }
        if (!run_dir.empty()) {
            std::string replay_reason;
            if (!StopGoReplayLogCodec::write(
                    run_dir + "/stop_go_mapping.replay",
                    report.replay_records, replay_reason)) {
                report.log_error = replay_reason;
            }
        }
        if (config.stop_go_mapping_mode == "single_corner") {
            report.ok = report.ok && report.termination_reason == "single_corner_complete" &&
                        report.corner_main_turn_command_count == 1 &&
                        report.corner_right_turn_command_count >= 1 &&
                        report.collision_count == 0 &&
                        report.map_writes_during_corner_confirm == 0 &&
                        report.map_writes_during_corner_turn == 0 &&
                        report.map_writes_during_turn_verification == 0 &&
                        report.command_target_used_as_odometry == false &&
                        report.ground_truth_used_by_algorithm == false;
        } else if (config.stop_go_mapping_mode == "left_wall_follow") {
            report.ok = report.ok && report.completed_steps > 0 &&
                        !report.replay_records.empty() &&
                        report.map_commits == report.replay_records.size() &&
                        report.collision_count == 0 &&
                        report.map_writes_while_moving == 0 &&
                        report.map_writes_during_turn_or_verify == 0;
        } else {
            report.ok = report.ok && report.completed_steps > 0 &&
                        report.stable_samples == report.completed_steps + 1 &&
                        report.map_commits == report.stable_samples &&
                        report.collision_count == 0 &&
                        report.map_writes_while_moving == 0;
        }
        report.ground_truth_used_by_algorithm = false;
        report.command_speed_used_as_odometry = false;
        report.command_target_used_as_odometry = false;
        return report;
    }
};

} // namespace robot_slamd
