#pragma once

#include "robot_slamd/exploration/safe_waypoint_tracker.hpp"
#include "robot_slamd/exploration/sparse_frontier_planner.hpp"
#include "robot_slamd/runtime/sparse_slam_runtime_core.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace robot_slamd {

enum class AutonomousExplorationState {
    Initializing,
    BootstrapMapping,
    Planning,
    AligningToPath,
    FollowingPath,
    WaitingForSettle,
    Replanning,
    Completed,
    Failed
};

inline const char *to_string(AutonomousExplorationState state) {
    switch (state) {
    case AutonomousExplorationState::Initializing: return "initializing";
    case AutonomousExplorationState::BootstrapMapping: return "bootstrap_mapping";
    case AutonomousExplorationState::Planning: return "planning";
    case AutonomousExplorationState::AligningToPath: return "aligning_to_path";
    case AutonomousExplorationState::FollowingPath: return "following_path";
    case AutonomousExplorationState::WaitingForSettle: return "waiting_for_settle";
    case AutonomousExplorationState::Replanning: return "replanning";
    case AutonomousExplorationState::Completed: return "completed";
    case AutonomousExplorationState::Failed: return "failed";
    }
    return "unknown";
}

struct AutonomousExplorationConfig {
    NavigationGridViewConfig navigation;
    SparseFrontierPlannerConfig frontier;
    SafeWaypointTrackerConfig tracker;
    std::size_t no_frontier_confirmation_cycles = 3;
    std::size_t bootstrap_minimum_known_cells = 80;
    double bootstrap_minimum_yaw_rad = 5.5;
    double bootstrap_max_duration_s = 18.0;
    double maximum_duration_s = 120.0;
    std::size_t maximum_planning_failures = 20;
    double bootstrap_turn_speed_normalized = 0.30;
    double bootstrap_turn_segment_duration_s = 0.30;
    double command_ttl_s = 0.60;
};

struct AutonomousExplorationReport {
    AutonomousExplorationState state = AutonomousExplorationState::Initializing;
    std::string termination_reason = "not_terminated";
    std::size_t known_cells_start = 0;
    std::size_t known_cells_end = 0;
    std::size_t occupied_cells_start = 0;
    std::size_t occupied_cells_end = 0;
    std::size_t free_cells_start = 0;
    std::size_t free_cells_end = 0;
    std::uint64_t map_revision_start = 0;
    std::uint64_t map_revision_end = 0;
    std::size_t frontiers_detected = 0;
    std::size_t reachable_frontiers = 0;
    std::size_t selected_goals = 0;
    std::size_t completed_goals = 0;
    std::size_t failed_goals = 0;
    std::size_t plan_count = 0;
    std::size_t replan_count = 0;
    std::size_t planning_failure_count = 0;
    std::string last_planning_reason = "not_planned";
    std::size_t expanded_astar_nodes = 0;
    std::size_t commanded_turns = 0;
    std::size_t commanded_forward_segments = 0;
    std::size_t obstacle_stops = 0;
    std::size_t matcher_attempts = 0;
    std::size_t atomic_commit_successes = 0;
    std::size_t keyframe_count = 0;
    double bootstrap_accumulated_yaw_rad = 0.0;
    double estimated_travel_distance_m = 0.0;
    RobotPose2D final_estimated_map_pose;
};

struct AutonomousExplorationStepResult {
    bool ok = false;
    bool terminal = false;
    std::string reason;
};

class AutonomousExplorationController {
public:
    explicit AutonomousExplorationController(
        AutonomousExplorationConfig config = {})
        : config_(config), frontier_planner_(config.frontier),
          tracker_(config.tracker) {}

    AutonomousExplorationStepResult step(
        const RobotSlamSensorSnapshot &snapshot,
        double now_s,
        const RobotSlamMotionFeedback &motion_feedback,
        SparseSlamRuntimeCore &core,
        RobotSlamMotionPort &motion_port) {
        if (!std::isfinite(now_s) || now_s < last_now_s_) {
            return fail("exploration_non_monotonic_time");
        }
        if (start_time_s_ < 0.0) start_time_s_ = now_s;
        last_now_s_ = now_s;
        if (now_s - start_time_s_ > config_.maximum_duration_s) {
            send_stop(now_s, motion_port, "exploration_timeout_stop");
            return fail("maximum_exploration_duration_exceeded");
        }

        SparseSlamStepRequest request;
        request.snapshot = snapshot;
        request.now_s = now_s;
        if (discard_required_) {
            request.observation_control = ActiveObservationControl::DiscardFrozenBundle;
            request.bundle_id = core.active_bundle_id();
        } else if (begin_collection_required_) {
            request.observation_control = ActiveObservationControl::BeginCollection;
        } else if (end_collection_required_) {
            request.observation_control = ActiveObservationControl::EndMotionAndWaitForSettle;
        }
        const auto core_step = core.step(request);
        if (discard_required_) {
            if (!core_step.ok) return fail("exploration_bundle_discard_failed");
            discard_required_ = false;
            active_collection_ = false;
            alignment_replan_required_ = false;
            state_ = AutonomousExplorationState::Replanning;
        } else if (begin_collection_required_) {
            if (!core_step.ok) return fail("exploration_bundle_begin_failed");
            begin_collection_required_ = false;
            active_collection_ = true;
        } else if (end_collection_required_) {
            if (!core_step.ok) return fail("exploration_bundle_end_failed");
            end_collection_required_ = false;
            state_ = AutonomousExplorationState::WaitingForSettle;
        } else if (!core_step.ok && state_ != AutonomousExplorationState::Initializing) {
            return fail("exploration_sparse_core_step_failed:" + core_step.status);
        }

        if (state_ == AutonomousExplorationState::Initializing) {
            if (!core.report().initialization_complete) {
                report_.state = state_;
                return {true, false, "exploration_waiting_initialization"};
            }
            const auto map = core.sparse_map_snapshot();
            record_map_start(map);
            state_ = AutonomousExplorationState::BootstrapMapping;
            bootstrap_start_s_ = now_s;
            last_bootstrap_yaw_ = core.current_map_pose().map_T_base.yaw_rad;
            have_bootstrap_yaw_ = true;
        }

        const RobotPose2D estimated_pose = core.current_map_pose().map_T_base;
        update_bootstrap_yaw(estimated_pose.yaw_rad);

        if (state_ == AutonomousExplorationState::WaitingForSettle) {
            const auto active = core.active_observation_state();
            if (active == ActiveObservationBundleState::Idle) {
                active_collection_ = false;
                state_ = alignment_replan_required_
                    ? AutonomousExplorationState::Replanning
                    : AutonomousExplorationState::FollowingPath;
                alignment_replan_required_ = false;
            } else if (active == ActiveObservationBundleState::FrozenReady ||
                       active == ActiveObservationBundleState::Aborted) {
                const auto *match = core.local_match_result();
                if (match && !match->accepted) {
                    excluded_frontiers_.insert(current_frontier_id_);
                    report_.failed_goals++;
                    discard_required_ = true;
                } else if (active == ActiveObservationBundleState::Aborted) {
                    discard_required_ = true;
                }
            }
            sync_report(core, estimated_pose);
            return {true, false, "exploration_waiting_local_slam_settle"};
        }

        if (state_ == AutonomousExplorationState::BootstrapMapping) {
            const auto map = core.sparse_map_snapshot();
            const bool enough = map.valid() &&
                map.cell_count() >= config_.bootstrap_minimum_known_cells &&
                report_.bootstrap_accumulated_yaw_rad >=
                    config_.bootstrap_minimum_yaw_rad;
            const bool expired = now_s - bootstrap_start_s_ >=
                config_.bootstrap_max_duration_s;
            if (enough || expired) {
                if (!map.valid() ||
                    map.cell_count() < config_.bootstrap_minimum_known_cells) {
                    return fail("bootstrap_mapping_insufficient");
                }
                if (motion_feedback.command_active) {
                    send_stop(now_s, motion_port, "bootstrap_complete_stop");
                }
                if (motion_feedback.command_settled) {
                    state_ = AutonomousExplorationState::Planning;
                }
            } else if (!motion_feedback.command_active &&
                       motion_feedback.command_settled) {
                if (!send_motion(now_s, AlgorithmMotionKind::TurnLeft,
                                 config_.bootstrap_turn_speed_normalized,
                                 config_.bootstrap_turn_segment_duration_s,
                                 "bootstrap_mapping_turn", motion_port)) {
                    return fail("bootstrap_motion_rejected");
                }
                report_.commanded_turns++;
            }
            sync_report(core, estimated_pose);
            return {true, false, "bootstrap_mapping_active"};
        }

        if (state_ == AutonomousExplorationState::Replanning) {
            if (motion_feedback.command_active) {
                send_stop(now_s, motion_port, "exploration_replan_stop");
            }
            if (motion_feedback.command_settled) {
                state_ = AutonomousExplorationState::Planning;
                report_.replan_count++;
            }
            sync_report(core, estimated_pose);
            return {true, false, "exploration_replanning"};
        }

        if (state_ == AutonomousExplorationState::Planning) {
            if (!motion_feedback.command_settled || motion_feedback.command_active) {
                sync_report(core, estimated_pose);
                return {true, false, "exploration_waiting_to_plan"};
            }
            if (!make_plan(core, estimated_pose)) {
                if (report_.planning_failure_count >=
                    config_.maximum_planning_failures) {
                    return fail("maximum_planning_failures_exceeded");
                }
                state_ = AutonomousExplorationState::Replanning;
                sync_report(core, estimated_pose);
                return {true, false, "exploration_plan_retry"};
            }
            if (state_ == AutonomousExplorationState::Completed) {
                send_stop(now_s, motion_port, "exploration_completed_stop");
                sync_report(core, estimated_pose);
                report_.termination_reason = "no_reachable_frontier";
                return {true, true, report_.termination_reason};
            }
        }

        if (state_ == AutonomousExplorationState::AligningToPath) {
            if (!active_collection_ && !begin_collection_required_) {
                begin_collection_required_ = true;
                sync_report(core, estimated_pose);
                return {true, false, "exploration_begin_alignment_bundle"};
            }
            if (alignment_replan_required_) {
                if (active_collection_ && motion_feedback.command_settled &&
                    !motion_feedback.command_active) {
                    end_collection_required_ = true;
                }
                sync_report(core, estimated_pose);
                return {true, false, "exploration_obstacle_waiting_settle"};
            }
            const auto *target = tracker_.current_waypoint();
            if (!target) return fail("exploration_alignment_target_missing");
            const double target_yaw = std::atan2(target->y_m - estimated_pose.y_m,
                                                 target->x_m - estimated_pose.x_m);
            const double yaw_error = normalize(target_yaw - estimated_pose.yaw_rad);
            if (active_collection_ && motion_feedback.command_settled &&
                !motion_feedback.command_active &&
                (alignment_replan_required_ ||
                 std::fabs(yaw_error) <= config_.tracker.yaw_tolerance_rad)) {
                end_collection_required_ = true;
                sync_report(core, estimated_pose);
                return {true, false, "exploration_end_alignment_bundle"};
            }
            auto tracked = tracker_.step(
                tracker_input(snapshot, now_s, motion_feedback, estimated_pose),
                motion_port);
            if (!tracked.ok) return fail(tracked.reason);
            record_tracker_command(tracked);
            if (tracked.replan_required) {
                excluded_frontiers_.insert(current_frontier_id_);
                remember_failed_goal();
                report_.failed_goals++;
                alignment_replan_required_ = true;
            }
            sync_report(core, estimated_pose);
            return {true, false, tracked.reason};
        }

        if (state_ == AutonomousExplorationState::FollowingPath) {
            const auto *target = tracker_.current_waypoint();
            if (target && motion_feedback.command_settled &&
                !motion_feedback.command_active) {
                const double target_yaw = std::atan2(
                    target->y_m - estimated_pose.y_m,
                    target->x_m - estimated_pose.x_m);
                if (std::fabs(normalize(target_yaw - estimated_pose.yaw_rad)) >
                    config_.tracker.yaw_tolerance_rad) {
                    state_ = AutonomousExplorationState::AligningToPath;
                    begin_collection_required_ = true;
                    sync_report(core, estimated_pose);
                    return {true, false, "exploration_path_turn_requires_bundle"};
                }
            }
            auto tracked = tracker_.step(
                tracker_input(snapshot, now_s, motion_feedback, estimated_pose),
                motion_port);
            if (!tracked.ok) return fail(tracked.reason);
            record_tracker_command(tracked);
            if (tracked.goal_reached) {
                report_.completed_goals++;
                state_ = AutonomousExplorationState::Replanning;
            } else if (tracked.replan_required) {
                report_.failed_goals++;
                excluded_frontiers_.insert(current_frontier_id_);
                remember_failed_goal();
                state_ = AutonomousExplorationState::Replanning;
            }
            sync_report(core, estimated_pose);
            return {true, false, tracked.reason};
        }

        sync_report(core, estimated_pose);
        return {state_ != AutonomousExplorationState::Failed,
                state_ == AutonomousExplorationState::Completed ||
                    state_ == AutonomousExplorationState::Failed,
                report_.termination_reason};
    }

    const AutonomousExplorationReport &report() const { return report_; }
    AutonomousExplorationState state() const { return state_; }

private:
    bool make_plan(SparseSlamRuntimeCore &core, const RobotPose2D &pose) {
        NavigationGridView view;
        const auto built = view.build(core.sparse_map_snapshot(), config_.navigation);
        report_.plan_count++;
        if (!built.ok) {
            report_.planning_failure_count++;
            report_.last_planning_reason = built.reason;
            return false;
        }
        const auto pose_cell = view.world_to_cell(pose.x_m, pose.y_m);
        const int start_projection_radius = static_cast<int>(std::ceil(
            (config_.navigation.robot_radius_m +
             config_.navigation.safety_margin_m) / view.resolution_m())) + 2;
        const auto start = view.nearest_traversable(
            pose_cell, start_projection_radius);
        if (!start) {
            report_.planning_failure_count++;
            report_.last_planning_reason = "no_free_start_cell_near_estimated_pose";
            return false;
        }
        auto cycle_excluded = excluded_frontiers_;
        SparseFrontierPlanResult planned;
        for (std::size_t attempt = 0; attempt < 64; ++attempt) {
            planned = frontier_planner_.plan(view, *start, cycle_excluded);
            report_.expanded_astar_nodes += planned.expanded_astar_nodes;
            if (!planned.ok || planned.no_reachable_frontier ||
                planned.path.waypoints.empty() ||
                !near_failed_goal(planned.path.waypoints.back())) {
                break;
            }
            cycle_excluded.insert(planned.selected_frontier_id);
        }
        report_.frontiers_detected += planned.detected_frontier_cells;
        report_.reachable_frontiers += planned.reachable_cluster_count;
        if (!planned.ok) {
            report_.planning_failure_count++;
            report_.last_planning_reason = planned.reason;
            return false;
        }
        report_.last_planning_reason = planned.reason;
        if (planned.no_reachable_frontier) {
            no_frontier_cycles_++;
            if (no_frontier_cycles_ >= config_.no_frontier_confirmation_cycles) {
                state_ = AutonomousExplorationState::Completed;
            }
            return true;
        }
        no_frontier_cycles_ = 0;
        current_frontier_id_ = planned.selected_frontier_id;
        current_goal_pose_ = planned.path.waypoints.back();
        report_.selected_goals++;
        if (!tracker_.set_path(planned.path.waypoints, pose)) {
            report_.planning_failure_count++;
            return false;
        }
        if (!tracker_.current_waypoint()) {
            report_.completed_goals++;
            excluded_frontiers_.insert(current_frontier_id_);
            state_ = AutonomousExplorationState::Replanning;
            return true;
        }
        state_ = AutonomousExplorationState::AligningToPath;
        return true;
    }

    SafeWaypointTrackerInput tracker_input(
        const RobotSlamSensorSnapshot &snapshot, double now_s,
        const RobotSlamMotionFeedback &feedback,
        const RobotPose2D &pose) const {
        SafeWaypointTrackerInput input;
        input.now_s = now_s;
        input.estimated_map_pose = pose;
        input.motion_feedback = feedback;
        if (snapshot.has_multi_tof && snapshot.multi_tof.has_front) {
            const auto &front = snapshot.multi_tof.front;
            input.front_hit = front.protocol_valid && front.usable_for_mapping &&
                front.source.find("explicit_no_return") == std::string::npos;
            input.front_distance_m = front.distance_m;
        }
        return input;
    }

    bool send_motion(double now_s, AlgorithmMotionKind kind, double speed,
                     double duration, const std::string &reason,
                     RobotSlamMotionPort &motion_port) {
        AlgorithmMotionCommand command;
        command.kind = kind;
        command.profile = AlgorithmMotionProfile::ManualTest;
        command.speed_normalized = speed;
        command.duration_s = duration;
        command.timestamp_s = now_s;
        command.ttl_s = config_.command_ttl_s;
        command.reason = reason;
        command.sequence = ++command_sequence_;
        const auto sent = motion_port.send_algorithm_command(command);
        return sent.ok && sent.accepted;
    }

    bool send_stop(double now_s, RobotSlamMotionPort &motion_port,
                   const std::string &reason) {
        return send_motion(now_s, AlgorithmMotionKind::Stop, 0.0, 0.0,
                           reason, motion_port);
    }

    void record_map_start(const SparseOccupancyGridSnapshot &map) {
        if (!map.valid()) return;
        report_.known_cells_start = map.cell_count();
        report_.occupied_cells_start = map.occupied_cell_count();
        report_.free_cells_start = map.free_cell_count();
        report_.map_revision_start = map.revision();
    }

    void sync_report(const SparseSlamRuntimeCore &core,
                     const RobotPose2D &pose) {
        const auto map = core.sparse_map_snapshot();
        if (map.valid()) {
            report_.known_cells_end = map.cell_count();
            report_.occupied_cells_end = map.occupied_cell_count();
            report_.free_cells_end = map.free_cell_count();
            report_.map_revision_end = map.revision();
        }
        report_.state = state_;
        if (have_last_estimated_pose_) {
            report_.estimated_travel_distance_m +=
                std::hypot(pose.x_m - last_estimated_pose_.x_m,
                           pose.y_m - last_estimated_pose_.y_m);
        }
        last_estimated_pose_ = pose;
        have_last_estimated_pose_ = true;
        report_.matcher_attempts = core.report().matcher_attempt_count;
        report_.atomic_commit_successes = core.report().atomic_commit_success_count;
        report_.keyframe_count = core.keyframe_count();
        report_.final_estimated_map_pose = pose;
    }

    void record_tracker_command(
        const SafeWaypointTrackerStepResult &tracked) {
        if (!tracked.command_sent) return;
        if (tracked.command.kind == AlgorithmMotionKind::Forward) {
            report_.commanded_forward_segments++;
        } else if (tracked.command.kind == AlgorithmMotionKind::TurnLeft ||
                   tracked.command.kind == AlgorithmMotionKind::TurnRight) {
            report_.commanded_turns++;
        }
        if (tracked.replan_required &&
            tracked.reason == "front_obstacle_replan") {
            report_.obstacle_stops++;
        }
    }

    bool near_failed_goal(const RobotPose2D &goal) const {
        const double radius = std::max(
            config_.tracker.obstacle_stop_distance_m * 1.5,
            config_.tracker.waypoint_tolerance_m * 2.0);
        for (const auto &failed : failed_goal_positions_) {
            if (std::hypot(goal.x_m - failed.x_m, goal.y_m - failed.y_m) <=
                radius) {
                return true;
            }
        }
        return false;
    }

    void remember_failed_goal() {
        if (!near_failed_goal(current_goal_pose_) &&
            failed_goal_positions_.size() <
                config_.maximum_planning_failures) {
            failed_goal_positions_.push_back(current_goal_pose_);
        }
    }

    void update_bootstrap_yaw(double yaw) {
        if (have_bootstrap_yaw_) {
            report_.bootstrap_accumulated_yaw_rad +=
                std::fabs(normalize(yaw - last_bootstrap_yaw_));
        }
        last_bootstrap_yaw_ = yaw;
        have_bootstrap_yaw_ = true;
    }

    AutonomousExplorationStepResult fail(const std::string &reason) {
        state_ = AutonomousExplorationState::Failed;
        report_.state = state_;
        report_.termination_reason = reason;
        return {false, true, reason};
    }

    static double normalize(double angle) {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 2.0 * kPi;
        while (angle > kPi) angle -= kTwoPi;
        while (angle < -kPi) angle += kTwoPi;
        return angle;
    }

    AutonomousExplorationConfig config_;
    SparseFrontierPlanner frontier_planner_;
    SafeWaypointTracker tracker_;
    AutonomousExplorationState state_ = AutonomousExplorationState::Initializing;
    AutonomousExplorationReport report_;
    std::set<std::uint64_t> excluded_frontiers_;
    std::uint64_t current_frontier_id_ = 0;
    RobotPose2D current_goal_pose_;
    std::vector<RobotPose2D> failed_goal_positions_;
    std::uint64_t command_sequence_ = 0;
    std::size_t no_frontier_cycles_ = 0;
    bool begin_collection_required_ = false;
    bool end_collection_required_ = false;
    bool discard_required_ = false;
    bool active_collection_ = false;
    bool alignment_replan_required_ = false;
    double start_time_s_ = -1.0;
    double bootstrap_start_s_ = 0.0;
    double last_now_s_ = -1.0;
    double last_bootstrap_yaw_ = 0.0;
    bool have_bootstrap_yaw_ = false;
    RobotPose2D last_estimated_pose_;
    bool have_last_estimated_pose_ = false;
};

} // namespace robot_slamd
