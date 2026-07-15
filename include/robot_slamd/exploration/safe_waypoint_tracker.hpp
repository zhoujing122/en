#pragma once

#include "robot_slamd/autonomy/ports/robot_slam_motion_port.hpp"
#include "robot_slamd/exploration/navigation_grid_view.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_slamd {

enum class SafeWaypointTrackerState {
    Idle,
    Aligning,
    Following,
    GoalReached,
    ReplanRequired,
    Fault
};

struct SafeWaypointTrackerConfig {
    double yaw_tolerance_rad = 0.10;
    double waypoint_tolerance_m = 0.10;
    double obstacle_stop_distance_m = 0.22;
    double emergency_stop_distance_m = 0.10;
    std::size_t obstacle_confirmation_frames = 2;
    double turn_speed_normalized = 0.25;
    double forward_speed_normalized = 0.35;
    double turn_segment_duration_s = 0.25;
    double forward_segment_duration_s = 0.25;
    double command_ttl_s = 0.50;
    double minimum_progress_m = 0.01;
    std::size_t maximum_segments_without_progress = 8;
    double maximum_no_progress_duration_s = 4.0;
};

struct SafeWaypointTrackerInput {
    double now_s = 0.0;
    RobotPose2D estimated_map_pose;
    RobotSlamMotionFeedback motion_feedback;
    bool front_hit = false;
    double front_distance_m = 0.0;
};

struct SafeWaypointTrackerStepResult {
    bool ok = false;
    bool command_sent = false;
    bool waypoint_reached = false;
    bool goal_reached = false;
    bool replan_required = false;
    bool path_validation_required = false;
    AlgorithmMotionCommand command;
    std::string reason;
};

struct SafeWaypointTrackerReport {
    SafeWaypointTrackerState state = SafeWaypointTrackerState::Idle;
    std::size_t waypoint_index = 0;
    std::size_t commanded_turns = 0;
    std::size_t commanded_forward_segments = 0;
    std::size_t commanded_stops = 0;
    std::size_t obstacle_stops = 0;
    std::size_t completed_waypoints = 0;
    std::size_t no_progress_replans = 0;
    double estimated_travel_distance_m = 0.0;
    std::string last_reason = "tracker_idle";
};

class SafeWaypointTracker {
public:
    explicit SafeWaypointTracker(SafeWaypointTrackerConfig config = {})
        : config_(config) {}

    bool set_path(const std::vector<RobotPose2D> &waypoints,
                  const RobotPose2D &estimated_pose) {
        reset();
        if (!valid_config() || waypoints.empty() || !finite_pose(estimated_pose)) {
            report_.state = SafeWaypointTrackerState::Fault;
            report_.last_reason = "tracker_invalid_path";
            return false;
        }
        for (const auto &waypoint : waypoints) {
            if (!finite_pose(waypoint)) {
                report_.state = SafeWaypointTrackerState::Fault;
                report_.last_reason = "tracker_non_finite_waypoint";
                return false;
            }
        }
        waypoints_ = waypoints;
        while (waypoint_index_ < waypoints_.size() &&
               distance(estimated_pose, waypoints_[waypoint_index_]) <=
                   config_.waypoint_tolerance_m) {
            waypoint_index_++;
        }
        report_.waypoint_index = waypoint_index_;
        last_pose_ = estimated_pose;
        have_last_pose_ = true;
        best_waypoint_distance_m_ = waypoint_index_ < waypoints_.size()
            ? distance(estimated_pose, waypoints_[waypoint_index_]) : 0.0;
        last_progress_time_s_ = -1.0;
        report_.state = waypoint_index_ == waypoints_.size()
            ? SafeWaypointTrackerState::GoalReached
            : SafeWaypointTrackerState::Aligning;
        report_.last_reason = "tracker_path_ready";
        need_transition_stop_ = true;
        return true;
    }

    SafeWaypointTrackerStepResult step(
        const SafeWaypointTrackerInput &input,
        RobotSlamMotionPort &motion_port) {
        SafeWaypointTrackerStepResult result;
        if (!valid_input(input) || !motion_port.ready()) {
            report_.state = SafeWaypointTrackerState::Fault;
            report_.last_reason = "tracker_invalid_input_or_motion_port";
            result.reason = report_.last_reason;
            return result;
        }
        update_estimated_distance(input.estimated_map_pose);
        if (input.motion_feedback.fault || input.motion_feedback.emergency_stop_active) {
            report_.state = SafeWaypointTrackerState::Fault;
            report_.last_reason = "tracker_motion_fault";
            result.reason = report_.last_reason;
            return result;
        }
        if (report_.state == SafeWaypointTrackerState::GoalReached) {
            result.ok = true;
            result.goal_reached = true;
            result.reason = "tracker_goal_reached";
            return result;
        }
        if (report_.state == SafeWaypointTrackerState::ReplanRequired ||
            report_.state == SafeWaypointTrackerState::Fault ||
            waypoint_index_ >= waypoints_.size()) {
            result.replan_required =
                report_.state == SafeWaypointTrackerState::ReplanRequired;
            result.reason = report_.last_reason;
            return result;
        }
        if (input.motion_feedback.command_active &&
            last_motion_kind_ == AlgorithmMotionKind::Forward &&
            input.front_hit &&
            input.front_distance_m <= config_.obstacle_stop_distance_m) {
            obstacle_confirmation_count_++;
            const bool emergency = input.front_distance_m <=
                config_.emergency_stop_distance_m;
            const bool confirmed = obstacle_confirmation_count_ >=
                config_.obstacle_confirmation_frames;
            if (emergency || confirmed) {
                obstacle_hold_ = true;
                const bool stopped = send_stop(
                    input.now_s, motion_port,
                    emergency ? "exploration_emergency_obstacle_stop"
                              : "exploration_confirmed_obstacle_stop");
                report_.obstacle_stops++;
                report_.last_reason = emergency
                    ? "front_emergency_obstacle_path_validation"
                    : "front_confirmed_obstacle_path_validation";
                result.ok = stopped;
                result.command_sent = stopped;
                result.command = last_command_;
                result.path_validation_required = true;
                result.replan_required = emergency;
                result.reason = report_.last_reason;
                return result;
            }
        } else if (!input.front_hit ||
                   input.front_distance_m > config_.obstacle_stop_distance_m) {
            obstacle_confirmation_count_ = 0;
            obstacle_hold_ = false;
        }
        if (input.motion_feedback.command_active ||
            !input.motion_feedback.command_settled) {
            result.ok = true;
            result.reason = "tracker_waiting_for_motion_settle";
            return result;
        }
        if (last_motion_pending_) {
            last_motion_pending_ = false;
            if (last_motion_kind_ == AlgorithmMotionKind::Forward) {
                const double current = distance(
                    input.estimated_map_pose, waypoints_[waypoint_index_]);
                if (best_waypoint_distance_m_ - current >=
                    config_.minimum_progress_m) {
                    best_waypoint_distance_m_ = current;
                    segments_without_progress_ = 0;
                    last_progress_time_s_ = input.now_s;
                } else {
                    segments_without_progress_++;
                    if (last_progress_time_s_ < 0.0) {
                        last_progress_time_s_ = input.now_s;
                    }
                    if (segments_without_progress_ >=
                            config_.maximum_segments_without_progress ||
                        input.now_s - last_progress_time_s_ >=
                            config_.maximum_no_progress_duration_s) {
                        send_stop(input.now_s, motion_port,
                                  "exploration_no_progress_stop");
                        report_.no_progress_replans++;
                        report_.state = SafeWaypointTrackerState::ReplanRequired;
                        report_.last_reason = "tracker_no_progress_replan";
                        result.ok = true;
                        result.command_sent = true;
                        result.command = last_command_;
                        result.replan_required = true;
                        result.reason = report_.last_reason;
                        return result;
                    }
                }
            }
        }

        if (distance(input.estimated_map_pose, waypoints_[waypoint_index_]) <=
            config_.waypoint_tolerance_m) {
            waypoint_index_++;
            report_.waypoint_index = waypoint_index_;
            report_.completed_waypoints++;
            result.waypoint_reached = true;
            segments_without_progress_ = 0;
            if (waypoint_index_ >= waypoints_.size()) {
                const bool stopped = send_stop(input.now_s, motion_port,
                                               "exploration_goal_stop");
                report_.state = SafeWaypointTrackerState::GoalReached;
                report_.last_reason = "tracker_goal_reached";
                result.ok = stopped;
                result.command_sent = stopped;
                result.command = last_command_;
                result.goal_reached = true;
                result.reason = report_.last_reason;
                return result;
            }
            best_waypoint_distance_m_ = distance(
                input.estimated_map_pose, waypoints_[waypoint_index_]);
            need_transition_stop_ = true;
        }

        const auto &target = waypoints_[waypoint_index_];
        const double target_yaw = std::atan2(
            target.y_m - input.estimated_map_pose.y_m,
            target.x_m - input.estimated_map_pose.x_m);
        const double yaw_error = shortest_angle(
            target_yaw - input.estimated_map_pose.yaw_rad);
        const auto desired_kind = std::fabs(yaw_error) > config_.yaw_tolerance_rad
            ? (yaw_error > 0.0 ? AlgorithmMotionKind::TurnLeft
                               : AlgorithmMotionKind::TurnRight)
            : AlgorithmMotionKind::Forward;
        const bool forward_intended = desired_kind == AlgorithmMotionKind::Forward;
        if (!input.front_hit || !forward_intended ||
            input.front_distance_m > config_.obstacle_stop_distance_m) {
            obstacle_confirmation_count_ = 0;
            obstacle_hold_ = false;
        } else {
            obstacle_confirmation_count_++;
            const bool emergency = input.front_distance_m <=
                config_.emergency_stop_distance_m;
            const bool confirmed = obstacle_confirmation_count_ >=
                config_.obstacle_confirmation_frames;
            if (emergency || confirmed || obstacle_hold_) {
                obstacle_hold_ = true;
                const auto stopped = send_stop(
                    input.now_s, motion_port,
                    emergency ? "exploration_emergency_obstacle_stop"
                              : "exploration_confirmed_obstacle_stop");
                if (emergency || confirmed) report_.obstacle_stops++;
                report_.last_reason = emergency
                    ? "front_emergency_obstacle_path_validation"
                    : "front_confirmed_obstacle_path_validation";
                result.ok = stopped;
                result.command_sent = stopped;
                result.command = last_command_;
                result.path_validation_required = true;
                result.replan_required = emergency;
                result.reason = report_.last_reason;
                return result;
            }
        }
        if (need_transition_stop_ ||
            (last_motion_kind_ != AlgorithmMotionKind::Stop &&
             motion_group(last_motion_kind_) != motion_group(desired_kind))) {
            const bool stopped = send_stop(input.now_s, motion_port,
                                           "exploration_transition_stop");
            need_transition_stop_ = false;
            result.ok = stopped;
            result.command_sent = stopped;
            result.command = last_command_;
            result.reason = "tracker_transition_stop";
            return result;
        }

        AlgorithmMotionCommand command;
        command.kind = desired_kind;
        command.profile = AlgorithmMotionProfile::ManualTest;
        command.speed_normalized = desired_kind == AlgorithmMotionKind::Forward
            ? config_.forward_speed_normalized : config_.turn_speed_normalized;
        command.duration_s = desired_kind == AlgorithmMotionKind::Forward
            ? config_.forward_segment_duration_s : config_.turn_segment_duration_s;
        command.timestamp_s = input.now_s;
        command.ttl_s = config_.command_ttl_s;
        command.reason = desired_kind == AlgorithmMotionKind::Forward
            ? "exploration_follow_forward" : "exploration_align_turn";
        command.sequence = ++next_sequence_;
        const auto sent = motion_port.send_algorithm_command(command);
        if (!sent.ok || !sent.accepted) {
            report_.state = SafeWaypointTrackerState::Fault;
            report_.last_reason = "tracker_motion_command_rejected:" + sent.error;
            result.reason = report_.last_reason;
            return result;
        }
        last_command_ = command;
        last_motion_kind_ = desired_kind;
        last_motion_pending_ = true;
        if (desired_kind == AlgorithmMotionKind::Forward) {
            report_.state = SafeWaypointTrackerState::Following;
            report_.commanded_forward_segments++;
        } else {
            report_.state = SafeWaypointTrackerState::Aligning;
            report_.commanded_turns++;
        }
        report_.last_reason = command.reason;
        result.ok = true;
        result.command_sent = true;
        result.command = command;
        result.reason = report_.last_reason;
        return result;
    }

    void reset() {
        waypoints_.clear();
        waypoint_index_ = 0;
        next_sequence_ = 0;
        last_motion_kind_ = AlgorithmMotionKind::Stop;
        last_motion_pending_ = false;
        need_transition_stop_ = false;
        segments_without_progress_ = 0;
        best_waypoint_distance_m_ = 0.0;
        last_progress_time_s_ = -1.0;
        obstacle_confirmation_count_ = 0;
        obstacle_hold_ = false;
        have_last_pose_ = false;
        report_ = {};
    }

    const SafeWaypointTrackerReport &report() const { return report_; }
    const RobotPose2D *current_waypoint() const {
        return waypoint_index_ < waypoints_.size()
            ? &waypoints_[waypoint_index_] : nullptr;
    }

    bool remaining_path_is_traversable(const NavigationGridView &grid,
                                       const RobotPose2D &estimated_pose) const {
        if (!grid.valid() || !finite_pose(estimated_pose) ||
            waypoint_index_ >= waypoints_.size()) return false;
        const auto start = grid.nearest_traversable(
            grid.world_to_cell(estimated_pose.x_m, estimated_pose.y_m), 4);
        if (!start) return false;
        RobotPose2D from = grid.cell_center(*start);
        for (std::size_t i = waypoint_index_; i < waypoints_.size(); ++i) {
            const auto &to = waypoints_[i];
            const double length = distance(from, to);
            const std::size_t samples = std::max<std::size_t>(
                1, static_cast<std::size_t>(std::ceil(
                    length / std::max(grid.resolution_m() * 0.5, 1e-6))));
            for (std::size_t sample = 1; sample <= samples; ++sample) {
                const double t = static_cast<double>(sample) / samples;
                if (!grid.traversable(grid.world_to_cell(
                        from.x_m + t * (to.x_m - from.x_m),
                        from.y_m + t * (to.y_m - from.y_m)))) {
                    return false;
                }
            }
            from = to;
        }
        return true;
    }

private:
    static bool finite_pose(const RobotPose2D &pose) {
        return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
               std::isfinite(pose.yaw_rad);
    }

    bool valid_config() const {
        return std::isfinite(config_.yaw_tolerance_rad) &&
               config_.yaw_tolerance_rad > 0.0 &&
               std::isfinite(config_.waypoint_tolerance_m) &&
               config_.waypoint_tolerance_m > 0.0 &&
               std::isfinite(config_.obstacle_stop_distance_m) &&
               config_.obstacle_stop_distance_m > 0.0 &&
               std::isfinite(config_.emergency_stop_distance_m) &&
               config_.emergency_stop_distance_m > 0.0 &&
               config_.emergency_stop_distance_m <
                   config_.obstacle_stop_distance_m &&
               config_.obstacle_confirmation_frames > 0 &&
               config_.turn_speed_normalized > 0.0 &&
               config_.turn_speed_normalized <= 1.0 &&
               config_.forward_speed_normalized > 0.0 &&
               config_.forward_speed_normalized <= 1.0 &&
               config_.turn_segment_duration_s > 0.0 &&
               config_.forward_segment_duration_s > 0.0 &&
               config_.command_ttl_s > 0.0 &&
               config_.minimum_progress_m >= 0.0 &&
               config_.maximum_segments_without_progress > 0 &&
               std::isfinite(config_.maximum_no_progress_duration_s) &&
               config_.maximum_no_progress_duration_s > 0.0;
    }

    bool valid_input(const SafeWaypointTrackerInput &input) const {
        return valid_config() && std::isfinite(input.now_s) &&
               finite_pose(input.estimated_map_pose) &&
               (!input.front_hit || (std::isfinite(input.front_distance_m) &&
                                      input.front_distance_m >= 0.0));
    }

    static double distance(const RobotPose2D &a, const RobotPose2D &b) {
        return std::hypot(a.x_m - b.x_m, a.y_m - b.y_m);
    }

    static double shortest_angle(double angle) {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 2.0 * kPi;
        while (angle > kPi) angle -= kTwoPi;
        while (angle < -kPi) angle += kTwoPi;
        return angle;
    }

    static int motion_group(AlgorithmMotionKind kind) {
        if (kind == AlgorithmMotionKind::Forward) return 1;
        if (kind == AlgorithmMotionKind::TurnLeft ||
            kind == AlgorithmMotionKind::TurnRight) return 2;
        return 0;
    }

    bool send_stop(double now_s, RobotSlamMotionPort &motion_port,
                   const std::string &reason) {
        AlgorithmMotionCommand command;
        command.kind = AlgorithmMotionKind::Stop;
        command.profile = AlgorithmMotionProfile::Safety;
        command.timestamp_s = now_s;
        command.ttl_s = config_.command_ttl_s;
        command.reason = reason;
        command.sequence = ++next_sequence_;
        const auto sent = motion_port.send_algorithm_command(command);
        if (!sent.ok || !sent.accepted) return false;
        last_command_ = command;
        last_motion_kind_ = AlgorithmMotionKind::Stop;
        last_motion_pending_ = false;
        report_.commanded_stops++;
        return true;
    }

    void update_estimated_distance(const RobotPose2D &pose) {
        if (have_last_pose_) report_.estimated_travel_distance_m += distance(last_pose_, pose);
        last_pose_ = pose;
        have_last_pose_ = true;
    }

    SafeWaypointTrackerConfig config_;
    std::vector<RobotPose2D> waypoints_;
    std::size_t waypoint_index_ = 0;
    std::uint64_t next_sequence_ = 0;
    AlgorithmMotionKind last_motion_kind_ = AlgorithmMotionKind::Stop;
    bool last_motion_pending_ = false;
    bool need_transition_stop_ = false;
    std::size_t segments_without_progress_ = 0;
    double best_waypoint_distance_m_ = 0.0;
    double last_progress_time_s_ = -1.0;
    std::size_t obstacle_confirmation_count_ = 0;
    bool obstacle_hold_ = false;
    RobotPose2D last_pose_;
    bool have_last_pose_ = false;
    AlgorithmMotionCommand last_command_;
    SafeWaypointTrackerReport report_;
};

} // namespace robot_slamd
