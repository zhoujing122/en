#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_single_corner_test_helpers.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

using robot_slamd::StopGoMappingRunReport;

static void print_report(const char *name, const StopGoMappingRunReport &r) {
    std::cout << name
              << " ok=" << r.ok
              << " before=" << r.completed_forward_steps_before_corner
              << " after=" << r.completed_forward_steps_after_corner
              << " candidates=" << r.corner_candidate_count
              << " confirmations=" << r.corner_confirmation_count
              << " rejects=" << r.corner_confirmation_reject_count
              << " clearance_checks=" << r.corner_right_clearance_check_count
              << " clearance_rejects=" << r.corner_right_clearance_reject_count
              << " main=" << r.corner_main_turn_command_count
              << " right=" << r.corner_right_turn_command_count
              << " left=" << r.corner_left_turn_command_count
              << " main_id=" << r.corner_main_command_id
              << " pre_yaw_deg=" << r.pre_turn_odom_yaw_rad * 180.0 / M_PI
              << " post_yaw_deg=" << r.post_main_turn_odom_yaw_rad * 180.0 / M_PI
              << " delta_deg=" << r.verified_corner_turn_delta_rad * 180.0 / M_PI
              << " error_deg=" << r.final_corner_turn_error_rad * 180.0 / M_PI
              << " residual=" << r.corner_residual_correction_count
              << " residual_right=" << r.corner_residual_right_count
              << " residual_left=" << r.corner_residual_left_count
              << " sensor_verify=" << r.post_turn_sensor_verify_count
              << " old_segment=" << r.old_wall_segment_id
              << " new_segment=" << r.new_wall_segment_id
              << " resets=" << r.wall_model_reset_due_to_corner_count
              << " reacq_samples=" << r.new_wall_reacquisition_samples
              << " reacq_steps=" << r.new_wall_acquisition_steps
              << " new_model=" << r.new_wall_model_valid
              << " follow=" << r.post_corner_follow_steps
              << " map_before=" << r.map_revision_before_corner
              << " map_after=" << r.map_revision_after_corner
              << " writes_confirm=" << r.map_writes_during_corner_confirm
              << " writes_turn=" << r.map_writes_during_corner_turn
              << " writes_verify=" << r.map_writes_during_turn_verification
              << " collision=" << r.collision_count
              << " commands=" << r.commands_submitted
              << " target_odometry=" << r.command_target_used_as_odometry
              << " gt_algorithm=" << r.ground_truth_used_by_algorithm
              << " epoch=" << r.frame_transform_epoch
              << " checksum=" << r.final_map_checksum
              << " final_pose=" << r.final_estimated_pose.x_m << ","
              << r.final_estimated_pose.y_m << "," << r.final_estimated_pose.yaw_rad
              << " wall_point_hash=" << r.wall_point_sequence_hash
              << " wall_model_hash=" << r.wall_model_sequence_hash
              << " corner_event_hash=" << r.corner_event_sequence_hash
              << " corner_command_hash=" << r.corner_command_sequence_hash
              << " termination=" << r.termination_reason << '\n';
}

int main() {
    using namespace robot_slamd;
    const std::filesystem::path root = "/tmp/p5_single_corner_formal";
    std::filesystem::create_directories(root);
    const auto run = [&](const char *scenario) {
        std::filesystem::create_directories(root / scenario);
        return StopGoStraightWallSimulationRunner{}.run_scenario(
            p5_single_corner_test_config(), (root / scenario).string(), scenario);
    };
    const auto nominal = run("nominal_single_corner");
    const auto under = run("under_rotation");
    const auto over = run("over_rotation");
    const auto clearance = run("right_turn_clearance_blocked");
    const auto false_front = run("false_front_measurement");
    const auto missing = run("new_left_wall_missing");
    const auto failed_turn = run("corner_turn_verification_failed");
    print_report("nominal_single_corner", nominal);
    print_report("under_rotation", under);
    print_report("over_rotation", over);
    print_report("right_turn_clearance_blocked", clearance);
    print_report("false_front_measurement", false_front);
    print_report("new_left_wall_missing", missing);
    print_report("corner_turn_verification_failed", failed_turn);
    const auto success = [](const StopGoMappingRunReport &r) {
        return r.ok && r.termination_reason == "single_corner_complete" &&
               r.corner_main_turn_command_count == 1 &&
               r.corner_right_turn_command_count >= 1 &&
               r.old_wall_segment_id == 1 && r.new_wall_segment_id == 2 &&
               r.wall_model_reset_due_to_corner_count == 1 &&
               r.new_wall_model_valid && r.post_corner_follow_steps >= 5 &&
               std::fabs(r.final_corner_turn_error_rad) <= 3.0 * M_PI / 180.0 &&
               r.collision_count == 0 && r.map_writes_during_corner_confirm == 0 &&
               r.map_writes_during_corner_turn == 0 &&
               r.map_writes_during_turn_verification == 0 &&
               !r.command_target_used_as_odometry && !r.ground_truth_used_by_algorithm;
    };
    const auto deterministic = [](const StopGoMappingRunReport &a,
                                  const StopGoMappingRunReport &b) {
        return a.ok == b.ok && a.command_ids == b.command_ids &&
               a.wall_point_sequence_hash == b.wall_point_sequence_hash &&
               a.wall_model_sequence_hash == b.wall_model_sequence_hash &&
               a.corner_event_sequence_hash == b.corner_event_sequence_hash &&
               a.corner_command_sequence_hash == b.corner_command_sequence_hash &&
               a.final_estimated_pose.x_m == b.final_estimated_pose.x_m &&
               a.final_estimated_pose.y_m == b.final_estimated_pose.y_m &&
               a.final_estimated_pose.yaw_rad == b.final_estimated_pose.yaw_rad &&
               a.map_revision == b.map_revision && a.final_map_checksum == b.final_map_checksum &&
               a.termination_reason == b.termination_reason;
    };
    const auto nominal_repeat = run("nominal_single_corner_repeat");
    return success(nominal) && success(under) && success(over) &&
           clearance.termination_reason == "turn_clearance_blocked" &&
           clearance.corner_main_turn_command_count == 0 &&
           false_front.corner_confirmation_reject_count > 0 && success(false_front) &&
           missing.termination_reason == "new_left_wall_reacquisition_failed" &&
           failed_turn.termination_reason == "corner_turn_verification_failed" &&
           failed_turn.corner_main_turn_command_count == 1 && !failed_turn.new_wall_model_valid &&
           deterministic(nominal, nominal_repeat) ? 0 : 1;
}
