#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_left_wall_test_helpers.hpp"

#include <filesystem>
#include <cmath>
#include <iostream>

static void print_report(const char *name,
                         const robot_slamd::StopGoMappingRunReport &r) {
    std::cout << name
              << " ok=" << r.ok
              << " completed_forward_steps=" << r.completed_steps
              << " commands_submitted=" << r.commands_submitted
              << " forward_command_count=" << r.forward_command_count
              << " correction_command_count=" << r.correction_command_count
              << " left_correction_count=" << r.left_correction_count
              << " right_correction_count=" << r.right_correction_count
              << " wall_acquisition_steps=" << r.wall_acquisition_steps
              << " wall_model_valid_count=" << r.wall_model_valid_count
              << " wall_fit_baseline_m=" << r.wall_fit_baseline_m
              << " wall_fit_rms_m=" << r.wall_fit_rms_m
              << " initial_heading_error_deg=" << r.initial_heading_error_deg
              << " final_heading_error_deg=" << r.final_heading_error_deg
              << " initial_distance_error_mm=" << r.initial_distance_error_mm
              << " final_distance_error_mm=" << r.final_distance_error_mm
              << " final_pose=" << r.final_estimated_pose.x_m << ","
              << r.final_estimated_pose.y_m << ","
              << r.final_estimated_pose.yaw_rad
              << " map_revision=" << r.map_revision
              << " map_cells=" << r.map_cells
              << " map_writes_while_moving=" << r.map_writes_while_moving
              << " map_writes_during_turn_or_verify="
              << r.map_writes_during_turn_or_verify
              << " collision_count=" << r.collision_count
              << " final_map_checksum=" << r.final_map_checksum
              << " epoch=" << r.frame_transform_epoch
              << " wall_point_hash=" << r.wall_point_sequence_hash
              << " wall_model_hash=" << r.wall_model_sequence_hash
              << " termination_reason=" << r.termination_reason << "\n";
}

int main() {
    using namespace robot_slamd;
    const std::filesystem::path root = "/tmp/p4_left_wall_formal";
    std::filesystem::create_directories(root);
    const auto run = [&](const char *scenario) {
        std::filesystem::create_directories(root / scenario);
        return StopGoStraightWallSimulationRunner{}.run_scenario(
            p4_test_config(), (root / scenario).string(), scenario);
    };
    const auto nominal = run("nominal");
    const auto nominal_repeat = run("nominal_repeat");
    const auto toward = run("heading_toward_wall");
    const auto away = run("heading_away_from_wall");
    const auto far = run("too_far_from_wall");
    const auto close = run("too_close_to_wall");
    const auto lost = run("left_wall_loss");
    const auto blocked = run("front_blocked");
    print_report("nominal", nominal);
    print_report("nominal_repeat", nominal_repeat);
    print_report("heading_toward_wall", toward);
    print_report("heading_away_from_wall", away);
    print_report("too_far_from_wall", far);
    print_report("too_close_to_wall", close);
    print_report("left_wall_loss", lost);
    print_report("front_blocked", blocked);

    const auto normal_ok = [](const StopGoMappingRunReport &r) {
        return r.ok && r.completed_steps > 0 && r.collision_count == 0 &&
               r.map_writes_while_moving == 0 &&
               r.map_writes_during_turn_or_verify == 0 &&
               r.wall_model_valid_count > 0 &&
               std::fabs(r.final_heading_error_deg) <= 2.0 &&
               std::fabs(r.final_distance_error_mm) <= 20.0 &&
               !r.command_target_used_as_odometry &&
               !r.ground_truth_used_by_algorithm;
    };
    const bool signed_actions_ok = toward.right_correction_count > 0 &&
        away.left_correction_count > 0 && far.left_correction_count > 0 &&
        close.right_correction_count > 0;
    return normal_ok(nominal) && normal_ok(toward) && normal_ok(away) &&
           normal_ok(far) && normal_ok(close) && signed_actions_ok &&
           nominal_repeat.ok &&
           nominal.command_ids == nominal_repeat.command_ids &&
           nominal.wall_point_sequence_hash == nominal_repeat.wall_point_sequence_hash &&
           nominal.wall_model_sequence_hash == nominal_repeat.wall_model_sequence_hash &&
           nominal.final_map_checksum == nominal_repeat.final_map_checksum &&
           nominal.final_estimated_pose.x_m == nominal_repeat.final_estimated_pose.x_m &&
           nominal.final_estimated_pose.y_m == nominal_repeat.final_estimated_pose.y_m &&
           nominal.final_estimated_pose.yaw_rad == nominal_repeat.final_estimated_pose.yaw_rad &&
           nominal.map_revision == nominal_repeat.map_revision &&
           !lost.ok && lost.termination_reason.find("left_wall") != std::string::npos &&
           !blocked.ok && blocked.termination_reason.find("front_") != std::string::npos
               ? 0 : 1;
}
