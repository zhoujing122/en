#include "robot_slamd/replay/stop_go_replay_runner.hpp"
#include "robot_slamd/simulation/application/stop_go_straight_wall_runner.hpp"
#include "stop_go_rectangle_test_helpers.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

using robot_slamd::StopGoMappingRunReport;

static void print_summary(const std::string &name, const StopGoMappingRunReport &r) {
    std::cout << name << " ok=" << r.ok
              << " termination=" << r.termination_reason
              << " transitions=" << r.corner_transition_count
              << " segments=";
    for (std::size_t i = 0; i < r.wall_segment_sequence.size(); ++i) {
        if (i != 0) std::cout << ',';
        std::cout << r.wall_segment_sequence[i];
    }
    std::cout << " main_right=" << r.corner_right_turn_command_count
              << " residuals=" << r.corner_residual_correction_count
              << " residual_right=" << r.corner_residual_right_count
              << " residual_left=" << r.corner_residual_left_count
              << " epoch_recovery=" << r.frame_epoch_recovery_count
              << " closure_pos=" << r.estimated_closure_position_error_m
              << " closure_yaw_deg="
              << r.estimated_closure_yaw_error_rad * 180.0 / robot_slamd::kPi
              << " gt_pos=" << r.ground_truth_final_position_error_m
              << " gt_final=" << r.final_estimated_pose.x_m << "," << r.final_estimated_pose.y_m
              << " gt_yaw_deg=" << r.ground_truth_final_yaw_error_rad * 180.0 / robot_slamd::kPi
              << " coverage=" << r.observable_wall_coverage_ratio
              << " p95=" << r.p95_wall_thickness_cells
              << " ghost=" << r.ghost_occupied_cell_ratio
              << " duplicate=" << r.duplicate_wall_band_count
              << " confirm=" << r.closure_confirmation_pass_count
              << " wall_hash=" << r.wall_point_sequence_hash
              << " model_hash=" << r.wall_model_sequence_hash
              << " map_checksum=" << r.final_map_checksum << '\n';
}

int main() {
    using namespace robot_slamd;
    const std::filesystem::path root = "/tmp/p6_rectangle_formal_matrix";
    std::filesystem::create_directories(root);
    const std::vector<std::string> scenarios = {
        "nominal_rectangle", "mixed_turn_residuals", "false_corner_on_middle_segment",
        "corner_clearance_failure", "new_wall_loss_mid_run", "closure_not_reached",
        "premature_start_proximity", "frame_epoch_change_mid_run"};
    std::vector<StopGoMappingRunReport> reports;
    for (const auto &scenario : scenarios) {
        const auto report = StopGoStraightWallSimulationRunner{}.run_scenario(
            p6_rectangle_test_config(), (root / scenario).string(), scenario);
        print_summary(scenario, report);
        reports.push_back(report);
    }
    const auto &nominal = reports[0];
    const auto &mixed = reports[1];
    const auto &false_corner = reports[2];
    const auto &clearance = reports[3];
    const auto &missing = reports[4];
    const auto &not_reached = reports[5];
    const auto &premature = reports[6];
    const auto &epoch_change = reports[7];
    const auto replay_one = StopGoReplayRunner{}.run(
        p6_rectangle_test_config(),
        (root / "nominal_rectangle" / "stop_go_mapping.replay").string(),
        (root / "replay_one").string());
    const auto replay_two = StopGoReplayRunner{}.run(
        p6_rectangle_test_config(),
        (root / "nominal_rectangle" / "stop_go_mapping.replay").string(),
        (root / "replay_two").string());
    print_summary("replay_one", replay_one);
    print_summary("replay_two", replay_two);
    const bool pass = nominal.ok && nominal.termination_reason == "rectangle_closure_confirmed" &&
           nominal.corner_transition_count == 4 &&
           nominal.wall_segment_sequence == std::vector<std::uint64_t>{1, 2, 3, 4, 5} &&
           nominal.corner_main_turn_command_count == 4 &&
           nominal.final_map_saved && nominal.final_map_reload_verified &&
           nominal.map_writes_while_moving == 0 &&
           nominal.map_writes_during_corner_confirm == 0 &&
           nominal.map_writes_during_corner_turn == 0 &&
           nominal.map_writes_during_turn_verification == 0 &&
           nominal.map_writes_during_closure_confirm == 0 &&
           nominal.collision_count == 0 &&
           nominal.ground_truth_final_position_error_m <= 0.10 &&
           std::fabs(nominal.ground_truth_final_yaw_error_rad) <= 5.0 * kPi / 180.0 &&
           nominal.first_closure_candidate_transition_count == 4 &&
           mixed.ok && mixed.corner_residual_correction_count >= 2 &&
           mixed.corner_residual_right_count >= 1 && mixed.corner_residual_left_count >= 1 &&
           false_corner.ok && false_corner.corner_confirmation_reject_count >= 1 &&
           false_corner.corner_transition_count == 4 &&
           clearance.termination_reason == "turn_clearance_blocked" &&
           clearance.corner_transition_count < 4 &&
           missing.termination_reason == "new_left_wall_reacquisition_failed" &&
           missing.corner_transition_count < 4 &&
           not_reached.termination_reason == "rectangle_closure_not_reached" &&
           not_reached.corner_transition_count == 4 &&
           premature.ok && premature.closure_candidate_count == 1 &&
           premature.first_closure_candidate_transition_count == 4 &&
           epoch_change.ok && epoch_change.frame_epoch_change_injected &&
           epoch_change.injected_frame_epoch_after > epoch_change.injected_frame_epoch_before &&
           epoch_change.frame_epoch_recovery_count == 1 &&
           epoch_change.run_start_anchor.valid &&
           replay_one.ok && replay_two.ok &&
           replay_one.termination_reason == "rectangle_closure_confirmed" &&
           replay_one.final_map_checksum == nominal.final_map_checksum &&
           replay_one.final_map_checksum == replay_two.final_map_checksum &&
           replay_one.wall_segment_sequence == replay_two.wall_segment_sequence;
    return pass ? 0 : 1;
}
