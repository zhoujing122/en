#pragma once
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <dlfcn.h>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace robot_slamd {

constexpr double kPi = 3.14159265358979323846;
inline volatile std::sig_atomic_t g_stop = 0;

std::string trim(const std::string &s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string strip_quotes(const std::string &s) {
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                          (s.front() == '\'' && s.back() == '\''))) {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

bool mkdir_p(const std::string &path) {
    if (path.empty()) return false;
    std::string cur;
    for (char c : path) {
        cur.push_back(c);
        if (c == '/' && cur.size() > 1) {
            if (mkdir(cur.c_str(), 0755) && errno != EEXIST) return false;
        }
    }
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

std::string absolute_path(const std::string &path) {
    if (path.empty() || path.front() == '/') return path;
    char cwd[4096];
    if (!getcwd(cwd, sizeof(cwd))) return path;
    return std::string(cwd) + "/" + path;
}

bool file_exists(const std::string &path) {
    struct stat st {};
    return stat(path.c_str(), &st) == 0;
}

uint64_t now_us_steady() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

std::string timestamp_dir_name() {
    std::time_t t = std::time(nullptr);
    std::tm tm {};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm);
    return buf;
}

bool create_unique_run_dir(const std::string &run_root, std::string &run_dir, std::string &err) {
    std::string base = timestamp_dir_name();
    for (int i = 0; i < 1000; ++i) {
        std::ostringstream name;
        name << run_root << "/" << base;
        if (i > 0) name << "-" << std::setw(3) << std::setfill('0') << i;
        std::string candidate = name.str();
        if (mkdir(candidate.c_str(), 0755) == 0) {
            run_dir = candidate;
            return true;
        }
        if (errno != EEXIST) {
            err = std::strerror(errno);
            return false;
        }
    }
    err = "too many run_dir collisions for timestamp " + base;
    return false;
}

double deg2rad(double d) { return d * kPi / 180.0; }

double wrap_pi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

struct Config {
    double wheel_base_m = 0.180;
    double wheel_radius_left_m = 0.032;
    double wheel_radius_right_m = 0.032;
    int encoder_ticks_per_rev = 1024;
    std::string encoder_source = "sim";
    std::string encoder_path = "/userdata/encoder.csv";
    std::string encoder_left_port;
    std::string encoder_right_port;
    int encoder_baudrate = 1000000;
    int encoder_uart_timeout_ms = 10;
    int encoder_left_id = 1;
    int encoder_right_id = 1;
    int encoder_left_sign = 1;
    int encoder_right_sign = 1;
    double encoder_read_hz = 100.0;
    bool encoder_position_unwrap = true;
    bool encoder_speed_diagnostic = true;
    int encoder_consecutive_error_limit = 5;
    std::string imu_source = "sim";
    std::string imu_path = "/userdata/imu.csv";
    std::string imu_mode = "localization";
    std::string imu_gyro_axis = "z";
    int imu_gyro_sign = 1;
    std::string imu_icm43600_lib_path = "/userdata/libicm43600.so";
    uint64_t imu_icm43600_device_no = 0;
    uint64_t imu_icm43600_accel_rate_hz = 100;
    uint64_t imu_icm43600_gyro_rate_hz = 100;
    uint64_t imu_icm43600_batch_timeout_ms = 0;
    uint32_t imu_icm43600_accel_fsr = 2;
    uint32_t imu_icm43600_gyro_fsr = 3;
    bool imu_icm43600_high_res_fifo = false;
    double gyro_bias_static_calib_s = 0.5;
    double static_gyro_max_abs_rad_s = 0.03;
    double yaw_fusion_alpha = 0.95;
    double encoder_max_wheel_speed_mps = 1.0;
    double gyro_lpf_alpha = 0.30;
    double gyro_spike_radps = 4.0;
    double stationary_linear_speed_mps = 0.01;
    double stationary_angular_speed_radps = 0.02;
    double gyro_bias_adapt_alpha = 0.001;
    std::string tof_source = "sim";
    std::string tof_csv_path = "/userdata/tof.csv";
    double tof_frequency_hz = 30.0;
    double tof_min_range_m = 0.05;
    double tof_max_range_m = 3.0;
    int confidence_min = 70;
    int median_window = 5;
    double jump_reject_m = 0.6;
    double mad_reject_m = 0.20;
    int stable_hits_required = 2;
    std::map<std::string, std::string> tof_devices{
        {"front", "/dev/mt3801_front"},
        {"left", "/dev/mt3801_left"},
        {"right", "/dev/mt3801_right"},
    };
    struct TofExtrinsic {
        double x_m = 0.0;
        double y_m = 0.0;
        double yaw_deg = 0.0;
        double fov_deg = 10.0;
    };
    std::map<std::string, TofExtrinsic> tof_extrinsics{
        {"front", {0.10, 0.00, 0.0, 10.0}},
        {"left", {0.00, 0.08, 90.0, 10.0}},
        {"right", {0.00, -0.08, -90.0, 10.0}},
    };
    double resolution_m = 0.05;
    int chunk_cells = 64;
    double initial_width_m = 10.0;
    double initial_height_m = 10.0;
    double ray_step_deg = 2.0;
    double hit_thickness_m = 0.08;
    double log_odds_occ = 0.85;
    double log_odds_free = -0.35;
    double log_odds_min = -3.5;
    double log_odds_max = 3.5;
    double occupied_thresh = 0.65;
    double free_thresh = 0.25;
    int max_cells_per_tof_update = 4096;
    int static_scan_stable_required = 3;
    double static_scan_boost = 1.5;
    double localization_hz = 50.0;
    double tof_read_hz = 30.0;
    double mapping_hz = 10.0;
    double log_hz = 10.0;
    double max_linear_speed_mps = 0.25;
    double pause_linear_speed_mps = 0.40;
    double max_angular_speed_radps = 0.785;
    double pause_angular_speed_radps = 1.571;
    std::string output_dir = "/userdata/robot_slamd";
    bool save_on_exit = true;
    double autosave_period_s = 0.0;
    bool tof_pose_correction_enabled = false;
    std::string tof_pose_correction_mode = "off";
    double tof_pose_correction_update_hz = 3.0;
    double tof_pose_correction_search_xy_m = 0.08;
    double tof_pose_correction_search_xy_step_m = 0.02;
    double tof_pose_correction_search_yaw_deg = 8.0;
    double tof_pose_correction_search_yaw_step_deg = 2.0;
    int tof_pose_correction_min_valid_tof = 2;
    double tof_pose_correction_max_residual_m = 0.25;
    double tof_pose_correction_min_margin = 0.05;
    double tof_pose_correction_max_xy_m = 0.03;
    double tof_pose_correction_max_yaw_deg = 2.0;
    double tof_pose_correction_gain = 0.3;
    bool tof_pose_correction_mapping_mode_apply = false;
    double tof_pose_correction_candidate_lpf_alpha = 0.2;
    int tof_pose_correction_required_consistency = 3;
    double tof_pose_correction_consistency_yaw_deg = 1.0;
    double tof_pose_correction_min_candidate_reliability = 0.7;
    double tof_pose_correction_min_best_second_margin = 0.02;
    bool tof_pose_correction_yaw_residual_curve_debug = false;
    double tof_pose_correction_debug_offset_x_m = 0.0;
    double tof_pose_correction_debug_offset_y_m = 0.0;
    double tof_pose_correction_debug_offset_yaw_deg = 0.0;
    bool spin_scan_enabled = false;
    std::string spin_scan_mode = "observe_only";
    double spin_scan_min_rotation_deg = 300.0;
    double spin_scan_target_rotation_deg = 360.0;
    double spin_scan_max_duration_s = 25.0;
    double spin_scan_max_linear_speed_mps = 0.03;
    double spin_scan_angular_speed_min_radps = 0.08;
    double spin_scan_angular_speed_max_radps = 0.45;
    double spin_scan_angle_bin_deg = 3.0;
    int spin_scan_min_valid_bins = 40;
    double spin_scan_min_valid_bin_ratio = 0.35;
    double spin_scan_match_search_yaw_deg = 15.0;
    double spin_scan_match_search_yaw_step_deg = 1.0;
    double spin_scan_min_best_second_margin = 0.02;
    double spin_scan_flat_curve_sharpness_thresh = 0.02;
    double spin_scan_multimodal_yaw_gap_deg = 3.0;
    double spin_scan_multimodal_residual_window = 0.02;
    double spin_scan_min_reliability = 0.75;
    bool spin_scan_debug_curve = true;
    bool map_quality_enabled = true;
    double map_quality_log_hz = 1.0;
    double map_quality_min_tof_valid_ratio = 0.30;
    double map_quality_min_map_update_rate_hz = 1.0;
    int map_quality_min_occupied_cells = 5;
    double map_quality_low_quality_score_threshold = 0.40;
    double map_quality_degraded_score_threshold = 0.25;
    double map_quality_window_s = 5.0;
    bool mapping_supervisor_enabled = true;
    double mapping_supervisor_log_hz = 1.0;
    double mapping_supervisor_startup_grace_s = 3.0;
    double mapping_supervisor_min_ready_time_s = 2.0;
    double mapping_supervisor_min_good_quality_score = 0.40;
    double mapping_supervisor_degraded_quality_score = 0.25;
    double mapping_supervisor_lost_quality_score = 0.10;
    double mapping_supervisor_max_degraded_duration_s = 10.0;
    double mapping_supervisor_max_no_data_duration_s = 5.0;
    bool mapping_supervisor_require_two_tof_routes = true;
    double mapping_supervisor_active_scan_after_low_quality_s = 3.0;
    double mapping_supervisor_active_scan_after_low_update_s = 3.0;
    int mapping_supervisor_encoder_error_degraded_threshold = 5;
    int mapping_supervisor_imu_error_degraded_threshold = 5;
    int mapping_supervisor_tof_unhealthy_degraded_threshold = 1;
    bool active_scan_enabled = true;
    double active_scan_log_hz = 1.0;
    bool active_scan_startup_scan_enabled = true;
    bool active_scan_low_quality_scan_enabled = true;
    bool active_scan_degraded_scan_enabled = true;
    bool active_scan_lost_scan_enabled = true;
    double active_scan_min_interval_s = 15.0;
    double active_scan_cooldown_s = 10.0;
    double active_scan_request_hold_s = 1.0;
    double active_scan_max_pending_s = 30.0;
    double active_scan_recommended_scan_angle_deg = 360.0;
    double active_scan_min_useful_scan_angle_deg = 90.0;
    double active_scan_complete_scan_angle_deg = 300.0;
    double active_scan_min_tof_valid_ratio_for_scan = 0.20;
    int active_scan_min_valid_tof_routes_for_scan = 1;
    double active_scan_max_linear_speed_for_scan_mps = 0.05;
    double active_scan_min_yaw_rate_for_observed_scan_dps = 5.0;
    double active_scan_max_yaw_rate_for_observed_scan_dps = 90.0;
    bool active_scan_observe_natural_rotation = true;
    bool active_scan_require_supervisor_recommendation = true;
    bool active_scan_execution_enabled = true;
    std::string active_scan_execution_mode = "dry_run";
    double active_scan_execution_log_hz = 5.0;
    bool active_scan_execution_require_active_scan_would_start = true;
    bool active_scan_execution_require_scan_recommended = true;
    bool active_scan_execution_require_localization_valid = bool{1};
    double active_scan_execution_target_scan_angle_deg = 360.0;
    double active_scan_execution_complete_scan_angle_deg = 300.0;
    double active_scan_execution_min_useful_scan_angle_deg = 90.0;
    double active_scan_execution_target_yaw_rate_dps = 20.0;
    double active_scan_execution_min_observed_yaw_rate_dps = 3.0;
    double active_scan_execution_max_observed_yaw_rate_dps = 60.0;
    double active_scan_execution_max_linear_speed_mps = 0.05;
    double active_scan_execution_precheck_hold_s = 0.5;
    double active_scan_execution_command_timeout_s = 30.0;
    double active_scan_execution_cooldown_s = 10.0;
    bool active_scan_execution_emit_zero_command_on_stop = true;
    bool active_scan_execution_command_log_only = true;
    bool active_scan_execution_hardware_write_enabled = false;
    bool sparse_scan_enabled = true;
    double sparse_scan_log_hz = 5.0;
    bool sparse_scan_collect_on_active_scan = true;
    bool sparse_scan_collect_on_command_verifying = true;
    bool sparse_scan_collect_on_natural_rotation = true;
    bool sparse_scan_require_active_scan_recommended = false;
    bool sparse_scan_require_mapping_allowed = false;
    double sparse_scan_min_yaw_rate_dps = 3.0;
    double sparse_scan_max_yaw_rate_dps = 90.0;
    double sparse_scan_max_linear_speed_mps = 0.08;
    double sparse_scan_min_range_m = 0.02;
    double sparse_scan_max_range_m = 10.0;
    int sparse_scan_min_confidence = 70;
    double sparse_scan_bin_angle_deg = 5.0;
    double sparse_scan_min_session_angle_deg = 45.0;
    double sparse_scan_useful_session_angle_deg = 90.0;
    double sparse_scan_complete_session_angle_deg = 300.0;
    double sparse_scan_session_timeout_s = 20.0;
    double sparse_scan_max_gap_s = 1.0;
    double sparse_scan_cooldown_s = 3.0;
    bool sparse_scan_keep_invalid_samples = true;
    bool sparse_scan_compute_hit_points = true;
    bool sparse_scan_write_sample_log = true;
    bool sparse_scan_write_bin_log = true;
    bool sparse_scan_write_session_log = true;
    int sparse_scan_max_samples_per_session = 2000;
    bool sparse_scan_yaw_match_enabled = true;
    double sparse_scan_yaw_match_log_hz = 2.0;
    bool sparse_scan_yaw_match_run_on_completed_sparse_scan = true;
    bool sparse_scan_yaw_match_run_on_useful_sparse_scan = true;
    bool sparse_scan_yaw_match_require_complete_session = false;
    double sparse_scan_yaw_match_max_yaw_search_deg = 10.0;
    double sparse_scan_yaw_match_coarse_step_deg = 1.0;
    bool sparse_scan_yaw_match_refine_enabled = true;
    double sparse_scan_yaw_match_refine_window_deg = 2.0;
    double sparse_scan_yaw_match_refine_step_deg = 0.25;
    int sparse_scan_yaw_match_min_valid_samples = 15;
    int sparse_scan_yaw_match_min_valid_bins = 8;
    double sparse_scan_yaw_match_min_observed_yaw_delta_deg = 90.0;
    double sparse_scan_yaw_match_min_valid_bin_ratio = 0.10;
    double sparse_scan_yaw_match_occupied_search_radius_m = 0.12;
    double sparse_scan_yaw_match_occupied_hit_reward = 1.0;
    double sparse_scan_yaw_match_free_hit_penalty = 0.7;
    double sparse_scan_yaw_match_unknown_hit_penalty = 0.2;
    double sparse_scan_yaw_match_distance_penalty_scale_m = 0.20;
    double sparse_scan_yaw_match_min_best_score = 0.20;
    double sparse_scan_yaw_match_min_score_margin = 0.08;
    double sparse_scan_yaw_match_min_inlier_ratio = 0.20;
    double sparse_scan_yaw_match_max_curve_flatness = 0.80;
    bool sparse_scan_yaw_match_multimodal_check_enabled = true;
    double sparse_scan_yaw_match_multimodal_peak_separation_deg = 3.0;
    double sparse_scan_yaw_match_max_second_peak_ratio = 0.85;
    int sparse_scan_yaw_match_max_samples_per_match = 300;
    bool sparse_scan_yaw_match_write_curve_log = true;
    bool sparse_scan_yaw_match_write_summary_log = true;
    bool yaw_correction_enabled = true;
    std::string yaw_correction_mode = "dry_run";
    double yaw_correction_log_hz = 2.0;
    bool yaw_correction_require_yaw_match_usable = true;
    bool yaw_correction_require_mapping_state_ok = true;
    bool yaw_correction_require_low_speed_or_static = true;
    bool yaw_correction_require_active_scan_complete = true;
    std::string yaw_correction_scan_completion_source = "either";
    bool yaw_correction_allow_yaw_match_evidence_complete = true;
    double yaw_correction_min_match_observed_yaw_delta_deg = 90.0;
    int yaw_correction_min_match_valid_samples = 15;
    int yaw_correction_min_match_valid_bins = 8;
    double yaw_correction_min_match_valid_bin_ratio = 0.10;
    bool yaw_correction_require_yaw_match_attempted = true;
    double yaw_correction_max_linear_speed_mps = 0.05;
    double yaw_correction_max_abs_yaw_rate_dps = 5.0;
    double yaw_correction_min_best_score = 0.20;
    double yaw_correction_min_score_margin = 0.08;
    double yaw_correction_min_inlier_ratio = 0.20;
    double yaw_correction_max_curve_flatness = 0.80;
    double yaw_correction_max_candidate_abs_deg = 5.0;
    double yaw_correction_min_candidate_abs_deg = 0.3;
    double yaw_correction_gain = 0.15;
    double yaw_correction_max_step_deg = 1.0;
    double yaw_correction_min_step_deg = 0.05;
    int yaw_correction_consistency_window = 3;
    double yaw_correction_max_consistency_spread_deg = 1.0;
    double yaw_correction_cooldown_s = 5.0;
    bool yaw_correction_writeback_enabled = false;
    std::string yaw_correction_acknowledgement = "";
    std::string yaw_correction_writeback_acknowledgement = "";
    std::string yaw_correction_required_writeback_acknowledgement = "I_UNDERSTAND_YAW_ONLY_CORRECTION_RISK";
    double yaw_correction_max_writeback_abs_deg = 1.0;
    double yaw_correction_max_total_writeback_per_session_deg = 3.0;
    int yaw_correction_max_writeback_count_per_session = 5;
    double yaw_correction_min_seconds_between_writebacks = 5.0;
    bool yaw_correction_require_gate_would_apply = true;
    std::string yaw_correction_require_gate_reason = "would_apply_dry_run";
    bool yaw_correction_require_scan_evidence_ok = true;
    bool yaw_correction_require_yaw_match_evidence_ok = true;
    bool yaw_correction_apply_log_enabled = true;
    bool yaw_correction_post_apply_enabled = true;
    double yaw_correction_post_apply_timeout_s = 20.0;
    double yaw_correction_post_apply_min_improvement_deg = 0.2;
    double yaw_correction_post_apply_min_improvement_fraction_of_applied_delta = 0.3;
    double yaw_correction_post_apply_min_absolute_improvement_deg = 0.05;
    double yaw_correction_post_apply_max_allowed_worse_deg = 0.5;
    bool yaw_correction_post_apply_require_new_scan_id = true;
    bool yaw_correction_post_apply_require_newer_match_timestamp = true;
    double yaw_correction_post_apply_max_post_apply_candidate_abs_deg = 10.0;
    bool yaw_correction_post_apply_log_enabled = true;
    bool recovery_enabled = true;
    std::string recovery_mode = "observe_only";
    double recovery_log_hz = 2.0;
    bool recovery_startup_recovery_enabled = true;
    bool recovery_lost_recovery_enabled = true;
    bool recovery_degraded_recovery_enabled = true;
    double recovery_startup_grace_s = 3.0;
    double recovery_lost_confirm_s = 2.0;
    double recovery_degraded_confirm_s = 5.0;
    bool recovery_require_map_quality_not_invalid = true;
    int recovery_require_min_known_cells = 50;
    int recovery_require_min_occupied_cells = 5;
    bool recovery_require_tof_recent = true;
    double recovery_max_tof_age_s = 1.0;
    bool recovery_require_localizer_initialized_for_local_recovery = true;
    bool recovery_allow_uninitialized_startup_recovery_observe_only = true;
    bool recovery_request_recovery_scan = true;
    double recovery_recovery_scan_cooldown_s = 10.0;
    bool recovery_require_yaw_correction_stable = true;
    int recovery_max_recent_yaw_apply_count = 3;
    int recovery_min_post_apply_validated_count = 0;
    bool recovery_block_if_post_apply_failed_recent = true;
    bool recovery_write_candidate_log = true;
    bool recovery_write_event_log = true;
    bool motion_execution_enabled = true;
    std::string motion_execution_mode = "dry_run";
    double motion_execution_log_hz = 5.0;
    std::string motion_execution_command_source = "active_scan_command";
    bool motion_execution_allow_recovery_scan_request_as_reason = true;
    bool motion_execution_require_active_scan_command = true;
    bool motion_execution_require_command_planner_verifying_or_commanding = true;
    double motion_execution_wheel_base_m = 0.160;
    double motion_execution_wheel_radius_m = 0.032;
    double motion_execution_max_linear_speed_mps = 0.0;
    double motion_execution_max_abs_yaw_rate_dps = 20.0;
    double motion_execution_max_wheel_speed_rpm = 60.0;
    double motion_execution_min_wheel_speed_rpm = 3.0;
    double motion_execution_max_command_duration_s = 5.0;
    double motion_execution_deadman_timeout_s = 0.3;
    double motion_execution_command_stale_timeout_s = 0.5;
    bool motion_execution_require_localizer_initialized = true;
    bool motion_execution_require_supervisor_not_lost = true;
    bool motion_execution_require_tof_recent = true;
    double motion_execution_max_tof_age_s = 0.5;
    bool motion_execution_obstacle_stop_enabled = true;
    double motion_execution_min_front_distance_m = 0.25;
    double motion_execution_min_side_distance_m = 0.15;
    bool motion_execution_require_encoder_feedback_recent = true;
    double motion_execution_max_encoder_age_s = 0.5;
    bool motion_execution_current_safety_enabled = true;
    double motion_execution_max_motor_current_a = 3.0;
    bool motion_execution_stall_detection_enabled = true;
    double motion_execution_stall_speed_rpm = 2.0;
    double motion_execution_stall_current_a = 2.0;
    int motion_execution_stall_confirm_count = 3;
    bool motion_execution_dry_run_write_motor_commands = false;
    bool motion_execution_hardware_write_enabled = false;
    std::string motion_execution_writeback_acknowledgement = "";
    std::string motion_execution_required_writeback_acknowledgement = "I_UNDERSTAND_MOTOR_WRITE_RISK";
    std::string motion_execution_write_mode_acknowledgement = "";
    std::string motion_execution_required_write_mode_acknowledgement = "I_UNDERSTAND_LOW_SPEED_MOTOR_WRITE_RISK";
    double motion_execution_max_allowed_write_yaw_rate_dps = 10.0;
    double motion_execution_max_allowed_write_duration_s = 3.0;
    bool motion_execution_apply_log_enabled = true;
};

struct Pose { double x = 0.0, y = 0.0, yaw = 0.0; };
struct EncoderSample { uint64_t t_us = 0; int64_t left_ticks = 0, right_ticks = 0; };

struct CjcBl4820WheelReading {
    bool ok = false;
    uint16_t position_raw = 0;
    int16_t rpm = 0;
    uint16_t current_raw = 0;
    uint8_t status = 0xff;
    std::string reason = "not_read";
};

struct EncoderLogSample {
    uint64_t t_us = 0;
    uint16_t left_pos_raw = 0;
    uint16_t right_pos_raw = 0;
    int64_t left_delta_ticks = 0;
    int64_t right_delta_ticks = 0;
    int64_t left_total_ticks = 0;
    int64_t right_total_ticks = 0;
    int16_t left_rpm = 0;
    int16_t right_rpm = 0;
    uint16_t left_current_raw = 0;
    uint16_t right_current_raw = 0;
    uint8_t left_status = 0xff;
    uint8_t right_status = 0xff;
    std::string decision = "not_read";
    std::string reason = "not_read";
};

struct EncoderStats {
    uint64_t left_reads = 0;
    uint64_t right_reads = 0;
    uint64_t left_errors = 0;
    uint64_t right_errors = 0;
    int64_t left_total_ticks = 0;
    int64_t right_total_ticks = 0;
    int16_t left_rpm = 0;
    int16_t right_rpm = 0;
    uint64_t uart_checksum_errors = 0;
    uint64_t uart_timeout_errors = 0;
    uint64_t uart_status_errors = 0;
    uint64_t uart_frame_errors = 0;
    uint64_t jump_rejects = 0;
    uint64_t left_consecutive_errors = 0;
    uint64_t right_consecutive_errors = 0;
    bool left_unhealthy = false;
    bool right_unhealthy = false;
};

struct ImuSample { uint64_t t_us = 0; double gyro_z_rad_s = 0.0; double raw_gyro_z_rad_s = 0.0; };
struct TofSample {
    uint64_t t_us = 0;
    std::string id;
    int raw_range_mm = 0;
    int range_status = 7;
    int confidence = 0;
};

struct TofLogEvent {
    uint64_t t_us = 0;
    std::string id;
    std::string decision;
};

struct TofFilterResult {
    bool map_update = false;
    bool hit = false;
    double range_m = 0.0;
    int filtered_confidence = 0;
    int stability_count = 0;
    std::string decision;
};

struct GridStats {
    size_t chunks = 0;
    size_t touched_cells = 0;
    int min_gx = 0, max_gx = 0, min_gy = 0, max_gy = 0;
    int width_cells = 0, height_cells = 0;
    uint64_t occupied_cells = 0;
    uint64_t free_cells = 0;
    uint64_t unknown_cells = 0;
    uint64_t known_cells = 0;
    uint64_t total_cells = 0;
    uint64_t map_alloc_failures = 0;
};

struct MapQualityRouteSnapshot {
    uint64_t samples = 0;
    uint64_t valid_hits = 0;
    uint64_t confidence_sum = 0;
    uint64_t confidence_count = 0;
};

struct MapQualitySnapshot {
    double timestamp_s = 0.0;
    double quality_score = 0.0;
    std::string quality_level = "NO_DATA";
    uint64_t tof_samples_total = 0;
    uint64_t tof_valid_hits = 0;
    uint64_t tof_low_confidence = 0;
    uint64_t tof_invalid = 0;
    uint64_t tof_rejected = 0;
    double tof_valid_ratio = 0.0;
    double tof_reject_ratio = 0.0;
    double tof_confidence_mean = 0.0;
    MapQualityRouteSnapshot front;
    MapQualityRouteSnapshot left;
    MapQualityRouteSnapshot right;
    uint64_t map_update_attempts = 0;
    uint64_t map_updates = 0;
    double map_update_rate_hz = 0.0;
    uint64_t occupied_cells = 0;
    uint64_t free_cells = 0;
    uint64_t unknown_cells = 0;
    uint64_t known_cells = 0;
    uint64_t total_cells = 0;
    double occupied_ratio = 0.0;
    double free_ratio = 0.0;
    double known_ratio = 0.0;
    uint64_t map_alloc_failures = 0;
    bool active_scan_recommended = false;
    bool degraded = false;
    std::string reason = "no_tof_data";
};

struct MapQualityRunStats {
    double score_last = 0.0;
    double score_min = 0.0;
    double score_sum = 0.0;
    uint64_t score_count = 0;
    uint64_t low_count = 0;
    uint64_t degraded_count = 0;
    uint64_t active_scan_recommended_count = 0;
    double tof_valid_ratio_last = 0.0;
    double tof_reject_ratio_last = 0.0;
    double map_update_rate_hz_last = 0.0;
    uint64_t map_known_cells_last = 0;
    uint64_t map_occupied_cells_last = 0;
    uint64_t map_alloc_failures = 0;
};

struct MappingSupervisorRunStats {
    std::string state_last = "INIT";
    std::string last_reason = "startup";
    uint64_t state_changes = 0;
    double mapping_seconds = 0.0;
    uint64_t active_scan_recommended_count = 0;
    uint64_t degraded_count = 0;
    uint64_t lost_count = 0;
    double active_scan_recommended_seconds = 0.0;
    double degraded_seconds = 0.0;
    double lost_seconds = 0.0;
};

struct ActiveScanRunStats {
    std::string state_last = "IDLE";
    std::string last_request_type = "NONE";
    std::string last_reason = "idle";
    uint64_t request_count = 0;
    uint64_t would_start_count = 0;
    uint64_t blocked_count = 0;
    uint64_t completed_count = 0;
    uint64_t useful_observed_count = 0;
    uint64_t state_changes = 0;
    double recommended_seconds = 0.0;
    double observing_seconds = 0.0;
    double cooldown_seconds = 0.0;
    double blocked_seconds = 0.0;
    double last_observed_yaw_delta_deg = 0.0;
    double max_observed_yaw_delta_deg = 0.0;
};

struct ActiveScanCommandRunStats {
    std::string state_last = "IDLE";
    std::string last_reason = "idle";
    std::string last_request_type = "NONE";
    uint64_t state_changes = 0;
    uint64_t precheck_count = 0;
    uint64_t command_start_count = 0;
    uint64_t zero_command_count = 0;
    uint64_t completed_count = 0;
    uint64_t aborted_count = 0;
    uint64_t blocked_count = 0;
    double command_active_seconds = 0.0;
    double verifying_seconds = 0.0;
    double cooldown_seconds = 0.0;
    double blocked_seconds = 0.0;
    double last_observed_yaw_delta_deg = 0.0;
    double max_observed_yaw_delta_deg = 0.0;
};

struct SparseScanRunStats {
    std::string state_last = "IDLE";
    uint64_t sessions_started = 0;
    uint64_t sessions_completed = 0;
    uint64_t sessions_aborted = 0;
    uint64_t useful_sessions = 0;
    uint64_t complete_sessions = 0;
    uint64_t total_samples = 0;
    uint64_t valid_samples = 0;
    uint64_t invalid_samples = 0;
    uint64_t front_valid = 0;
    uint64_t left_valid = 0;
    uint64_t right_valid = 0;
    double last_observed_yaw_delta_deg = 0.0;
    double max_observed_yaw_delta_deg = 0.0;
    double last_valid_bin_ratio = 0.0;
    double valid_bin_ratio_sum = 0.0;
    uint64_t valid_bin_ratio_count = 0;
};

struct SparseScanYawMatchRunStats {
    uint64_t attempts = 0;
    uint64_t usable_count = 0;
    uint64_t rejected_count = 0;
    double last_best_yaw_delta_deg = 0.0;
    double last_best_score = 0.0;
    double last_score_margin = 0.0;
    double last_inlier_ratio = 0.0;
    bool last_usable = false;
    std::string last_reason = "none";
    uint64_t curve_flat_count = 0;
    uint64_t multimodal_count = 0;
    uint64_t insufficient_data_count = 0;
};

struct YawCorrectionGateRunStats {
    std::string state_last = "IDLE";
    uint64_t candidates_seen = 0;
    uint64_t would_apply_count = 0;
    uint64_t rejected_count = 0;
    double last_candidate_deg = 0.0;
    double last_suggested_correction_deg = 0.0;
    std::string last_reason = "idle";
    int consistency_count_last = 0;
    double consistency_spread_deg_last = 0.0;
    uint64_t robot_moving_reject_count = 0;
    uint64_t low_quality_reject_count = 0;
    uint64_t consistency_reject_count = 0;
    uint64_t scan_evidence_reject_count = 0;
    uint64_t active_scan_evidence_reject_count = 0;
    uint64_t yaw_match_evidence_reject_count = 0;
    bool last_scan_evidence_ok = false;
    bool last_active_scan_evidence_ok = false;
    bool last_yaw_match_evidence_ok = false;
    double last_match_observed_yaw_delta_deg = 0.0;
    int last_match_valid_samples = 0;
    int last_match_valid_bins = 0;
    double last_match_valid_bin_ratio = 0.0;
    uint64_t apply_feedback_count = 0;
    uint64_t window_reset_count = 0;
    uint64_t last_applied_scan_id = 0;
    double last_applied_delta_deg = 0.0;
};

struct YawCorrectionApplyRunStats {
    uint64_t attempts = 0;
    uint64_t success_count = 0;
    uint64_t rejected_count = 0;
    bool last_applied = false;
    std::string last_reason = "idle";
    double last_delta_deg = 0.0;
    double last_old_yaw_rad = 0.0;
    double last_new_yaw_rad = 0.0;
    uint64_t session_count = 0;
    double session_total_abs_deg = 0.0;
    uint64_t cooldown_reject_count = 0;
    uint64_t gate_reject_count = 0;
    uint64_t safety_reject_count = 0;
    uint64_t duplicate_candidate_reject_count = 0;
    uint64_t last_match_scan_id = 0;
    double last_match_timestamp_s = 0.0;
    std::string localization_invalid_reason_last = "unknown";
};

struct YawCorrectionPostApplyRunStats {
    uint64_t validated_count = 0;
    uint64_t suspect_count = 0;
    uint64_t failed_count = 0;
    uint64_t timeout_count = 0;
    std::string last_state = "IDLE";
    std::string last_reason = "idle";
    double last_improvement_deg = 0.0;
    double last_expected_min_improvement_deg = 0.0;
    double last_new_match_timestamp_s = 0.0;
};


struct RecoveryManagerRunStats {
    std::string state_last = "IDLE";
    uint64_t state_changes = 0;
    uint64_t startup_observe_count = 0;
    uint64_t degraded_observe_count = 0;
    uint64_t lost_observe_count = 0;
    uint64_t scan_recommended_count = 0;
    uint64_t candidate_seen_count = 0;
    uint64_t candidate_ready_count = 0;
    uint64_t rejected_count = 0;
    uint64_t map_evidence_reject_count = 0;
    uint64_t tof_evidence_reject_count = 0;
    uint64_t localizer_evidence_reject_count = 0;
    uint64_t yaw_match_evidence_reject_count = 0;
    uint64_t yaw_correction_unstable_reject_count = 0;
    std::string last_reason = "idle";
    std::string last_type = "none";
    std::string last_missing_evidence = "none";
    uint64_t last_match_scan_id = 0;
    double last_yaw_candidate_deg = 0.0;
};


struct MotionSafetyExecutorRunStats {
    std::string state_last = "IDLE";
    uint64_t state_changes = 0;
    uint64_t command_seen_count = 0;
    uint64_t would_command_count = 0;
    uint64_t would_zero_count = 0;
    uint64_t blocked_count = 0;
    uint64_t fault_count = 0;
    uint64_t obstacle_stop_count = 0;
    uint64_t tof_stale_block_count = 0;
    uint64_t encoder_stale_block_count = 0;
    uint64_t current_high_block_count = 0;
    uint64_t status_error_block_count = 0;
    uint64_t stall_block_count = 0;
    uint64_t supervisor_lost_block_count = 0;
    std::string last_reason = "idle";
    double last_target_left_rpm = 0.0;
    double last_target_right_rpm = 0.0;
    double last_desired_yaw_rate_dps = 0.0;
    double last_front_distance_m = 0.0;
    double last_left_distance_m = 0.0;
    double last_right_distance_m = 0.0;
    uint64_t command_duration_exceeded_count = 0;
    uint64_t last_command_session_id = 0;
    double last_command_session_duration_s = 0.0;
    uint64_t command_stale_count = 0;
    uint64_t deadman_timeout_count = 0;
    double last_command_age_s = -1.0;
    double last_deadman_age_s = -1.0;
    bool write_authorization_valid_last = false;
    uint64_t feedback_not_finite_block_count = 0;
    uint64_t wheel_direction_invalid_count = 0;
};

struct RunMetrics {
    uint64_t localization_updates = 0;
    uint64_t tof_samples = 0;
    uint64_t tof_hit_updates = 0;
    uint64_t tof_free_only = 0;
    uint64_t tof_rejected = 0;
    uint64_t map_updates = 0;
    uint64_t map_alloc_failures = 0;
    uint64_t static_scan_boost_updates = 0;
    uint64_t low_odom_quality_pauses = 0;
    uint64_t tof_unhealthy_pauses = 0;
    uint64_t tof_correction_attempts = 0;
    uint64_t tof_correction_accepts = 0;
    uint64_t tof_correction_rejects = 0;
    double tof_correction_residual_sum = 0.0;
    double tof_correction_odom_residual_sum = 0.0;
    double tof_correction_improvement_sum = 0.0;
    double tof_correction_best_shift_sum = 0.0;
    double tof_correction_max_dx = 0.0;
    double tof_correction_max_dyaw = 0.0;
    double pose_quality_sum = 0.0;
    double pose_quality_min = 1.0;
    double localization_confidence_sum = 0.0;
    double localization_confidence_min = 1.0;
    uint64_t spin_scan_attempts = 0;
    uint64_t spin_scan_completed = 0;
    uint64_t spin_scan_failed = 0;
    uint64_t spin_scan_matched = 0;
    uint64_t spin_scan_usable = 0;
    double spin_scan_valid_bins_sum = 0.0;
    double spin_scan_valid_bin_ratio_sum = 0.0;
    double spin_scan_reliability_sum = 0.0;
    double spin_scan_reliability_min = 1.0;
    uint64_t spin_scan_multimodal_count = 0;
    uint64_t spin_scan_flat_curve_count = 0;
    uint64_t spin_scan_edge_best_count = 0;
    MapQualityRunStats map_quality;
    MappingSupervisorRunStats mapping_supervisor;
    ActiveScanRunStats active_scan;
    ActiveScanCommandRunStats active_scan_command;
    SparseScanRunStats sparse_scan;
    SparseScanYawMatchRunStats sparse_scan_yaw_match;
    YawCorrectionGateRunStats yaw_correction;
    YawCorrectionApplyRunStats yaw_correction_apply;
    YawCorrectionPostApplyRunStats yaw_correction_post_apply;
    RecoveryManagerRunStats recovery;
    MotionSafetyExecutorRunStats motion;
};

bool one_of(const std::string &v, std::initializer_list<const char *> allowed) {
    for (const char *a : allowed) if (v == a) return true;
    return false;
}

void add_unique(std::vector<std::string> &items, const std::string &value) {
    if (std::find(items.begin(), items.end(), value) == items.end()) items.push_back(value);
}

std::string join_pipe(const std::vector<std::string> &items) {
    std::ostringstream o;
    for (size_t i = 0; i < items.size(); ++i) { if (i) o << "|"; o << items[i]; }
    return o.str();
}

std::string join_errors(const std::vector<std::string> &items) {
    std::ostringstream o;
    for (size_t i = 0; i < items.size(); ++i) { if (i) o << "; "; o << items[i]; }
    return o.str();
}
std::vector<std::string> merge_warnings(std::vector<std::string> a, const std::vector<std::string> &b) {
    for (const auto &x : b) add_unique(a, x);
    return a;
}

std::vector<std::string> warning_list(const Config &c, double v, double w, const std::vector<std::string> &extra = {}, bool map_alloc_fail = false) {
    std::vector<std::string> f;
    if (std::fabs(v) > c.max_linear_speed_mps) add_unique(f, "fast_linear");
    if (std::fabs(v) > c.pause_linear_speed_mps) add_unique(f, "mapping_paused_fast_linear");
    if (std::fabs(w) > c.max_angular_speed_radps) add_unique(f, "fast_angular");
    if (std::fabs(w) > c.pause_angular_speed_radps) add_unique(f, "mapping_paused_fast_angular");
    if (map_alloc_fail) add_unique(f, "map_chunk_alloc_fail");
    for (const auto &wflag : extra) add_unique(f, wflag);
    return f;
}

std::string warn_flags(const std::vector<std::string> &f) {
    if (f.empty()) return "ok";
    return join_pipe(f);
}

double clamp01(double v) {
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

double confidence_weight(int confidence) {
    return clamp01(static_cast<double>(confidence) / 100.0);
}

double odom_quality_from_flags(const std::vector<std::string> &flags) {
    double q = 1.0;
    for (const auto &f : flags) {
        if (f == "encoder_jump") q -= 0.5;
        else if (f == "gyro_spike") q -= 0.3;
        else if (f == "encoder_gap") q -= 0.4;
        else if (f == "imu_gap") q -= 0.3;
        else if (f == "fast_linear") q -= 0.2;
        else if (f == "fast_angular") q -= 0.2;
    }
    return clamp01(q);
}

} // namespace robot_slamd
