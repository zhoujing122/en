#pragma once
#include "common.hpp"

namespace robot_slamd {

double get_double(const std::unordered_map<std::string, std::string> &kv, const std::string &k, double d) {
    auto it = kv.find(k);
    return it == kv.end() ? d : std::strtod(it->second.c_str(), nullptr);
}

int get_int(const std::unordered_map<std::string, std::string> &kv, const std::string &k, int d) {
    auto it = kv.find(k);
    return it == kv.end() ? d : std::strtol(it->second.c_str(), nullptr, 10);
}

bool get_bool(const std::unordered_map<std::string, std::string> &kv, const std::string &k, bool d) {
    auto it = kv.find(k);
    if (it == kv.end()) return d;
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    return v == "true" || v == "1" || v == "yes";
}

std::string get_string(const std::unordered_map<std::string, std::string> &kv, const std::string &k, const std::string &d) {
    auto it = kv.find(k);
    return it == kv.end() ? d : it->second;
}

const char *bool_yaml(bool v) {
    return v ? "true" : "false";
}

std::unordered_map<std::string, std::string> parse_config_file(const std::string &path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("failed to open config: " + path);
    std::unordered_map<std::string, std::string> kv;
    std::string section;
    std::string line;
    while (std::getline(in, line)) {
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        if (trim(line).empty()) continue;
        size_t indent = line.find_first_not_of(' ');
        std::string t = trim(line);
        size_t colon = t.find(':');
        if (colon == std::string::npos) continue;
        std::string key = trim(t.substr(0, colon));
        std::string val = trim(t.substr(colon + 1));
        if (indent == 0 && val.empty()) {
            section = key;
            continue;
        }
        std::string full = section.empty() ? key : section + "." + key;
        kv[full] = strip_quotes(val);
    }
    return kv;
}

Config load_config(const std::string &path, const std::string &output_override) {
    Config c;
    auto kv = parse_config_file(path);
    c.wheel_base_m = get_double(kv, "robot.wheel_base_m", c.wheel_base_m);
    c.wheel_radius_left_m = get_double(kv, "robot.wheel_radius_left_m", c.wheel_radius_left_m);
    c.wheel_radius_right_m = get_double(kv, "robot.wheel_radius_right_m", c.wheel_radius_right_m);
    c.encoder_ticks_per_rev = get_int(kv, "robot.encoder_ticks_per_rev", c.encoder_ticks_per_rev);
    c.encoder_ticks_per_rev = get_int(kv, "encoder.counts_per_rev", c.encoder_ticks_per_rev);
    c.encoder_source = get_string(kv, "encoder.source", c.encoder_source);
    c.encoder_source = get_string(kv, "encoder.mode", c.encoder_source);
    c.encoder_path = get_string(kv, "encoder.path", c.encoder_path);
    c.encoder_left_port = get_string(kv, "encoder.left_port", c.encoder_left_port);
    c.encoder_right_port = get_string(kv, "encoder.right_port", c.encoder_right_port);
    c.encoder_baudrate = get_int(kv, "encoder.baudrate", c.encoder_baudrate);
    c.encoder_uart_timeout_ms = get_int(kv, "encoder.uart_timeout_ms", c.encoder_uart_timeout_ms);
    c.encoder_left_id = get_int(kv, "encoder.left_id", c.encoder_left_id);
    c.encoder_right_id = get_int(kv, "encoder.right_id", c.encoder_right_id);
    c.encoder_left_sign = get_int(kv, "encoder.left_sign", c.encoder_left_sign);
    c.encoder_right_sign = get_int(kv, "encoder.right_sign", c.encoder_right_sign);
    c.encoder_read_hz = get_double(kv, "encoder.read_hz", c.encoder_read_hz);
    c.encoder_position_unwrap = get_bool(kv, "encoder.position_unwrap", c.encoder_position_unwrap);
    c.encoder_speed_diagnostic = get_bool(kv, "encoder.speed_diagnostic", c.encoder_speed_diagnostic);
    c.encoder_consecutive_error_limit = get_int(kv, "encoder.consecutive_error_limit", c.encoder_consecutive_error_limit);
    c.imu_source = get_string(kv, "imu.source", c.imu_source);
    c.imu_path = get_string(kv, "imu.path", c.imu_path);
    c.imu_mode = get_string(kv, "imu.mode", c.imu_mode);
    c.imu_gyro_axis = get_string(kv, "imu.gyro_axis", c.imu_gyro_axis);
    c.imu_gyro_sign = get_int(kv, "imu.gyro_sign", c.imu_gyro_sign);
    c.imu_icm43600_lib_path = get_string(kv, "imu.icm43600_lib_path", c.imu_icm43600_lib_path);
    c.imu_icm43600_device_no = static_cast<uint64_t>(get_int(kv, "imu.icm43600_device_no", static_cast<int>(c.imu_icm43600_device_no)));
    c.imu_icm43600_accel_rate_hz = static_cast<uint64_t>(get_int(kv, "imu.icm43600_accel_rate_hz", static_cast<int>(c.imu_icm43600_accel_rate_hz)));
    c.imu_icm43600_gyro_rate_hz = static_cast<uint64_t>(get_int(kv, "imu.icm43600_gyro_rate_hz", static_cast<int>(c.imu_icm43600_gyro_rate_hz)));
    c.imu_icm43600_batch_timeout_ms = static_cast<uint64_t>(get_int(kv, "imu.icm43600_batch_timeout_ms", static_cast<int>(c.imu_icm43600_batch_timeout_ms)));
    c.imu_icm43600_accel_fsr = static_cast<uint32_t>(get_int(kv, "imu.icm43600_accel_fsr", static_cast<int>(c.imu_icm43600_accel_fsr)));
    c.imu_icm43600_gyro_fsr = static_cast<uint32_t>(get_int(kv, "imu.icm43600_gyro_fsr", static_cast<int>(c.imu_icm43600_gyro_fsr)));
    c.imu_icm43600_high_res_fifo = get_bool(kv, "imu.icm43600_high_res_fifo", c.imu_icm43600_high_res_fifo);
    c.gyro_bias_static_calib_s = get_double(kv, "imu.gyro_bias_static_calib_s", c.gyro_bias_static_calib_s);
    c.static_gyro_max_abs_rad_s = get_double(kv, "imu.static_gyro_max_abs_rad_s", c.static_gyro_max_abs_rad_s);
    c.yaw_fusion_alpha = get_double(kv, "imu.yaw_fusion_alpha", c.yaw_fusion_alpha);
    c.encoder_max_wheel_speed_mps = get_double(kv, "localization.encoder_max_wheel_speed_mps", c.encoder_max_wheel_speed_mps);
    c.gyro_lpf_alpha = get_double(kv, "localization.gyro_lpf_alpha", c.gyro_lpf_alpha);
    c.gyro_spike_radps = get_double(kv, "localization.gyro_spike_radps", c.gyro_spike_radps);
    c.stationary_linear_speed_mps = get_double(kv, "localization.stationary_linear_speed_mps", c.stationary_linear_speed_mps);
    c.stationary_angular_speed_radps = get_double(kv, "localization.stationary_angular_speed_radps", c.stationary_angular_speed_radps);
    c.gyro_bias_adapt_alpha = get_double(kv, "localization.gyro_bias_adapt_alpha", c.gyro_bias_adapt_alpha);
    c.tof_source = get_string(kv, "tof.source", c.tof_source);
    c.tof_csv_path = get_string(kv, "tof.csv_path", c.tof_csv_path);
    c.tof_frequency_hz = get_double(kv, "tof.frequency_hz", c.tof_frequency_hz);
    c.tof_min_range_m = get_double(kv, "tof.min_range_m", c.tof_min_range_m);
    c.tof_max_range_m = get_double(kv, "tof.max_range_m", c.tof_max_range_m);
    c.confidence_min = get_int(kv, "tof.confidence_min", c.confidence_min);
    c.median_window = get_int(kv, "tof.median_window", c.median_window);
    c.jump_reject_m = get_double(kv, "tof.jump_reject_m", c.jump_reject_m);
    c.mad_reject_m = get_double(kv, "tof.mad_reject_m", c.mad_reject_m);
    c.stable_hits_required = get_int(kv, "tof.stable_hits_required", c.stable_hits_required);
    for (auto &id : {"front", "left", "right"}) {
        c.tof_devices[id] = get_string(kv, std::string("tof_devices.") + id, c.tof_devices[id]);
        auto &e = c.tof_extrinsics[id];
        std::string p = std::string("tof_extrinsics.") + id + ".";
        e.x_m = get_double(kv, p + "x_m", e.x_m);
        e.y_m = get_double(kv, p + "y_m", e.y_m);
        e.yaw_deg = get_double(kv, p + "yaw_deg", e.yaw_deg);
        e.fov_deg = get_double(kv, p + "fov_deg", e.fov_deg);
    }
    c.resolution_m = get_double(kv, "mapping.resolution_m", c.resolution_m);
    c.chunk_cells = get_int(kv, "mapping.chunk_cells", c.chunk_cells);
    c.initial_width_m = get_double(kv, "mapping.initial_width_m", c.initial_width_m);
    c.initial_height_m = get_double(kv, "mapping.initial_height_m", c.initial_height_m);
    c.ray_step_deg = get_double(kv, "mapping.ray_step_deg", c.ray_step_deg);
    c.hit_thickness_m = get_double(kv, "mapping.hit_thickness_m", c.hit_thickness_m);
    c.log_odds_occ = get_double(kv, "mapping.log_odds_occ", c.log_odds_occ);
    c.log_odds_free = get_double(kv, "mapping.log_odds_free", c.log_odds_free);
    c.log_odds_min = get_double(kv, "mapping.log_odds_min", c.log_odds_min);
    c.log_odds_max = get_double(kv, "mapping.log_odds_max", c.log_odds_max);
    c.occupied_thresh = get_double(kv, "mapping.occupied_thresh", c.occupied_thresh);
    c.free_thresh = get_double(kv, "mapping.free_thresh", c.free_thresh);
    c.max_cells_per_tof_update = get_int(kv, "mapping.max_cells_per_tof_update", c.max_cells_per_tof_update);
    c.static_scan_stable_required = get_int(kv, "mapping.static_scan_stable_required", c.static_scan_stable_required);
    c.static_scan_boost = get_double(kv, "mapping.static_scan_boost", c.static_scan_boost);
    c.localization_hz = get_double(kv, "runtime.localization_hz", c.localization_hz);
    c.tof_read_hz = get_double(kv, "runtime.tof_read_hz", c.tof_read_hz);
    c.mapping_hz = get_double(kv, "runtime.mapping_hz", c.mapping_hz);
    c.log_hz = get_double(kv, "runtime.log_hz", c.log_hz);
    c.max_linear_speed_mps = get_double(kv, "runtime_limits.max_linear_speed_mps", c.max_linear_speed_mps);
    c.pause_linear_speed_mps = get_double(kv, "runtime_limits.pause_linear_speed_mps", c.pause_linear_speed_mps);
    c.max_angular_speed_radps = get_double(kv, "runtime_limits.max_angular_speed_radps", c.max_angular_speed_radps);
    c.pause_angular_speed_radps = get_double(kv, "runtime_limits.pause_angular_speed_radps", c.pause_angular_speed_radps);
    c.output_dir = get_string(kv, "logging.output_dir", c.output_dir);
    c.save_on_exit = get_bool(kv, "map_saver.save_on_exit", c.save_on_exit);
    c.autosave_period_s = get_double(kv, "map_saver.autosave_period_s", c.autosave_period_s);
    c.tof_pose_correction_enabled = get_bool(kv, "tof_pose_correction.enabled", c.tof_pose_correction_enabled);
    c.tof_pose_correction_mode = get_string(kv, "tof_pose_correction.mode", c.tof_pose_correction_mode);
    c.tof_pose_correction_update_hz = get_double(kv, "tof_pose_correction.update_hz", c.tof_pose_correction_update_hz);
    c.tof_pose_correction_search_xy_m = get_double(kv, "tof_pose_correction.search_xy_m", c.tof_pose_correction_search_xy_m);
    c.tof_pose_correction_search_xy_step_m = get_double(kv, "tof_pose_correction.search_xy_step_m", c.tof_pose_correction_search_xy_step_m);
    c.tof_pose_correction_search_yaw_deg = get_double(kv, "tof_pose_correction.search_yaw_deg", c.tof_pose_correction_search_yaw_deg);
    c.tof_pose_correction_search_yaw_step_deg = get_double(kv, "tof_pose_correction.search_yaw_step_deg", c.tof_pose_correction_search_yaw_step_deg);
    c.tof_pose_correction_min_valid_tof = get_int(kv, "tof_pose_correction.min_valid_tof", c.tof_pose_correction_min_valid_tof);
    c.tof_pose_correction_max_residual_m = get_double(kv, "tof_pose_correction.max_residual_m", c.tof_pose_correction_max_residual_m);
    c.tof_pose_correction_min_margin = get_double(kv, "tof_pose_correction.min_margin", c.tof_pose_correction_min_margin);
    c.tof_pose_correction_max_xy_m = get_double(kv, "tof_pose_correction.max_xy_m", c.tof_pose_correction_max_xy_m);
    c.tof_pose_correction_max_yaw_deg = get_double(kv, "tof_pose_correction.max_yaw_deg", c.tof_pose_correction_max_yaw_deg);
    c.tof_pose_correction_gain = get_double(kv, "tof_pose_correction.gain", c.tof_pose_correction_gain);
    c.tof_pose_correction_mapping_mode_apply = get_bool(kv, "tof_pose_correction.mapping_mode_apply", c.tof_pose_correction_mapping_mode_apply);
    c.tof_pose_correction_candidate_lpf_alpha = get_double(kv, "tof_pose_correction.candidate_lpf_alpha", c.tof_pose_correction_candidate_lpf_alpha);
    c.tof_pose_correction_required_consistency = get_int(kv, "tof_pose_correction.required_consistency", c.tof_pose_correction_required_consistency);
    c.tof_pose_correction_consistency_yaw_deg = get_double(kv, "tof_pose_correction.consistency_yaw_deg", c.tof_pose_correction_consistency_yaw_deg);
    c.tof_pose_correction_min_candidate_reliability = get_double(kv, "tof_pose_correction.min_candidate_reliability", c.tof_pose_correction_min_candidate_reliability);
    c.tof_pose_correction_min_best_second_margin = get_double(kv, "tof_pose_correction.min_best_second_margin", c.tof_pose_correction_min_best_second_margin);
    c.tof_pose_correction_yaw_residual_curve_debug = get_bool(kv, "tof_pose_correction.yaw_residual_curve_debug", c.tof_pose_correction_yaw_residual_curve_debug);
    c.tof_pose_correction_debug_offset_x_m = get_double(kv, "tof_pose_correction.debug_pose_offset_x_m", c.tof_pose_correction_debug_offset_x_m);
    c.tof_pose_correction_debug_offset_y_m = get_double(kv, "tof_pose_correction.debug_pose_offset_y_m", c.tof_pose_correction_debug_offset_y_m);
    c.tof_pose_correction_debug_offset_yaw_deg = get_double(kv, "tof_pose_correction.debug_pose_offset_yaw_deg", c.tof_pose_correction_debug_offset_yaw_deg);
    c.spin_scan_enabled = get_bool(kv, "spin_scan_localization.enabled", c.spin_scan_enabled);
    c.spin_scan_mode = get_string(kv, "spin_scan_localization.mode", c.spin_scan_mode);
    c.spin_scan_min_rotation_deg = get_double(kv, "spin_scan_localization.min_rotation_deg", c.spin_scan_min_rotation_deg);
    c.spin_scan_target_rotation_deg = get_double(kv, "spin_scan_localization.target_rotation_deg", c.spin_scan_target_rotation_deg);
    c.spin_scan_max_duration_s = get_double(kv, "spin_scan_localization.max_duration_s", c.spin_scan_max_duration_s);
    c.spin_scan_max_linear_speed_mps = get_double(kv, "spin_scan_localization.max_linear_speed_mps", c.spin_scan_max_linear_speed_mps);
    c.spin_scan_angular_speed_min_radps = get_double(kv, "spin_scan_localization.angular_speed_min_radps", c.spin_scan_angular_speed_min_radps);
    c.spin_scan_angular_speed_max_radps = get_double(kv, "spin_scan_localization.angular_speed_max_radps", c.spin_scan_angular_speed_max_radps);
    c.spin_scan_angle_bin_deg = get_double(kv, "spin_scan_localization.angle_bin_deg", c.spin_scan_angle_bin_deg);
    c.spin_scan_min_valid_bins = get_int(kv, "spin_scan_localization.min_valid_bins", c.spin_scan_min_valid_bins);
    c.spin_scan_min_valid_bin_ratio = get_double(kv, "spin_scan_localization.min_valid_bin_ratio", c.spin_scan_min_valid_bin_ratio);
    c.spin_scan_match_search_yaw_deg = get_double(kv, "spin_scan_localization.match_search_yaw_deg", c.spin_scan_match_search_yaw_deg);
    c.spin_scan_match_search_yaw_step_deg = get_double(kv, "spin_scan_localization.match_search_yaw_step_deg", c.spin_scan_match_search_yaw_step_deg);
    c.spin_scan_min_best_second_margin = get_double(kv, "spin_scan_localization.min_best_second_margin", c.spin_scan_min_best_second_margin);
    c.spin_scan_flat_curve_sharpness_thresh = get_double(kv, "spin_scan_localization.flat_curve_sharpness_thresh", c.spin_scan_flat_curve_sharpness_thresh);
    c.spin_scan_multimodal_yaw_gap_deg = get_double(kv, "spin_scan_localization.multimodal_yaw_gap_deg", c.spin_scan_multimodal_yaw_gap_deg);
    c.spin_scan_multimodal_residual_window = get_double(kv, "spin_scan_localization.multimodal_residual_window", c.spin_scan_multimodal_residual_window);
    c.spin_scan_min_reliability = get_double(kv, "spin_scan_localization.min_reliability", c.spin_scan_min_reliability);
    c.spin_scan_debug_curve = get_bool(kv, "spin_scan_localization.debug_curve", c.spin_scan_debug_curve);
    c.map_quality_enabled = get_bool(kv, "map_quality.enabled", c.map_quality_enabled);
    c.map_quality_log_hz = get_double(kv, "map_quality.log_hz", c.map_quality_log_hz);
    c.map_quality_min_tof_valid_ratio = get_double(kv, "map_quality.min_tof_valid_ratio", c.map_quality_min_tof_valid_ratio);
    c.map_quality_min_map_update_rate_hz = get_double(kv, "map_quality.min_map_update_rate_hz", c.map_quality_min_map_update_rate_hz);
    c.map_quality_min_occupied_cells = get_int(kv, "map_quality.min_occupied_cells", c.map_quality_min_occupied_cells);
    c.map_quality_low_quality_score_threshold = get_double(kv, "map_quality.low_quality_score_threshold", c.map_quality_low_quality_score_threshold);
    c.map_quality_degraded_score_threshold = get_double(kv, "map_quality.degraded_score_threshold", c.map_quality_degraded_score_threshold);
    c.map_quality_window_s = get_double(kv, "map_quality.window_s", c.map_quality_window_s);
    c.mapping_supervisor_enabled = get_bool(kv, "mapping_supervisor.enabled", c.mapping_supervisor_enabled);
    c.mapping_supervisor_log_hz = get_double(kv, "mapping_supervisor.log_hz", c.mapping_supervisor_log_hz);
    c.mapping_supervisor_startup_grace_s = get_double(kv, "mapping_supervisor.startup_grace_s", c.mapping_supervisor_startup_grace_s);
    c.mapping_supervisor_min_ready_time_s = get_double(kv, "mapping_supervisor.min_ready_time_s", c.mapping_supervisor_min_ready_time_s);
    c.mapping_supervisor_min_good_quality_score = get_double(kv, "mapping_supervisor.min_good_quality_score", c.mapping_supervisor_min_good_quality_score);
    c.mapping_supervisor_degraded_quality_score = get_double(kv, "mapping_supervisor.degraded_quality_score", c.mapping_supervisor_degraded_quality_score);
    c.mapping_supervisor_lost_quality_score = get_double(kv, "mapping_supervisor.lost_quality_score", c.mapping_supervisor_lost_quality_score);
    c.mapping_supervisor_max_degraded_duration_s = get_double(kv, "mapping_supervisor.max_degraded_duration_s", c.mapping_supervisor_max_degraded_duration_s);
    c.mapping_supervisor_max_no_data_duration_s = get_double(kv, "mapping_supervisor.max_no_data_duration_s", c.mapping_supervisor_max_no_data_duration_s);
    c.mapping_supervisor_require_two_tof_routes = get_bool(kv, "mapping_supervisor.require_two_tof_routes", c.mapping_supervisor_require_two_tof_routes);
    c.mapping_supervisor_active_scan_after_low_quality_s = get_double(kv, "mapping_supervisor.active_scan_after_low_quality_s", c.mapping_supervisor_active_scan_after_low_quality_s);
    c.mapping_supervisor_active_scan_after_low_update_s = get_double(kv, "mapping_supervisor.active_scan_after_low_update_s", c.mapping_supervisor_active_scan_after_low_update_s);
    c.mapping_supervisor_encoder_error_degraded_threshold = get_int(kv, "mapping_supervisor.encoder_error_degraded_threshold", c.mapping_supervisor_encoder_error_degraded_threshold);
    c.mapping_supervisor_imu_error_degraded_threshold = get_int(kv, "mapping_supervisor.imu_error_degraded_threshold", c.mapping_supervisor_imu_error_degraded_threshold);
    c.mapping_supervisor_tof_unhealthy_degraded_threshold = get_int(kv, "mapping_supervisor.tof_unhealthy_degraded_threshold", c.mapping_supervisor_tof_unhealthy_degraded_threshold);
    c.active_scan_enabled = get_bool(kv, "active_scan.enabled", c.active_scan_enabled);
    c.active_scan_log_hz = get_double(kv, "active_scan.log_hz", c.active_scan_log_hz);
    c.active_scan_startup_scan_enabled = get_bool(kv, "active_scan.startup_scan_enabled", c.active_scan_startup_scan_enabled);
    c.active_scan_low_quality_scan_enabled = get_bool(kv, "active_scan.low_quality_scan_enabled", c.active_scan_low_quality_scan_enabled);
    c.active_scan_degraded_scan_enabled = get_bool(kv, "active_scan.degraded_scan_enabled", c.active_scan_degraded_scan_enabled);
    c.active_scan_lost_scan_enabled = get_bool(kv, "active_scan.lost_scan_enabled", c.active_scan_lost_scan_enabled);
    c.active_scan_min_interval_s = get_double(kv, "active_scan.min_interval_s", c.active_scan_min_interval_s);
    c.active_scan_cooldown_s = get_double(kv, "active_scan.cooldown_s", c.active_scan_cooldown_s);
    c.active_scan_request_hold_s = get_double(kv, "active_scan.request_hold_s", c.active_scan_request_hold_s);
    c.active_scan_max_pending_s = get_double(kv, "active_scan.max_pending_s", c.active_scan_max_pending_s);
    c.active_scan_recommended_scan_angle_deg = get_double(kv, "active_scan.recommended_scan_angle_deg", c.active_scan_recommended_scan_angle_deg);
    c.active_scan_min_useful_scan_angle_deg = get_double(kv, "active_scan.min_useful_scan_angle_deg", c.active_scan_min_useful_scan_angle_deg);
    c.active_scan_complete_scan_angle_deg = get_double(kv, "active_scan.complete_scan_angle_deg", c.active_scan_complete_scan_angle_deg);
    c.active_scan_min_tof_valid_ratio_for_scan = get_double(kv, "active_scan.min_tof_valid_ratio_for_scan", c.active_scan_min_tof_valid_ratio_for_scan);
    c.active_scan_min_valid_tof_routes_for_scan = get_int(kv, "active_scan.min_valid_tof_routes_for_scan", c.active_scan_min_valid_tof_routes_for_scan);
    c.active_scan_max_linear_speed_for_scan_mps = get_double(kv, "active_scan.max_linear_speed_for_scan_mps", c.active_scan_max_linear_speed_for_scan_mps);
    c.active_scan_min_yaw_rate_for_observed_scan_dps = get_double(kv, "active_scan.min_yaw_rate_for_observed_scan_dps", c.active_scan_min_yaw_rate_for_observed_scan_dps);
    c.active_scan_max_yaw_rate_for_observed_scan_dps = get_double(kv, "active_scan.max_yaw_rate_for_observed_scan_dps", c.active_scan_max_yaw_rate_for_observed_scan_dps);
    c.active_scan_observe_natural_rotation = get_bool(kv, "active_scan.observe_natural_rotation", c.active_scan_observe_natural_rotation);
    c.active_scan_require_supervisor_recommendation = get_bool(kv, "active_scan.require_supervisor_recommendation", c.active_scan_require_supervisor_recommendation);
    if (!output_override.empty()) c.output_dir = output_override;
    return c;
}

void validate_config(const Config &c) {
    std::vector<std::string> errors;
    auto positive = [&](const char *name, double v) {
        if (!(v > 0.0) || !std::isfinite(v)) errors.push_back(std::string(name) + " must be > 0");
    };
    auto non_negative = [&](const char *name, double v) {
        if (v < 0.0 || !std::isfinite(v)) errors.push_back(std::string(name) + " must be >= 0");
    };
    auto probability = [&](const char *name, double v) {
        if (v < 0.0 || v > 1.0 || !std::isfinite(v)) errors.push_back(std::string(name) + " must be in [0, 1]");
    };

    positive("robot.wheel_base_m", c.wheel_base_m);
    positive("robot.wheel_radius_left_m", c.wheel_radius_left_m);
    positive("robot.wheel_radius_right_m", c.wheel_radius_right_m);
    if (c.encoder_ticks_per_rev <= 0) errors.push_back("robot.encoder_ticks_per_rev / encoder.counts_per_rev must be > 0");
    if (!one_of(c.encoder_source, {"sim", "csv", "cjc_bl4820_uart"})) errors.push_back("encoder.source/mode must be sim, csv, or cjc_bl4820_uart");
    if (c.encoder_source == "cjc_bl4820_uart") {
        if (c.encoder_left_port.empty()) errors.push_back("encoder.left_port must not be empty when encoder.mode=cjc_bl4820_uart");
        if (c.encoder_right_port.empty()) errors.push_back("encoder.right_port must not be empty when encoder.mode=cjc_bl4820_uart");
        if (!c.encoder_left_port.empty() && c.encoder_left_port == c.encoder_right_port) errors.push_back("encoder.left_port and encoder.right_port must differ because BL4820 does not support shared UART bus IDs");
        if (c.encoder_baudrate != 1000000) errors.push_back("encoder.baudrate must be 1000000 for CJC BL4820 protocol");
        if (c.encoder_uart_timeout_ms <= 0 || c.encoder_uart_timeout_ms > 1000) errors.push_back("encoder.uart_timeout_ms must be in [1, 1000]");
        if (c.encoder_left_id < 1 || c.encoder_left_id > 253) errors.push_back("encoder.left_id must be in [1, 253]");
        if (c.encoder_right_id < 1 || c.encoder_right_id > 253) errors.push_back("encoder.right_id must be in [1, 253]");
        if (c.encoder_left_sign != 1 && c.encoder_left_sign != -1) errors.push_back("encoder.left_sign must be 1 or -1");
        if (c.encoder_right_sign != 1 && c.encoder_right_sign != -1) errors.push_back("encoder.right_sign must be 1 or -1");
        if (c.encoder_ticks_per_rev != 1024) errors.push_back("encoder.counts_per_rev must be 1024 for CJC BL4820 single-turn position");
        if (!c.encoder_position_unwrap) errors.push_back("encoder.position_unwrap must be true when encoder.mode=cjc_bl4820_uart");
        if (!(c.encoder_read_hz > 0.0) || !std::isfinite(c.encoder_read_hz)) errors.push_back("encoder.read_hz must be > 0");
        if (c.encoder_consecutive_error_limit < 1) errors.push_back("encoder.consecutive_error_limit must be >= 1");
    }
    if (!one_of(c.imu_source, {"sim", "csv", "icm43600_ffi"})) errors.push_back("imu.source must be sim, csv, or icm43600_ffi");
    if (!one_of(c.imu_mode, {"localization", "log_only"})) errors.push_back("imu.mode must be localization or log_only");
    if (!one_of(c.imu_gyro_axis, {"x", "y", "z"})) errors.push_back("imu.gyro_axis must be x, y, or z");
    if (c.imu_gyro_sign != 1 && c.imu_gyro_sign != -1) errors.push_back("imu.gyro_sign must be 1 or -1");
    if (!one_of(c.tof_source, {"sim", "csv", "mt3801_ioctl"})) errors.push_back("tof.source must be sim, csv, or mt3801_ioctl");

    if (c.imu_source == "icm43600_ffi" && c.imu_icm43600_lib_path.empty()) errors.push_back("imu.icm43600_lib_path must not be empty when imu.source=icm43600_ffi");
    if (c.imu_icm43600_accel_rate_hz == 0) errors.push_back("imu.icm43600_accel_rate_hz must be > 0");
    if (c.imu_icm43600_gyro_rate_hz == 0) errors.push_back("imu.icm43600_gyro_rate_hz must be > 0");
    if (c.imu_icm43600_accel_fsr > 4) errors.push_back("imu.icm43600_accel_fsr must be in [0, 4]");
    if (c.imu_icm43600_gyro_fsr > 4) errors.push_back("imu.icm43600_gyro_fsr must be in [0, 4]");
    non_negative("imu.gyro_bias_static_calib_s", c.gyro_bias_static_calib_s);
    non_negative("imu.static_gyro_max_abs_rad_s", c.static_gyro_max_abs_rad_s);
    probability("imu.yaw_fusion_alpha", c.yaw_fusion_alpha);
    positive("localization.encoder_max_wheel_speed_mps", c.encoder_max_wheel_speed_mps);
    probability("localization.gyro_lpf_alpha", c.gyro_lpf_alpha);
    positive("localization.gyro_spike_radps", c.gyro_spike_radps);
    non_negative("localization.stationary_linear_speed_mps", c.stationary_linear_speed_mps);
    non_negative("localization.stationary_angular_speed_radps", c.stationary_angular_speed_radps);
    probability("localization.gyro_bias_adapt_alpha", c.gyro_bias_adapt_alpha);
    positive("tof.frequency_hz", c.tof_frequency_hz);
    positive("tof.min_range_m", c.tof_min_range_m);
    positive("tof.max_range_m", c.tof_max_range_m);
    if (c.tof_min_range_m >= c.tof_max_range_m) errors.push_back("tof.min_range_m must be < tof.max_range_m");
    if (c.confidence_min < 0 || c.confidence_min > 255) errors.push_back("tof.confidence_min must be in [0, 255]");
    if (c.median_window <= 0) errors.push_back("tof.median_window must be > 0");
    non_negative("tof.jump_reject_m", c.jump_reject_m);
    non_negative("tof.mad_reject_m", c.mad_reject_m);
    if (c.stable_hits_required <= 0) errors.push_back("tof.stable_hits_required must be > 0");
    for (const auto &id : {"front", "left", "right"}) {
        const auto &e = c.tof_extrinsics.at(id);
        positive((std::string("tof_extrinsics.") + id + ".fov_deg").c_str(), e.fov_deg);
        if (e.fov_deg > 180.0) errors.push_back(std::string("tof_extrinsics.") + id + ".fov_deg must be <= 180");
    }

    positive("mapping.resolution_m", c.resolution_m);
    if (c.chunk_cells <= 0) errors.push_back("mapping.chunk_cells must be > 0");
    positive("mapping.initial_width_m", c.initial_width_m);
    positive("mapping.initial_height_m", c.initial_height_m);
    positive("mapping.ray_step_deg", c.ray_step_deg);
    non_negative("mapping.hit_thickness_m", c.hit_thickness_m);
    if (!(c.log_odds_min < c.log_odds_max)) errors.push_back("mapping.log_odds_min must be < mapping.log_odds_max");
    probability("mapping.occupied_thresh", c.occupied_thresh);
    probability("mapping.free_thresh", c.free_thresh);
    if (!(c.free_thresh < c.occupied_thresh)) errors.push_back("mapping.free_thresh must be < mapping.occupied_thresh");
    if (c.max_cells_per_tof_update <= 0) errors.push_back("mapping.max_cells_per_tof_update must be > 0");
    if (c.static_scan_stable_required <= 0) errors.push_back("mapping.static_scan_stable_required must be > 0");
    positive("mapping.static_scan_boost", c.static_scan_boost);

    positive("runtime.localization_hz", c.localization_hz);
    positive("runtime.tof_read_hz", c.tof_read_hz);
    positive("runtime.mapping_hz", c.mapping_hz);
    positive("runtime.log_hz", c.log_hz);
    positive("runtime_limits.max_linear_speed_mps", c.max_linear_speed_mps);
    positive("runtime_limits.pause_linear_speed_mps", c.pause_linear_speed_mps);
    positive("runtime_limits.max_angular_speed_radps", c.max_angular_speed_radps);
    positive("runtime_limits.pause_angular_speed_radps", c.pause_angular_speed_radps);
    if (c.max_linear_speed_mps > c.pause_linear_speed_mps) errors.push_back("runtime_limits.max_linear_speed_mps must be <= pause_linear_speed_mps");
    if (c.max_angular_speed_radps > c.pause_angular_speed_radps) errors.push_back("runtime_limits.max_angular_speed_radps must be <= pause_angular_speed_radps");
    if (c.output_dir.empty()) errors.push_back("logging.output_dir must not be empty");
    if (c.autosave_period_s < 0.0 || !std::isfinite(c.autosave_period_s)) errors.push_back("map_saver.autosave_period_s must be >= 0");
    if (c.autosave_period_s > 0.0) errors.push_back("autosave_not_implemented: map_saver.autosave_period_s must be 0 in first version");

    if (!one_of(c.tof_pose_correction_mode, {"off", "score_only", "yaw_candidate", "yaw_only", "xy_yaw"})) errors.push_back("tof_pose_correction.mode must be off, score_only, yaw_candidate, yaw_only, or xy_yaw");
    positive("tof_pose_correction.update_hz", c.tof_pose_correction_update_hz);
    non_negative("tof_pose_correction.search_xy_m", c.tof_pose_correction_search_xy_m);
    positive("tof_pose_correction.search_xy_step_m", c.tof_pose_correction_search_xy_step_m);
    non_negative("tof_pose_correction.search_yaw_deg", c.tof_pose_correction_search_yaw_deg);
    positive("tof_pose_correction.search_yaw_step_deg", c.tof_pose_correction_search_yaw_step_deg);
    if (c.tof_pose_correction_min_valid_tof < 1 || c.tof_pose_correction_min_valid_tof > 3) errors.push_back("tof_pose_correction.min_valid_tof must be in [1, 3]");
    positive("tof_pose_correction.max_residual_m", c.tof_pose_correction_max_residual_m);
    non_negative("tof_pose_correction.min_margin", c.tof_pose_correction_min_margin);
    non_negative("tof_pose_correction.max_xy_m", c.tof_pose_correction_max_xy_m);
    non_negative("tof_pose_correction.max_yaw_deg", c.tof_pose_correction_max_yaw_deg);
    probability("tof_pose_correction.gain", c.tof_pose_correction_gain);
    if (c.tof_pose_correction_enabled && c.tof_pose_correction_mode == "off") errors.push_back("tof_pose_correction.enabled=true requires mode != off");
    if (c.tof_pose_correction_enabled && c.tof_pose_correction_mode != "score_only" && c.tof_pose_correction_mode != "yaw_candidate") errors.push_back("tof_pose_correction only supports mode=score_only or yaw_candidate in this version");
    if (c.tof_pose_correction_mapping_mode_apply) errors.push_back("tof_pose_correction.mapping_mode_apply must be false in score_only/yaw_candidate version");
    if (!(c.tof_pose_correction_candidate_lpf_alpha > 0.0) || c.tof_pose_correction_candidate_lpf_alpha > 1.0 || !std::isfinite(c.tof_pose_correction_candidate_lpf_alpha)) errors.push_back("tof_pose_correction.candidate_lpf_alpha must be in (0, 1]");
    if (c.tof_pose_correction_required_consistency < 1) errors.push_back("tof_pose_correction.required_consistency must be >= 1");
    positive("tof_pose_correction.consistency_yaw_deg", c.tof_pose_correction_consistency_yaw_deg);
    probability("tof_pose_correction.min_candidate_reliability", c.tof_pose_correction_min_candidate_reliability);
    non_negative("tof_pose_correction.min_best_second_margin", c.tof_pose_correction_min_best_second_margin);
    if (!std::isfinite(c.tof_pose_correction_debug_offset_x_m)) errors.push_back("tof_pose_correction.debug_pose_offset_x_m must be finite");
    if (!std::isfinite(c.tof_pose_correction_debug_offset_y_m)) errors.push_back("tof_pose_correction.debug_pose_offset_y_m must be finite");
    if (!std::isfinite(c.tof_pose_correction_debug_offset_yaw_deg)) errors.push_back("tof_pose_correction.debug_pose_offset_yaw_deg must be finite");

    if (!one_of(c.spin_scan_mode, {"observe_only"})) errors.push_back("spin_scan_localization.mode currently only supports observe_only");
    if (c.spin_scan_enabled && c.spin_scan_mode != "observe_only") errors.push_back("spin_scan_localization.enabled=true requires mode=observe_only");
    if (!(c.spin_scan_angle_bin_deg > 0.0) || c.spin_scan_angle_bin_deg > 30.0 || !std::isfinite(c.spin_scan_angle_bin_deg)) errors.push_back("spin_scan_localization.angle_bin_deg must be > 0 and <= 30");
    if (!(c.spin_scan_min_rotation_deg > 0.0) || c.spin_scan_min_rotation_deg > 360.0 || !std::isfinite(c.spin_scan_min_rotation_deg)) errors.push_back("spin_scan_localization.min_rotation_deg must be > 0 and <= 360");
    if (c.spin_scan_target_rotation_deg < c.spin_scan_min_rotation_deg || c.spin_scan_target_rotation_deg > 720.0 || !std::isfinite(c.spin_scan_target_rotation_deg)) errors.push_back("spin_scan_localization.target_rotation_deg must be >= min_rotation_deg and <= 720");
    positive("spin_scan_localization.max_duration_s", c.spin_scan_max_duration_s);
    non_negative("spin_scan_localization.max_linear_speed_mps", c.spin_scan_max_linear_speed_mps);
    positive("spin_scan_localization.angular_speed_min_radps", c.spin_scan_angular_speed_min_radps);
    if (!(c.spin_scan_angular_speed_max_radps > c.spin_scan_angular_speed_min_radps) || !std::isfinite(c.spin_scan_angular_speed_max_radps)) errors.push_back("spin_scan_localization.angular_speed_max_radps must be > angular_speed_min_radps");
    if (c.spin_scan_min_valid_bins < 1) errors.push_back("spin_scan_localization.min_valid_bins must be >= 1");
    probability("spin_scan_localization.min_valid_bin_ratio", c.spin_scan_min_valid_bin_ratio);
    positive("spin_scan_localization.match_search_yaw_deg", c.spin_scan_match_search_yaw_deg);
    positive("spin_scan_localization.match_search_yaw_step_deg", c.spin_scan_match_search_yaw_step_deg);
    non_negative("spin_scan_localization.min_best_second_margin", c.spin_scan_min_best_second_margin);
    non_negative("spin_scan_localization.flat_curve_sharpness_thresh", c.spin_scan_flat_curve_sharpness_thresh);
    positive("spin_scan_localization.multimodal_yaw_gap_deg", c.spin_scan_multimodal_yaw_gap_deg);
    non_negative("spin_scan_localization.multimodal_residual_window", c.spin_scan_multimodal_residual_window);
    probability("spin_scan_localization.min_reliability", c.spin_scan_min_reliability);

    positive("map_quality.log_hz", c.map_quality_log_hz);
    positive("map_quality.window_s", c.map_quality_window_s);
    probability("map_quality.min_tof_valid_ratio", c.map_quality_min_tof_valid_ratio);
    non_negative("map_quality.min_map_update_rate_hz", c.map_quality_min_map_update_rate_hz);
    if (c.map_quality_min_occupied_cells < 0) errors.push_back("map_quality.min_occupied_cells must be >= 0");
    probability("map_quality.low_quality_score_threshold", c.map_quality_low_quality_score_threshold);
    probability("map_quality.degraded_score_threshold", c.map_quality_degraded_score_threshold);
    if (c.map_quality_degraded_score_threshold > c.map_quality_low_quality_score_threshold) errors.push_back("map_quality.degraded_score_threshold must be <= low_quality_score_threshold");

    positive("mapping_supervisor.log_hz", c.mapping_supervisor_log_hz);
    non_negative("mapping_supervisor.startup_grace_s", c.mapping_supervisor_startup_grace_s);
    non_negative("mapping_supervisor.min_ready_time_s", c.mapping_supervisor_min_ready_time_s);
    probability("mapping_supervisor.min_good_quality_score", c.mapping_supervisor_min_good_quality_score);
    probability("mapping_supervisor.degraded_quality_score", c.mapping_supervisor_degraded_quality_score);
    probability("mapping_supervisor.lost_quality_score", c.mapping_supervisor_lost_quality_score);
    non_negative("mapping_supervisor.max_degraded_duration_s", c.mapping_supervisor_max_degraded_duration_s);
    non_negative("mapping_supervisor.max_no_data_duration_s", c.mapping_supervisor_max_no_data_duration_s);
    non_negative("mapping_supervisor.active_scan_after_low_quality_s", c.mapping_supervisor_active_scan_after_low_quality_s);
    non_negative("mapping_supervisor.active_scan_after_low_update_s", c.mapping_supervisor_active_scan_after_low_update_s);
    if (c.mapping_supervisor_encoder_error_degraded_threshold < 0) errors.push_back("mapping_supervisor.encoder_error_degraded_threshold must be >= 0");
    if (c.mapping_supervisor_imu_error_degraded_threshold < 0) errors.push_back("mapping_supervisor.imu_error_degraded_threshold must be >= 0");
    if (c.mapping_supervisor_tof_unhealthy_degraded_threshold < 0) errors.push_back("mapping_supervisor.tof_unhealthy_degraded_threshold must be >= 0");
    if (c.mapping_supervisor_lost_quality_score > c.mapping_supervisor_degraded_quality_score) errors.push_back("mapping_supervisor.lost_quality_score must be <= degraded_quality_score");
    if (c.mapping_supervisor_degraded_quality_score > c.mapping_supervisor_min_good_quality_score) errors.push_back("mapping_supervisor.degraded_quality_score must be <= min_good_quality_score");

    positive("active_scan.log_hz", c.active_scan_log_hz);
    non_negative("active_scan.min_interval_s", c.active_scan_min_interval_s);
    non_negative("active_scan.cooldown_s", c.active_scan_cooldown_s);
    non_negative("active_scan.request_hold_s", c.active_scan_request_hold_s);
    non_negative("active_scan.max_pending_s", c.active_scan_max_pending_s);
    non_negative("active_scan.recommended_scan_angle_deg", c.active_scan_recommended_scan_angle_deg);
    non_negative("active_scan.min_useful_scan_angle_deg", c.active_scan_min_useful_scan_angle_deg);
    non_negative("active_scan.complete_scan_angle_deg", c.active_scan_complete_scan_angle_deg);
    probability("active_scan.min_tof_valid_ratio_for_scan", c.active_scan_min_tof_valid_ratio_for_scan);
    if (c.active_scan_min_valid_tof_routes_for_scan < 0) errors.push_back("active_scan.min_valid_tof_routes_for_scan must be >= 0");
    non_negative("active_scan.max_linear_speed_for_scan_mps", c.active_scan_max_linear_speed_for_scan_mps);
    non_negative("active_scan.min_yaw_rate_for_observed_scan_dps", c.active_scan_min_yaw_rate_for_observed_scan_dps);
    non_negative("active_scan.max_yaw_rate_for_observed_scan_dps", c.active_scan_max_yaw_rate_for_observed_scan_dps);
    if (c.active_scan_complete_scan_angle_deg < c.active_scan_min_useful_scan_angle_deg) errors.push_back("active_scan.complete_scan_angle_deg must be >= min_useful_scan_angle_deg");
    if (c.active_scan_recommended_scan_angle_deg < c.active_scan_min_useful_scan_angle_deg) errors.push_back("active_scan.recommended_scan_angle_deg must be >= min_useful_scan_angle_deg");
    if (c.active_scan_max_yaw_rate_for_observed_scan_dps < c.active_scan_min_yaw_rate_for_observed_scan_dps) errors.push_back("active_scan.max_yaw_rate_for_observed_scan_dps must be >= min_yaw_rate_for_observed_scan_dps");

    if (!errors.empty()) throw std::runtime_error("invalid config: " + join_errors(errors));
}


void write_resolved_config(const Config &c, const std::string &path) {
    std::ofstream o(path);
    o << std::fixed << std::setprecision(6);
    o << "robot:\n"
      << "  wheel_base_m: " << c.wheel_base_m << "\n"
      << "  wheel_radius_left_m: " << c.wheel_radius_left_m << "\n"
      << "  wheel_radius_right_m: " << c.wheel_radius_right_m << "\n"
      << "  encoder_ticks_per_rev: " << c.encoder_ticks_per_rev << "\n";
    o << "encoder:\n"
      << "  source: " << c.encoder_source << "\n"
      << "  mode: " << c.encoder_source << "\n"
      << "  path: " << c.encoder_path << "\n"
      << "  left_port: " << c.encoder_left_port << "\n"
      << "  right_port: " << c.encoder_right_port << "\n"
      << "  baudrate: " << c.encoder_baudrate << "\n"
      << "  uart_timeout_ms: " << c.encoder_uart_timeout_ms << "\n"
      << "  left_id: " << c.encoder_left_id << "\n"
      << "  right_id: " << c.encoder_right_id << "\n"
      << "  counts_per_rev: " << c.encoder_ticks_per_rev << "\n"
      << "  read_hz: " << c.encoder_read_hz << "\n"
      << "  position_unwrap: " << bool_yaml(c.encoder_position_unwrap) << "\n"
      << "  speed_diagnostic: " << bool_yaml(c.encoder_speed_diagnostic) << "\n"
      << "  left_sign: " << c.encoder_left_sign << "\n"
      << "  right_sign: " << c.encoder_right_sign << "\n"
      << "  consecutive_error_limit: " << c.encoder_consecutive_error_limit << "\n";
    o << "imu:\n"
      << "  source: " << c.imu_source << "\n"
      << "  path: " << c.imu_path << "\n"
      << "  mode: " << c.imu_mode << "\n"
      << "  gyro_axis: " << c.imu_gyro_axis << "\n"
      << "  gyro_sign: " << c.imu_gyro_sign << "\n"
      << "  icm43600_lib_path: " << c.imu_icm43600_lib_path << "\n"
      << "  icm43600_device_no: " << c.imu_icm43600_device_no << "\n"
      << "  icm43600_accel_rate_hz: " << c.imu_icm43600_accel_rate_hz << "\n"
      << "  icm43600_gyro_rate_hz: " << c.imu_icm43600_gyro_rate_hz << "\n"
      << "  icm43600_batch_timeout_ms: " << c.imu_icm43600_batch_timeout_ms << "\n"
      << "  icm43600_accel_fsr: " << c.imu_icm43600_accel_fsr << "\n"
      << "  icm43600_gyro_fsr: " << c.imu_icm43600_gyro_fsr << "\n"
      << "  icm43600_high_res_fifo: " << bool_yaml(c.imu_icm43600_high_res_fifo) << "\n"
      << "  gyro_bias_static_calib_s: " << c.gyro_bias_static_calib_s << "\n"
      << "  static_gyro_max_abs_rad_s: " << c.static_gyro_max_abs_rad_s << "\n"
      << "  yaw_fusion_alpha: " << c.yaw_fusion_alpha << "\n";
    o << "localization:\n"
      << "  pose_source: encoder_imu_odometry\n"
      << "  wheel_base_m: " << c.wheel_base_m << "\n"
      << "  wheel_radius_left_m: " << c.wheel_radius_left_m << "\n"
      << "  wheel_radius_right_m: " << c.wheel_radius_right_m << "\n"
      << "  encoder_ticks_per_rev: " << c.encoder_ticks_per_rev << "\n"
      << "  encoder_source: " << c.encoder_source << "\n"
      << "  encoder_path: " << c.encoder_path << "\n"
      << "  encoder_left_port: " << c.encoder_left_port << "\n"
      << "  encoder_right_port: " << c.encoder_right_port << "\n"
      << "  encoder_baudrate: " << c.encoder_baudrate << "\n"
      << "  encoder_uart_timeout_ms: " << c.encoder_uart_timeout_ms << "\n"
      << "  encoder_left_id: " << c.encoder_left_id << "\n"
      << "  encoder_right_id: " << c.encoder_right_id << "\n"
      << "  encoder_counts_per_rev: " << c.encoder_ticks_per_rev << "\n"
      << "  encoder_read_hz: " << c.encoder_read_hz << "\n"
      << "  encoder_position_unwrap: " << bool_yaml(c.encoder_position_unwrap) << "\n"
      << "  encoder_speed_diagnostic: " << bool_yaml(c.encoder_speed_diagnostic) << "\n"
      << "  encoder_left_sign: " << c.encoder_left_sign << "\n"
      << "  encoder_right_sign: " << c.encoder_right_sign << "\n"
      << "  encoder_consecutive_error_limit: " << c.encoder_consecutive_error_limit << "\n"
      << "  imu_source: " << c.imu_source << "\n"
      << "  imu_path: " << c.imu_path << "\n"
      << "  imu_mode: " << c.imu_mode << "\n"
      << "  imu_gyro_axis: " << c.imu_gyro_axis << "\n"
      << "  imu_gyro_sign: " << c.imu_gyro_sign << "\n"
      << "  imu_icm43600_lib_path: " << c.imu_icm43600_lib_path << "\n"
      << "  imu_icm43600_device_no: " << c.imu_icm43600_device_no << "\n"
      << "  imu_icm43600_accel_rate_hz: " << c.imu_icm43600_accel_rate_hz << "\n"
      << "  imu_icm43600_gyro_rate_hz: " << c.imu_icm43600_gyro_rate_hz << "\n"
      << "  imu_icm43600_batch_timeout_ms: " << c.imu_icm43600_batch_timeout_ms << "\n"
      << "  imu_icm43600_accel_fsr: " << c.imu_icm43600_accel_fsr << "\n"
      << "  imu_icm43600_gyro_fsr: " << c.imu_icm43600_gyro_fsr << "\n"
      << "  imu_icm43600_high_res_fifo: " << bool_yaml(c.imu_icm43600_high_res_fifo) << "\n"
      << "  gyro_bias_static_calib_s: " << c.gyro_bias_static_calib_s << "\n"
      << "  static_gyro_max_abs_rad_s: " << c.static_gyro_max_abs_rad_s << "\n"
      << "  yaw_fusion_alpha: " << c.yaw_fusion_alpha << "\n"
      << "  encoder_max_wheel_speed_mps: " << c.encoder_max_wheel_speed_mps << "\n"
      << "  gyro_lpf_alpha: " << c.gyro_lpf_alpha << "\n"
      << "  gyro_spike_radps: " << c.gyro_spike_radps << "\n"
      << "  stationary_linear_speed_mps: " << c.stationary_linear_speed_mps << "\n"
      << "  stationary_angular_speed_radps: " << c.stationary_angular_speed_radps << "\n"
      << "  gyro_bias_adapt_alpha: " << c.gyro_bias_adapt_alpha << "\n";
    o << "tof:\n"
      << "  source: " << c.tof_source << "\n"
      << "  csv_path: " << c.tof_csv_path << "\n"
      << "  frequency_hz: " << c.tof_frequency_hz << "\n"
      << "  min_range_m: " << c.tof_min_range_m << "\n"
      << "  max_range_m: " << c.tof_max_range_m << "\n"
      << "  confidence_min: " << c.confidence_min << "\n"
      << "  median_window: " << c.median_window << "\n"
      << "  jump_reject_m: " << c.jump_reject_m << "\n"
      << "  mad_reject_m: " << c.mad_reject_m << "\n"
      << "  stable_hits_required: " << c.stable_hits_required << "\n";
    o << "tof_devices:\n";
    for (auto &id : {"front", "left", "right"}) o << "  " << id << ": " << c.tof_devices.at(id) << "\n";
    o << "tof_extrinsics:\n";
    for (auto &id : {"front", "left", "right"}) {
        const auto &e = c.tof_extrinsics.at(id);
        o << "  " << id << ".x_m: " << e.x_m << "\n  " << id << ".y_m: " << e.y_m
          << "\n  " << id << ".yaw_deg: " << e.yaw_deg << "\n  " << id << ".fov_deg: " << e.fov_deg << "\n";
    }
    o << "mapping:\n"
      << "  resolution_m: " << c.resolution_m << "\n"
      << "  chunk_cells: " << c.chunk_cells << "\n"
      << "  # initial_width_m/initial_height_m are recorded for compatibility; chunked grid grows from touched cells in first version.\n"
      << "  initial_width_m: " << c.initial_width_m << "\n"
      << "  initial_height_m: " << c.initial_height_m << "\n"
      << "  ray_step_deg: " << c.ray_step_deg << "\n"
      << "  hit_thickness_m: " << c.hit_thickness_m << "\n"
      << "  log_odds_occ: " << c.log_odds_occ << "\n"
      << "  log_odds_free: " << c.log_odds_free << "\n"
      << "  log_odds_min: " << c.log_odds_min << "\n"
      << "  log_odds_max: " << c.log_odds_max << "\n"
      << "  occupied_thresh: " << c.occupied_thresh << "\n"
      << "  free_thresh: " << c.free_thresh << "\n"
      << "  max_cells_per_tof_update: " << c.max_cells_per_tof_update << "\n"
      << "  static_scan_stable_required: " << c.static_scan_stable_required << "\n"
      << "  static_scan_boost: " << c.static_scan_boost << "\n";
    o << "runtime:\n"
      << "  localization_hz: " << c.localization_hz << "\n"
      << "  tof_read_hz: " << c.tof_read_hz << "\n"
      << "  mapping_hz: " << c.mapping_hz << "\n"
      << "  log_hz: " << c.log_hz << "\n";
    o << "runtime_limits:\n"
      << "  max_linear_speed_mps: " << c.max_linear_speed_mps << "\n"
      << "  pause_linear_speed_mps: " << c.pause_linear_speed_mps << "\n"
      << "  max_angular_speed_radps: " << c.max_angular_speed_radps << "\n"
      << "  pause_angular_speed_radps: " << c.pause_angular_speed_radps << "\n";
    o << "logging:\n"
      << "  output_dir: " << c.output_dir << "\n";
    o << "map_saver:\n"
      << "  save_on_exit: " << bool_yaml(c.save_on_exit) << "\n"
      << "  autosave_period_s: " << c.autosave_period_s << "\n";
    o << "map_quality:\n"
      << "  enabled: " << bool_yaml(c.map_quality_enabled) << "\n"
      << "  log_hz: " << c.map_quality_log_hz << "\n"
      << "  min_tof_valid_ratio: " << c.map_quality_min_tof_valid_ratio << "\n"
      << "  min_map_update_rate_hz: " << c.map_quality_min_map_update_rate_hz << "\n"
      << "  min_occupied_cells: " << c.map_quality_min_occupied_cells << "\n"
      << "  low_quality_score_threshold: " << c.map_quality_low_quality_score_threshold << "\n"
      << "  degraded_score_threshold: " << c.map_quality_degraded_score_threshold << "\n"
      << "  window_s: " << c.map_quality_window_s << "\n";
    o << "mapping_supervisor:\n"
      << "  enabled: " << bool_yaml(c.mapping_supervisor_enabled) << "\n"
      << "  log_hz: " << c.mapping_supervisor_log_hz << "\n"
      << "  startup_grace_s: " << c.mapping_supervisor_startup_grace_s << "\n"
      << "  min_ready_time_s: " << c.mapping_supervisor_min_ready_time_s << "\n"
      << "  min_good_quality_score: " << c.mapping_supervisor_min_good_quality_score << "\n"
      << "  degraded_quality_score: " << c.mapping_supervisor_degraded_quality_score << "\n"
      << "  lost_quality_score: " << c.mapping_supervisor_lost_quality_score << "\n"
      << "  max_degraded_duration_s: " << c.mapping_supervisor_max_degraded_duration_s << "\n"
      << "  max_no_data_duration_s: " << c.mapping_supervisor_max_no_data_duration_s << "\n"
      << "  require_two_tof_routes: " << bool_yaml(c.mapping_supervisor_require_two_tof_routes) << "\n"
      << "  active_scan_after_low_quality_s: " << c.mapping_supervisor_active_scan_after_low_quality_s << "\n"
      << "  active_scan_after_low_update_s: " << c.mapping_supervisor_active_scan_after_low_update_s << "\n"
      << "  encoder_error_degraded_threshold: " << c.mapping_supervisor_encoder_error_degraded_threshold << "\n"
      << "  imu_error_degraded_threshold: " << c.mapping_supervisor_imu_error_degraded_threshold << "\n"
      << "  tof_unhealthy_degraded_threshold: " << c.mapping_supervisor_tof_unhealthy_degraded_threshold << "\n";
    o << "active_scan:\n"
      << "  enabled: " << bool_yaml(c.active_scan_enabled) << "\n"
      << "  log_hz: " << c.active_scan_log_hz << "\n"
      << "  startup_scan_enabled: " << bool_yaml(c.active_scan_startup_scan_enabled) << "\n"
      << "  low_quality_scan_enabled: " << bool_yaml(c.active_scan_low_quality_scan_enabled) << "\n"
      << "  degraded_scan_enabled: " << bool_yaml(c.active_scan_degraded_scan_enabled) << "\n"
      << "  lost_scan_enabled: " << bool_yaml(c.active_scan_lost_scan_enabled) << "\n"
      << "  min_interval_s: " << c.active_scan_min_interval_s << "\n"
      << "  cooldown_s: " << c.active_scan_cooldown_s << "\n"
      << "  request_hold_s: " << c.active_scan_request_hold_s << "\n"
      << "  max_pending_s: " << c.active_scan_max_pending_s << "\n"
      << "  recommended_scan_angle_deg: " << c.active_scan_recommended_scan_angle_deg << "\n"
      << "  min_useful_scan_angle_deg: " << c.active_scan_min_useful_scan_angle_deg << "\n"
      << "  complete_scan_angle_deg: " << c.active_scan_complete_scan_angle_deg << "\n"
      << "  min_tof_valid_ratio_for_scan: " << c.active_scan_min_tof_valid_ratio_for_scan << "\n"
      << "  min_valid_tof_routes_for_scan: " << c.active_scan_min_valid_tof_routes_for_scan << "\n"
      << "  max_linear_speed_for_scan_mps: " << c.active_scan_max_linear_speed_for_scan_mps << "\n"
      << "  min_yaw_rate_for_observed_scan_dps: " << c.active_scan_min_yaw_rate_for_observed_scan_dps << "\n"
      << "  max_yaw_rate_for_observed_scan_dps: " << c.active_scan_max_yaw_rate_for_observed_scan_dps << "\n"
      << "  observe_natural_rotation: " << bool_yaml(c.active_scan_observe_natural_rotation) << "\n"
      << "  require_supervisor_recommendation: " << bool_yaml(c.active_scan_require_supervisor_recommendation) << "\n";
    o << "tof_pose_correction:\n"
      << "  enabled: " << bool_yaml(c.tof_pose_correction_enabled) << "\n"
      << "  mode: " << c.tof_pose_correction_mode << "\n"
      << "  update_hz: " << c.tof_pose_correction_update_hz << "\n"
      << "  search_xy_m: " << c.tof_pose_correction_search_xy_m << "\n"
      << "  search_xy_step_m: " << c.tof_pose_correction_search_xy_step_m << "\n"
      << "  search_yaw_deg: " << c.tof_pose_correction_search_yaw_deg << "\n"
      << "  search_yaw_step_deg: " << c.tof_pose_correction_search_yaw_step_deg << "\n"
      << "  min_valid_tof: " << c.tof_pose_correction_min_valid_tof << "\n"
      << "  max_residual_m: " << c.tof_pose_correction_max_residual_m << "\n"
      << "  min_margin: " << c.tof_pose_correction_min_margin << "\n"
      << "  max_xy_m: " << c.tof_pose_correction_max_xy_m << "\n"
      << "  max_yaw_deg: " << c.tof_pose_correction_max_yaw_deg << "\n"
      << "  gain: " << c.tof_pose_correction_gain << "\n"
      << "  mapping_mode_apply: " << bool_yaml(c.tof_pose_correction_mapping_mode_apply) << "\n"
      << "  candidate_lpf_alpha: " << c.tof_pose_correction_candidate_lpf_alpha << "\n"
      << "  required_consistency: " << c.tof_pose_correction_required_consistency << "\n"
      << "  consistency_yaw_deg: " << c.tof_pose_correction_consistency_yaw_deg << "\n"
      << "  min_candidate_reliability: " << c.tof_pose_correction_min_candidate_reliability << "\n"
      << "  min_best_second_margin: " << c.tof_pose_correction_min_best_second_margin << "\n"
      << "  yaw_residual_curve_debug: " << bool_yaml(c.tof_pose_correction_yaw_residual_curve_debug) << "\n"
      << "  candidate_search_xy_disabled: " << bool_yaml(c.tof_pose_correction_mode == "yaw_candidate") << "\n"
      << "  debug_pose_offset_x_m: " << c.tof_pose_correction_debug_offset_x_m << "\n"
      << "  debug_pose_offset_y_m: " << c.tof_pose_correction_debug_offset_y_m << "\n"
      << "  debug_pose_offset_yaw_deg: " << c.tof_pose_correction_debug_offset_yaw_deg << "\n";
    o << "spin_scan_localization:\n"
      << "  enabled: " << bool_yaml(c.spin_scan_enabled) << "\n"
      << "  mode: " << c.spin_scan_mode << "\n"
      << "  min_rotation_deg: " << c.spin_scan_min_rotation_deg << "\n"
      << "  target_rotation_deg: " << c.spin_scan_target_rotation_deg << "\n"
      << "  max_duration_s: " << c.spin_scan_max_duration_s << "\n"
      << "  max_linear_speed_mps: " << c.spin_scan_max_linear_speed_mps << "\n"
      << "  angular_speed_min_radps: " << c.spin_scan_angular_speed_min_radps << "\n"
      << "  angular_speed_max_radps: " << c.spin_scan_angular_speed_max_radps << "\n"
      << "  angle_bin_deg: " << c.spin_scan_angle_bin_deg << "\n"
      << "  min_valid_bins: " << c.spin_scan_min_valid_bins << "\n"
      << "  min_valid_bin_ratio: " << c.spin_scan_min_valid_bin_ratio << "\n"
      << "  match_search_yaw_deg: " << c.spin_scan_match_search_yaw_deg << "\n"
      << "  match_search_yaw_step_deg: " << c.spin_scan_match_search_yaw_step_deg << "\n"
      << "  min_best_second_margin: " << c.spin_scan_min_best_second_margin << "\n"
      << "  flat_curve_sharpness_thresh: " << c.spin_scan_flat_curve_sharpness_thresh << "\n"
      << "  multimodal_yaw_gap_deg: " << c.spin_scan_multimodal_yaw_gap_deg << "\n"
      << "  multimodal_residual_window: " << c.spin_scan_multimodal_residual_window << "\n"
      << "  min_reliability: " << c.spin_scan_min_reliability << "\n"
      << "  debug_curve: " << bool_yaml(c.spin_scan_debug_curve) << "\n";
}

} // namespace robot_slamd
