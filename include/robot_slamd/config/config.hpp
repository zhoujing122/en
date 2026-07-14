#pragma once
#include "robot_slamd/core/common.hpp"

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
    std::string subsection;
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
            subsection.clear();
            continue;
        }
        if (indent > 0 && val.empty()) {
            subsection = key;
            continue;
        }
        std::string full;
        if (section.empty()) full = key;
        else if (!subsection.empty() && indent > 2) full = section + "." + subsection + "." + key;
        else {
            if (indent <= 2) subsection.clear();
            full = section + "." + key;
        }
        kv[full] = strip_quotes(val);
    }
    return kv;
}

#include "robot_slamd/config/config_motion_execution.hpp"

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
    c.encoder_max_pair_skew_us = get_double(kv, "encoder.max_pair_skew_us", c.encoder_max_pair_skew_us);
    c.encoder_max_read_latency_us = get_double(kv, "encoder.max_read_latency_us", c.encoder_max_read_latency_us);
    c.encoder_require_status_zero = get_bool(kv, "encoder.require_status_zero", c.encoder_require_status_zero);
    c.encoder_require_pair_skew_ok_for_odometry = get_bool(kv, "encoder.require_pair_skew_ok_for_odometry", c.encoder_require_pair_skew_ok_for_odometry);
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
    c.tof_protocol_min_range_m = get_double(kv, "tof.protocol_min_range_m", c.tof_protocol_min_range_m);
    c.tof_protocol_max_range_m = get_double(kv, "tof.protocol_max_range_m", c.tof_protocol_max_range_m);
    c.tof_mapping_min_range_m = get_double(kv, "tof.mapping_min_range_m", c.tof_mapping_min_range_m);
    c.tof_mapping_max_range_m = get_double(kv, "tof.mapping_max_range_m", c.tof_mapping_max_range_m);
    c.confidence_min = get_int(kv, "tof.confidence_min", c.confidence_min);
    c.tof_mapping_min_confidence = get_int(kv, "tof.mapping_min_confidence", c.tof_mapping_min_confidence);
    c.tof_full_fov_deg = get_double(kv, "tof.full_fov_deg", c.tof_full_fov_deg);
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
    c.slam_runtime_mode = get_string(kv, "runtime.slam_runtime_mode", c.slam_runtime_mode);
    c.slam_runtime_mode = get_string(kv, "slam_runtime.mode", c.slam_runtime_mode);
    c.sparse_shadow_sensor_source = get_string(kv, "runtime.sparse_shadow_sensor_source", c.sparse_shadow_sensor_source);
    c.sparse_shadow_sensor_source = get_string(kv, "slam_runtime.sparse_shadow_sensor_source", c.sparse_shadow_sensor_source);
    c.sparse_slam_map_startup_mode = get_string(kv, "sparse_slam.map_startup_mode", c.sparse_slam_map_startup_mode);
    c.sparse_slam_initial_pose_mode = get_string(kv, "sparse_slam.initial_pose_mode", c.sparse_slam_initial_pose_mode);
    c.sparse_slam_has_configured_pose = get_bool(kv, "sparse_slam.has_configured_pose", c.sparse_slam_has_configured_pose);
    c.sparse_slam_configured_pose_x_m = get_double(kv, "sparse_slam.configured_pose_x_m", c.sparse_slam_configured_pose_x_m);
    c.sparse_slam_configured_pose_y_m = get_double(kv, "sparse_slam.configured_pose_y_m", c.sparse_slam_configured_pose_y_m);
    c.sparse_slam_configured_pose_yaw_rad = get_double(kv, "sparse_slam.configured_pose_yaw_rad", c.sparse_slam_configured_pose_yaw_rad);
    c.sparse_slam_pose_buffer_capacity = get_int(kv, "sparse_slam.pose_buffer_capacity", c.sparse_slam_pose_buffer_capacity);
    c.sparse_slam_pose_buffer_max_age_s = get_double(kv, "sparse_slam.pose_buffer_max_age_s", c.sparse_slam_pose_buffer_max_age_s);
    c.sparse_slam_pose_interpolation_max_gap_s = get_double(kv, "sparse_slam.pose_interpolation_max_gap_s", c.sparse_slam_pose_interpolation_max_gap_s);
    c.sparse_slam_require_all_measurement_poses = get_bool(kv, "sparse_slam.require_all_measurement_poses", c.sparse_slam_require_all_measurement_poses);
    c.sparse_slam_startup_min_imu_samples = get_int(kv, "sparse_slam.startup_min_imu_samples", c.sparse_slam_startup_min_imu_samples);
    c.sparse_slam_startup_max_abs_linear_speed_mps = get_double(kv, "sparse_slam.startup_max_abs_linear_speed_mps", c.sparse_slam_startup_max_abs_linear_speed_mps);
    c.sparse_slam_startup_max_abs_wheel_yaw_rate_rad_s = get_double(kv, "sparse_slam.startup_max_abs_wheel_yaw_rate_rad_s", c.sparse_slam_startup_max_abs_wheel_yaw_rate_rad_s);
    c.sparse_slam_startup_max_abs_imu_yaw_rate_rad_s = get_double(kv, "sparse_slam.startup_max_abs_imu_yaw_rate_rad_s", c.sparse_slam_startup_max_abs_imu_yaw_rate_rad_s);
    c.sparse_slam_startup_max_gyro_bias_abs_rad_s = get_double(kv, "sparse_slam.startup_max_gyro_bias_abs_rad_s", c.sparse_slam_startup_max_gyro_bias_abs_rad_s);
    c.sparse_slam_startup_max_gyro_spread_rad_s = get_double(kv, "sparse_slam.startup_max_gyro_spread_rad_s", c.sparse_slam_startup_max_gyro_spread_rad_s);
    c.sparse_slam_active_bundle_max_snapshots = get_int(kv, "sparse_slam.active_bundle_max_snapshots", c.sparse_slam_active_bundle_max_snapshots);
    c.sparse_slam_active_bundle_max_rays = get_int(kv, "sparse_slam.active_bundle_max_rays", c.sparse_slam_active_bundle_max_rays);
    c.sparse_slam_active_bundle_max_duration_s = get_double(kv, "sparse_slam.active_bundle_max_duration_s", c.sparse_slam_active_bundle_max_duration_s);
    c.sparse_slam_active_bundle_max_snapshot_gap_s = get_double(kv, "sparse_slam.active_bundle_max_snapshot_gap_s", c.sparse_slam_active_bundle_max_snapshot_gap_s);
    c.sparse_slam_active_bundle_min_snapshots = get_int(kv, "sparse_slam.active_bundle_min_snapshots", c.sparse_slam_active_bundle_min_snapshots);
    c.sparse_slam_active_bundle_min_matchable_rays = get_int(kv, "sparse_slam.active_bundle_min_matchable_rays", c.sparse_slam_active_bundle_min_matchable_rays);
    c.sparse_slam_active_bundle_min_yaw_span_rad = get_double(kv, "sparse_slam.active_bundle_min_yaw_span_rad", c.sparse_slam_active_bundle_min_yaw_span_rad);
    c.sparse_slam_active_bundle_require_all_measurement_poses = get_bool(kv, "sparse_slam.active_bundle_require_all_measurement_poses", c.sparse_slam_active_bundle_require_all_measurement_poses);
    c.sparse_slam_settle_max_abs_linear_speed_mps = get_double(kv, "sparse_slam.settle_max_abs_linear_speed_mps", c.sparse_slam_settle_max_abs_linear_speed_mps);
    c.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s = get_double(kv, "sparse_slam.settle_max_abs_wheel_yaw_rate_rad_s", c.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s);
    c.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s = get_double(kv, "sparse_slam.settle_max_abs_imu_yaw_rate_rad_s", c.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s);
    c.sparse_slam_settle_min_consecutive_samples = get_int(kv, "sparse_slam.settle_min_consecutive_samples", c.sparse_slam_settle_min_consecutive_samples);
    c.sparse_slam_settle_min_stable_duration_s = get_double(kv, "sparse_slam.settle_min_stable_duration_s", c.sparse_slam_settle_min_stable_duration_s);
    c.sparse_slam_settle_max_sample_gap_s = get_double(kv, "sparse_slam.settle_max_sample_gap_s", c.sparse_slam_settle_max_sample_gap_s);
    c.sparse_slam_reference_snapshot_max_cells = get_int(kv, "sparse_slam.reference_snapshot_max_cells", c.sparse_slam_reference_snapshot_max_cells);
    c.sparse_slam_local_match_mode = get_string(kv, "sparse_slam.local_match_mode", c.sparse_slam_local_match_mode);
    c.sparse_slam_local_match_max_abs_yaw_rad = get_double(kv, "sparse_slam.local_match_max_abs_yaw_rad", c.sparse_slam_local_match_max_abs_yaw_rad);
    c.sparse_slam_local_match_coarse_yaw_step_rad = get_double(kv, "sparse_slam.local_match_coarse_yaw_step_rad", c.sparse_slam_local_match_coarse_yaw_step_rad);
    c.sparse_slam_local_match_fine_yaw_step_rad = get_double(kv, "sparse_slam.local_match_fine_yaw_step_rad", c.sparse_slam_local_match_fine_yaw_step_rad);
    c.sparse_slam_local_match_max_abs_translation_x_m = get_double(kv, "sparse_slam.local_match_max_abs_translation_x_m", c.sparse_slam_local_match_max_abs_translation_x_m);
    c.sparse_slam_local_match_max_abs_translation_y_m = get_double(kv, "sparse_slam.local_match_max_abs_translation_y_m", c.sparse_slam_local_match_max_abs_translation_y_m);
    c.sparse_slam_local_match_max_candidate_count = get_int(kv, "sparse_slam.local_match_max_candidate_count", c.sparse_slam_local_match_max_candidate_count);
    c.sparse_slam_local_match_min_bundle_frames = get_int(kv, "sparse_slam.local_match_min_bundle_frames", c.sparse_slam_local_match_min_bundle_frames);
    c.sparse_slam_local_match_min_matchable_rays = get_int(kv, "sparse_slam.local_match_min_matchable_rays", c.sparse_slam_local_match_min_matchable_rays);
    c.sparse_slam_local_match_min_reference_cells = get_int(kv, "sparse_slam.local_match_min_reference_cells", c.sparse_slam_local_match_min_reference_cells);
    c.sparse_slam_local_match_min_reference_occupied_cells = get_int(kv, "sparse_slam.local_match_min_reference_occupied_cells", c.sparse_slam_local_match_min_reference_occupied_cells);
    c.sparse_slam_local_match_min_reference_free_cells = get_int(kv, "sparse_slam.local_match_min_reference_free_cells", c.sparse_slam_local_match_min_reference_free_cells);
    c.sparse_slam_local_match_require_revision_match = get_bool(kv, "sparse_slam.local_match_require_revision_match", c.sparse_slam_local_match_require_revision_match);
    c.sparse_slam_local_match_require_immutable_snapshot = get_bool(kv, "sparse_slam.local_match_require_immutable_snapshot", c.sparse_slam_local_match_require_immutable_snapshot);
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
    c.active_scan_execution_enabled = get_bool(kv, "active_scan_execution.enabled", c.active_scan_execution_enabled);
    c.active_scan_execution_mode = get_string(kv, "active_scan_execution.mode", c.active_scan_execution_mode);
    c.active_scan_execution_log_hz = get_double(kv, "active_scan_execution.log_hz", c.active_scan_execution_log_hz);
    c.active_scan_execution_require_active_scan_would_start = get_bool(kv, "active_scan_execution.require_active_scan_would_start", c.active_scan_execution_require_active_scan_would_start);
    c.active_scan_execution_require_scan_recommended = get_bool(kv, "active_scan_execution.require_scan_recommended", c.active_scan_execution_require_scan_recommended);
    c.active_scan_execution_require_localization_valid = get_bool(kv, "active_scan_execution.require_localization_valid", c.active_scan_execution_require_localization_valid);
    c.active_scan_execution_target_scan_angle_deg = get_double(kv, "active_scan_execution.target_scan_angle_deg", c.active_scan_execution_target_scan_angle_deg);
    c.active_scan_execution_complete_scan_angle_deg = get_double(kv, "active_scan_execution.complete_scan_angle_deg", c.active_scan_execution_complete_scan_angle_deg);
    c.active_scan_execution_min_useful_scan_angle_deg = get_double(kv, "active_scan_execution.min_useful_scan_angle_deg", c.active_scan_execution_min_useful_scan_angle_deg);
    c.active_scan_execution_target_yaw_rate_dps = get_double(kv, "active_scan_execution.target_yaw_rate_dps", c.active_scan_execution_target_yaw_rate_dps);
    c.active_scan_execution_min_observed_yaw_rate_dps = get_double(kv, "active_scan_execution.min_observed_yaw_rate_dps", c.active_scan_execution_min_observed_yaw_rate_dps);
    c.active_scan_execution_max_observed_yaw_rate_dps = get_double(kv, "active_scan_execution.max_observed_yaw_rate_dps", c.active_scan_execution_max_observed_yaw_rate_dps);
    c.active_scan_execution_max_linear_speed_mps = get_double(kv, "active_scan_execution.max_linear_speed_mps", c.active_scan_execution_max_linear_speed_mps);
    c.active_scan_execution_precheck_hold_s = get_double(kv, "active_scan_execution.precheck_hold_s", c.active_scan_execution_precheck_hold_s);
    c.active_scan_execution_command_timeout_s = get_double(kv, "active_scan_execution.command_timeout_s", c.active_scan_execution_command_timeout_s);
    c.active_scan_execution_cooldown_s = get_double(kv, "active_scan_execution.cooldown_s", c.active_scan_execution_cooldown_s);
    c.active_scan_execution_emit_zero_command_on_stop = get_bool(kv, "active_scan_execution.emit_zero_command_on_stop", c.active_scan_execution_emit_zero_command_on_stop);
    c.active_scan_execution_command_log_only = get_bool(kv, "active_scan_execution.command_log_only", c.active_scan_execution_command_log_only);
    c.active_scan_execution_hardware_write_enabled = get_bool(kv, "active_scan_execution.hardware_write_enabled", c.active_scan_execution_hardware_write_enabled);
    c.sparse_scan_enabled = get_bool(kv, "sparse_scan.enabled", c.sparse_scan_enabled);
    c.sparse_scan_log_hz = get_double(kv, "sparse_scan.log_hz", c.sparse_scan_log_hz);
    c.sparse_scan_collect_on_active_scan = get_bool(kv, "sparse_scan.collect_on_active_scan", c.sparse_scan_collect_on_active_scan);
    c.sparse_scan_collect_on_command_verifying = get_bool(kv, "sparse_scan.collect_on_command_verifying", c.sparse_scan_collect_on_command_verifying);
    c.sparse_scan_collect_on_natural_rotation = get_bool(kv, "sparse_scan.collect_on_natural_rotation", c.sparse_scan_collect_on_natural_rotation);
    c.sparse_scan_require_active_scan_recommended = get_bool(kv, "sparse_scan.require_active_scan_recommended", c.sparse_scan_require_active_scan_recommended);
    c.sparse_scan_require_mapping_allowed = get_bool(kv, "sparse_scan.require_mapping_allowed", c.sparse_scan_require_mapping_allowed);
    c.sparse_scan_min_yaw_rate_dps = get_double(kv, "sparse_scan.min_yaw_rate_dps", c.sparse_scan_min_yaw_rate_dps);
    c.sparse_scan_max_yaw_rate_dps = get_double(kv, "sparse_scan.max_yaw_rate_dps", c.sparse_scan_max_yaw_rate_dps);
    c.sparse_scan_max_linear_speed_mps = get_double(kv, "sparse_scan.max_linear_speed_mps", c.sparse_scan_max_linear_speed_mps);
    c.sparse_scan_min_range_m = get_double(kv, "sparse_scan.min_range_m", c.sparse_scan_min_range_m);
    c.sparse_scan_max_range_m = get_double(kv, "sparse_scan.max_range_m", c.sparse_scan_max_range_m);
    c.sparse_scan_min_confidence = get_int(kv, "sparse_scan.min_confidence", c.sparse_scan_min_confidence);
    c.sparse_scan_bin_angle_deg = get_double(kv, "sparse_scan.bin_angle_deg", c.sparse_scan_bin_angle_deg);
    c.sparse_scan_min_session_angle_deg = get_double(kv, "sparse_scan.min_session_angle_deg", c.sparse_scan_min_session_angle_deg);
    c.sparse_scan_useful_session_angle_deg = get_double(kv, "sparse_scan.useful_session_angle_deg", c.sparse_scan_useful_session_angle_deg);
    c.sparse_scan_complete_session_angle_deg = get_double(kv, "sparse_scan.complete_session_angle_deg", c.sparse_scan_complete_session_angle_deg);
    c.sparse_scan_session_timeout_s = get_double(kv, "sparse_scan.session_timeout_s", c.sparse_scan_session_timeout_s);
    c.sparse_scan_max_gap_s = get_double(kv, "sparse_scan.max_gap_s", c.sparse_scan_max_gap_s);
    c.sparse_scan_cooldown_s = get_double(kv, "sparse_scan.cooldown_s", c.sparse_scan_cooldown_s);
    c.sparse_scan_keep_invalid_samples = get_bool(kv, "sparse_scan.keep_invalid_samples", c.sparse_scan_keep_invalid_samples);
    c.sparse_scan_compute_hit_points = get_bool(kv, "sparse_scan.compute_hit_points", c.sparse_scan_compute_hit_points);
    c.sparse_scan_write_sample_log = get_bool(kv, "sparse_scan.write_sample_log", c.sparse_scan_write_sample_log);
    c.sparse_scan_write_bin_log = get_bool(kv, "sparse_scan.write_bin_log", c.sparse_scan_write_bin_log);
    c.sparse_scan_write_session_log = get_bool(kv, "sparse_scan.write_session_log", c.sparse_scan_write_session_log);
    c.sparse_scan_max_samples_per_session = get_int(kv, "sparse_scan.max_samples_per_session", c.sparse_scan_max_samples_per_session);
    c.sparse_scan_yaw_match_enabled = get_bool(kv, "sparse_scan_yaw_match.enabled", c.sparse_scan_yaw_match_enabled);
    c.sparse_scan_yaw_match_log_hz = get_double(kv, "sparse_scan_yaw_match.log_hz", c.sparse_scan_yaw_match_log_hz);
    c.sparse_scan_yaw_match_run_on_completed_sparse_scan = get_bool(kv, "sparse_scan_yaw_match.run_on_completed_sparse_scan", c.sparse_scan_yaw_match_run_on_completed_sparse_scan);
    c.sparse_scan_yaw_match_run_on_useful_sparse_scan = get_bool(kv, "sparse_scan_yaw_match.run_on_useful_sparse_scan", c.sparse_scan_yaw_match_run_on_useful_sparse_scan);
    c.sparse_scan_yaw_match_require_complete_session = get_bool(kv, "sparse_scan_yaw_match.require_complete_session", c.sparse_scan_yaw_match_require_complete_session);
    c.sparse_scan_yaw_match_max_yaw_search_deg = get_double(kv, "sparse_scan_yaw_match.max_yaw_search_deg", c.sparse_scan_yaw_match_max_yaw_search_deg);
    c.sparse_scan_yaw_match_coarse_step_deg = get_double(kv, "sparse_scan_yaw_match.coarse_step_deg", c.sparse_scan_yaw_match_coarse_step_deg);
    c.sparse_scan_yaw_match_refine_enabled = get_bool(kv, "sparse_scan_yaw_match.refine_enabled", c.sparse_scan_yaw_match_refine_enabled);
    c.sparse_scan_yaw_match_refine_window_deg = get_double(kv, "sparse_scan_yaw_match.refine_window_deg", c.sparse_scan_yaw_match_refine_window_deg);
    c.sparse_scan_yaw_match_refine_step_deg = get_double(kv, "sparse_scan_yaw_match.refine_step_deg", c.sparse_scan_yaw_match_refine_step_deg);
    c.sparse_scan_yaw_match_min_valid_samples = get_int(kv, "sparse_scan_yaw_match.min_valid_samples", c.sparse_scan_yaw_match_min_valid_samples);
    c.sparse_scan_yaw_match_min_valid_bins = get_int(kv, "sparse_scan_yaw_match.min_valid_bins", c.sparse_scan_yaw_match_min_valid_bins);
    c.sparse_scan_yaw_match_min_observed_yaw_delta_deg = get_double(kv, "sparse_scan_yaw_match.min_observed_yaw_delta_deg", c.sparse_scan_yaw_match_min_observed_yaw_delta_deg);
    c.sparse_scan_yaw_match_min_valid_bin_ratio = get_double(kv, "sparse_scan_yaw_match.min_valid_bin_ratio", c.sparse_scan_yaw_match_min_valid_bin_ratio);
    c.sparse_scan_yaw_match_occupied_search_radius_m = get_double(kv, "sparse_scan_yaw_match.occupied_search_radius_m", c.sparse_scan_yaw_match_occupied_search_radius_m);
    c.sparse_scan_yaw_match_occupied_hit_reward = get_double(kv, "sparse_scan_yaw_match.occupied_hit_reward", c.sparse_scan_yaw_match_occupied_hit_reward);
    c.sparse_scan_yaw_match_free_hit_penalty = get_double(kv, "sparse_scan_yaw_match.free_hit_penalty", c.sparse_scan_yaw_match_free_hit_penalty);
    c.sparse_scan_yaw_match_unknown_hit_penalty = get_double(kv, "sparse_scan_yaw_match.unknown_hit_penalty", c.sparse_scan_yaw_match_unknown_hit_penalty);
    c.sparse_scan_yaw_match_distance_penalty_scale_m = get_double(kv, "sparse_scan_yaw_match.distance_penalty_scale_m", c.sparse_scan_yaw_match_distance_penalty_scale_m);
    c.sparse_scan_yaw_match_min_best_score = get_double(kv, "sparse_scan_yaw_match.min_best_score", c.sparse_scan_yaw_match_min_best_score);
    c.sparse_scan_yaw_match_min_score_margin = get_double(kv, "sparse_scan_yaw_match.min_score_margin", c.sparse_scan_yaw_match_min_score_margin);
    c.sparse_scan_yaw_match_min_inlier_ratio = get_double(kv, "sparse_scan_yaw_match.min_inlier_ratio", c.sparse_scan_yaw_match_min_inlier_ratio);
    c.sparse_scan_yaw_match_max_curve_flatness = get_double(kv, "sparse_scan_yaw_match.max_curve_flatness", c.sparse_scan_yaw_match_max_curve_flatness);
    c.sparse_scan_yaw_match_multimodal_check_enabled = get_bool(kv, "sparse_scan_yaw_match.multimodal_check_enabled", c.sparse_scan_yaw_match_multimodal_check_enabled);
    c.sparse_scan_yaw_match_multimodal_peak_separation_deg = get_double(kv, "sparse_scan_yaw_match.multimodal_peak_separation_deg", c.sparse_scan_yaw_match_multimodal_peak_separation_deg);
    c.sparse_scan_yaw_match_max_second_peak_ratio = get_double(kv, "sparse_scan_yaw_match.max_second_peak_ratio", c.sparse_scan_yaw_match_max_second_peak_ratio);
    c.sparse_scan_yaw_match_max_samples_per_match = get_int(kv, "sparse_scan_yaw_match.max_samples_per_match", c.sparse_scan_yaw_match_max_samples_per_match);
    c.sparse_scan_yaw_match_write_curve_log = get_bool(kv, "sparse_scan_yaw_match.write_curve_log", c.sparse_scan_yaw_match_write_curve_log);
    c.sparse_scan_yaw_match_write_summary_log = get_bool(kv, "sparse_scan_yaw_match.write_summary_log", c.sparse_scan_yaw_match_write_summary_log);
    c.yaw_correction_enabled = get_bool(kv, "yaw_correction.enabled", c.yaw_correction_enabled);
    c.yaw_correction_mode = get_string(kv, "yaw_correction.mode", c.yaw_correction_mode);
    c.yaw_correction_log_hz = get_double(kv, "yaw_correction.log_hz", c.yaw_correction_log_hz);
    c.yaw_correction_require_yaw_match_usable = get_bool(kv, "yaw_correction.require_yaw_match_usable", c.yaw_correction_require_yaw_match_usable);
    c.yaw_correction_require_mapping_state_ok = get_bool(kv, "yaw_correction.require_mapping_state_ok", c.yaw_correction_require_mapping_state_ok);
    c.yaw_correction_require_low_speed_or_static = get_bool(kv, "yaw_correction.require_low_speed_or_static", c.yaw_correction_require_low_speed_or_static);
    c.yaw_correction_require_active_scan_complete = get_bool(kv, "yaw_correction.require_active_scan_complete", c.yaw_correction_require_active_scan_complete);
    c.yaw_correction_scan_completion_source = get_string(kv, "yaw_correction.scan_completion_source", c.yaw_correction_scan_completion_source);
    c.yaw_correction_allow_yaw_match_evidence_complete = get_bool(kv, "yaw_correction.allow_yaw_match_evidence_complete", c.yaw_correction_allow_yaw_match_evidence_complete);
    c.yaw_correction_min_match_observed_yaw_delta_deg = get_double(kv, "yaw_correction.min_match_observed_yaw_delta_deg", c.yaw_correction_min_match_observed_yaw_delta_deg);
    c.yaw_correction_min_match_valid_samples = get_int(kv, "yaw_correction.min_match_valid_samples", c.yaw_correction_min_match_valid_samples);
    c.yaw_correction_min_match_valid_bins = get_int(kv, "yaw_correction.min_match_valid_bins", c.yaw_correction_min_match_valid_bins);
    c.yaw_correction_min_match_valid_bin_ratio = get_double(kv, "yaw_correction.min_match_valid_bin_ratio", c.yaw_correction_min_match_valid_bin_ratio);
    c.yaw_correction_require_yaw_match_attempted = get_bool(kv, "yaw_correction.require_yaw_match_attempted", c.yaw_correction_require_yaw_match_attempted);
    c.yaw_correction_max_linear_speed_mps = get_double(kv, "yaw_correction.max_linear_speed_mps", c.yaw_correction_max_linear_speed_mps);
    c.yaw_correction_max_abs_yaw_rate_dps = get_double(kv, "yaw_correction.max_abs_yaw_rate_dps", c.yaw_correction_max_abs_yaw_rate_dps);
    c.yaw_correction_min_best_score = get_double(kv, "yaw_correction.min_best_score", c.yaw_correction_min_best_score);
    c.yaw_correction_min_score_margin = get_double(kv, "yaw_correction.min_score_margin", c.yaw_correction_min_score_margin);
    c.yaw_correction_min_inlier_ratio = get_double(kv, "yaw_correction.min_inlier_ratio", c.yaw_correction_min_inlier_ratio);
    c.yaw_correction_max_curve_flatness = get_double(kv, "yaw_correction.max_curve_flatness", c.yaw_correction_max_curve_flatness);
    c.yaw_correction_max_candidate_abs_deg = get_double(kv, "yaw_correction.max_candidate_abs_deg", c.yaw_correction_max_candidate_abs_deg);
    c.yaw_correction_min_candidate_abs_deg = get_double(kv, "yaw_correction.min_candidate_abs_deg", c.yaw_correction_min_candidate_abs_deg);
    c.yaw_correction_gain = get_double(kv, "yaw_correction.correction_gain", c.yaw_correction_gain);
    c.yaw_correction_max_step_deg = get_double(kv, "yaw_correction.max_step_deg", c.yaw_correction_max_step_deg);
    c.yaw_correction_min_step_deg = get_double(kv, "yaw_correction.min_step_deg", c.yaw_correction_min_step_deg);
    c.yaw_correction_consistency_window = get_int(kv, "yaw_correction.consistency_window", c.yaw_correction_consistency_window);
    c.yaw_correction_max_consistency_spread_deg = get_double(kv, "yaw_correction.max_consistency_spread_deg", c.yaw_correction_max_consistency_spread_deg);
    c.yaw_correction_cooldown_s = get_double(kv, "yaw_correction.cooldown_s", c.yaw_correction_cooldown_s);
    c.yaw_correction_writeback_enabled = get_bool(kv, "yaw_correction.writeback_enabled", c.yaw_correction_writeback_enabled);
    c.yaw_correction_acknowledgement = get_string(kv, "yaw_correction.acknowledgement", c.yaw_correction_acknowledgement);
    c.yaw_correction_writeback_acknowledgement = get_string(kv, "yaw_correction.acknowledgement", c.yaw_correction_writeback_acknowledgement);
    c.yaw_correction_writeback_acknowledgement = get_string(kv, "yaw_correction.writeback_acknowledgement", c.yaw_correction_writeback_acknowledgement);
    c.yaw_correction_required_writeback_acknowledgement = get_string(kv, "yaw_correction.required_writeback_acknowledgement", c.yaw_correction_required_writeback_acknowledgement);
    c.yaw_correction_max_writeback_abs_deg = get_double(kv, "yaw_correction.max_writeback_abs_deg", c.yaw_correction_max_writeback_abs_deg);
    c.yaw_correction_max_total_writeback_per_session_deg = get_double(kv, "yaw_correction.max_total_writeback_per_session_deg", c.yaw_correction_max_total_writeback_per_session_deg);
    c.yaw_correction_max_writeback_count_per_session = get_int(kv, "yaw_correction.max_writeback_count_per_session", c.yaw_correction_max_writeback_count_per_session);
    c.yaw_correction_min_seconds_between_writebacks = get_double(kv, "yaw_correction.min_seconds_between_writebacks", c.yaw_correction_min_seconds_between_writebacks);
    c.yaw_correction_require_gate_would_apply = get_bool(kv, "yaw_correction.require_gate_would_apply", c.yaw_correction_require_gate_would_apply);
    c.yaw_correction_require_gate_reason = get_string(kv, "yaw_correction.require_gate_reason", c.yaw_correction_require_gate_reason);
    c.yaw_correction_require_scan_evidence_ok = get_bool(kv, "yaw_correction.require_scan_evidence_ok", c.yaw_correction_require_scan_evidence_ok);
    c.yaw_correction_require_yaw_match_evidence_ok = get_bool(kv, "yaw_correction.require_yaw_match_evidence_ok", c.yaw_correction_require_yaw_match_evidence_ok);
    c.yaw_correction_apply_log_enabled = get_bool(kv, "yaw_correction.apply_log_enabled", c.yaw_correction_apply_log_enabled);
    c.yaw_correction_post_apply_enabled = get_bool(kv, "yaw_correction_post_apply.enabled", c.yaw_correction_post_apply_enabled);
    c.yaw_correction_post_apply_timeout_s = get_double(kv, "yaw_correction_post_apply.timeout_s", c.yaw_correction_post_apply_timeout_s);
    c.yaw_correction_post_apply_min_improvement_deg = get_double(kv, "yaw_correction_post_apply.min_improvement_deg", c.yaw_correction_post_apply_min_improvement_deg);
    c.yaw_correction_post_apply_min_improvement_fraction_of_applied_delta = get_double(kv, "yaw_correction_post_apply.min_improvement_fraction_of_applied_delta", c.yaw_correction_post_apply_min_improvement_fraction_of_applied_delta);
    c.yaw_correction_post_apply_min_absolute_improvement_deg = get_double(kv, "yaw_correction_post_apply.min_absolute_improvement_deg", c.yaw_correction_post_apply_min_absolute_improvement_deg);
    c.yaw_correction_post_apply_max_allowed_worse_deg = get_double(kv, "yaw_correction_post_apply.max_allowed_worse_deg", c.yaw_correction_post_apply_max_allowed_worse_deg);
    c.yaw_correction_post_apply_require_new_scan_id = get_bool(kv, "yaw_correction_post_apply.require_new_scan_id", c.yaw_correction_post_apply_require_new_scan_id);
    c.yaw_correction_post_apply_require_newer_match_timestamp = get_bool(kv, "yaw_correction_post_apply.require_newer_match_timestamp", c.yaw_correction_post_apply_require_newer_match_timestamp);
    c.yaw_correction_post_apply_max_post_apply_candidate_abs_deg = get_double(kv, "yaw_correction_post_apply.max_post_apply_candidate_abs_deg", c.yaw_correction_post_apply_max_post_apply_candidate_abs_deg);
    c.yaw_correction_post_apply_log_enabled = get_bool(kv, "yaw_correction_post_apply.log_enabled", c.yaw_correction_post_apply_log_enabled);
    c.recovery_enabled = get_bool(kv, "recovery.enabled", c.recovery_enabled);
    c.recovery_mode = get_string(kv, "recovery.mode", c.recovery_mode);
    c.recovery_log_hz = get_double(kv, "recovery.log_hz", c.recovery_log_hz);
    c.recovery_startup_recovery_enabled = get_bool(kv, "recovery.startup_recovery_enabled", c.recovery_startup_recovery_enabled);
    c.recovery_lost_recovery_enabled = get_bool(kv, "recovery.lost_recovery_enabled", c.recovery_lost_recovery_enabled);
    c.recovery_degraded_recovery_enabled = get_bool(kv, "recovery.degraded_recovery_enabled", c.recovery_degraded_recovery_enabled);
    c.recovery_startup_grace_s = get_double(kv, "recovery.startup_grace_s", c.recovery_startup_grace_s);
    c.recovery_lost_confirm_s = get_double(kv, "recovery.lost_confirm_s", c.recovery_lost_confirm_s);
    c.recovery_degraded_confirm_s = get_double(kv, "recovery.degraded_confirm_s", c.recovery_degraded_confirm_s);
    c.recovery_require_map_quality_not_invalid = get_bool(kv, "recovery.require_map_quality_not_invalid", c.recovery_require_map_quality_not_invalid);
    c.recovery_require_min_known_cells = get_int(kv, "recovery.require_min_known_cells", c.recovery_require_min_known_cells);
    c.recovery_require_min_occupied_cells = get_int(kv, "recovery.require_min_occupied_cells", c.recovery_require_min_occupied_cells);
    c.recovery_require_tof_recent = get_bool(kv, "recovery.require_tof_recent", c.recovery_require_tof_recent);
    c.recovery_max_tof_age_s = get_double(kv, "recovery.max_tof_age_s", c.recovery_max_tof_age_s);
    c.recovery_require_localizer_initialized_for_local_recovery = get_bool(kv, "recovery.require_localizer_initialized_for_local_recovery", c.recovery_require_localizer_initialized_for_local_recovery);
    c.recovery_allow_uninitialized_startup_recovery_observe_only = get_bool(kv, "recovery.allow_uninitialized_startup_recovery_observe_only", c.recovery_allow_uninitialized_startup_recovery_observe_only);
    c.recovery_request_recovery_scan = get_bool(kv, "recovery.request_recovery_scan", c.recovery_request_recovery_scan);
    c.recovery_recovery_scan_cooldown_s = get_double(kv, "recovery.recovery_scan_cooldown_s", c.recovery_recovery_scan_cooldown_s);
    c.recovery_require_yaw_correction_stable = get_bool(kv, "recovery.require_yaw_correction_stable", c.recovery_require_yaw_correction_stable);
    c.recovery_max_recent_yaw_apply_count = get_int(kv, "recovery.max_recent_yaw_apply_count", c.recovery_max_recent_yaw_apply_count);
    c.recovery_min_post_apply_validated_count = get_int(kv, "recovery.min_post_apply_validated_count", c.recovery_min_post_apply_validated_count);
    c.recovery_block_if_post_apply_failed_recent = get_bool(kv, "recovery.block_if_post_apply_failed_recent", c.recovery_block_if_post_apply_failed_recent);
    c.recovery_write_candidate_log = get_bool(kv, "recovery.write_candidate_log", c.recovery_write_candidate_log);
    c.recovery_write_event_log = get_bool(kv, "recovery.write_event_log", c.recovery_write_event_log);
    parse_motion_execution_config(c, kv);
    c.autonomous_slam_enabled = get_bool(kv, "autonomous_slam.enabled", c.autonomous_slam_enabled);
    c.autonomous_slam_max_iterations = get_int(kv, "autonomous_slam.max_iterations", c.autonomous_slam_max_iterations);
    c.autonomous_slam_sensor_timeout_s = get_double(kv, "autonomous_slam.sensor_timeout_s", c.autonomous_slam_sensor_timeout_s);
    c.autonomous_slam_motion_settle_timeout_s = get_double(kv, "autonomous_slam.motion_settle_timeout_s", c.autonomous_slam_motion_settle_timeout_s);
    c.autonomous_slam_active_scan_speed = get_double(kv, "autonomous_slam.active_scan_speed", c.autonomous_slam_active_scan_speed);
    c.autonomous_slam_active_scan_duration_s = get_double(kv, "autonomous_slam.active_scan_duration_s", c.autonomous_slam_active_scan_duration_s);
    c.autonomous_slam_max_active_scan_commands = get_int(kv, "autonomous_slam.max_active_scan_commands", c.autonomous_slam_max_active_scan_commands);
    c.autonomous_slam_prefer_left_turn = get_bool(kv, "autonomous_slam.prefer_left_turn", c.autonomous_slam_prefer_left_turn);
    c.autonomous_slam_require_tof = get_bool(kv, "autonomous_slam.require_tof", c.autonomous_slam_require_tof);
    c.autonomous_slam_require_imu_or_wheel = get_bool(kv, "autonomous_slam.require_imu_or_wheel", c.autonomous_slam_require_imu_or_wheel);
    c.autonomous_slam_allow_forward_backward = get_bool(kv, "autonomous_slam.allow_forward_backward", c.autonomous_slam_allow_forward_backward);
    c.real_adapter_contract_enabled = get_bool(kv, "real_adapter_contract.enabled", c.real_adapter_contract_enabled);
    c.real_adapter_contract_require_tof = get_bool(kv, "real_adapter_contract.require_tof", c.real_adapter_contract_require_tof);
    c.real_adapter_contract_require_imu_or_wheel = get_bool(kv, "real_adapter_contract.require_imu_or_wheel", c.real_adapter_contract_require_imu_or_wheel);
    c.real_adapter_contract_tof_max_frame_age_s = get_double(kv, "real_adapter_contract.tof_max_frame_age_s", c.real_adapter_contract_tof_max_frame_age_s);
    c.real_adapter_contract_imu_max_frame_age_s = get_double(kv, "real_adapter_contract.imu_max_frame_age_s", c.real_adapter_contract_imu_max_frame_age_s);
    c.real_adapter_contract_wheel_max_frame_age_s = get_double(kv, "real_adapter_contract.wheel_max_frame_age_s", c.real_adapter_contract_wheel_max_frame_age_s);
    c.real_adapter_contract_tof_min_range_count = get_int(kv, "real_adapter_contract.tof_min_range_count", c.real_adapter_contract_tof_min_range_count);
    c.real_adapter_contract_tof_max_range_count = get_int(kv, "real_adapter_contract.tof_max_range_count", c.real_adapter_contract_tof_max_range_count);
    c.real_adapter_contract_tof_min_range_m = get_double(kv, "real_adapter_contract.tof_min_range_m", c.real_adapter_contract_tof_min_range_m);
    c.real_adapter_contract_tof_max_range_m = get_double(kv, "real_adapter_contract.tof_max_range_m", c.real_adapter_contract_tof_max_range_m);
    c.real_adapter_contract_tof_max_allowed_nan_ratio = get_double(kv, "real_adapter_contract.tof_max_allowed_nan_ratio", c.real_adapter_contract_tof_max_allowed_nan_ratio);
    c.real_adapter_contract_require_tof_frame_id = get_bool(kv, "real_adapter_contract.require_tof_frame_id", c.real_adapter_contract_require_tof_frame_id);
    c.real_adapter_contract_imu_max_abs_yaw_rate_rad_s = get_double(kv, "real_adapter_contract.imu_max_abs_yaw_rate_rad_s", c.real_adapter_contract_imu_max_abs_yaw_rate_rad_s);
    c.real_adapter_contract_imu_max_abs_acc_mps2 = get_double(kv, "real_adapter_contract.imu_max_abs_acc_mps2", c.real_adapter_contract_imu_max_abs_acc_mps2);
    c.real_adapter_contract_wheel_max_abs_linear_mps = get_double(kv, "real_adapter_contract.wheel_max_abs_linear_mps", c.real_adapter_contract_wheel_max_abs_linear_mps);
    c.real_adapter_contract_wheel_max_abs_angular_rad_s = get_double(kv, "real_adapter_contract.wheel_max_abs_angular_rad_s", c.real_adapter_contract_wheel_max_abs_angular_rad_s);
    c.real_adapter_contract_require_shadow_before_live = get_bool(kv, "real_adapter_contract.require_shadow_before_live", c.real_adapter_contract_require_shadow_before_live);
    c.prelive_autonomous_slam_enabled = get_bool(kv, "prelive_autonomous_slam.enabled", c.prelive_autonomous_slam_enabled);
    c.prelive_autonomous_slam_max_iterations = get_int(kv, "prelive_autonomous_slam.max_iterations", c.prelive_autonomous_slam_max_iterations);
    c.prelive_autonomous_slam_start_time_s = get_double(kv, "prelive_autonomous_slam.start_time_s", c.prelive_autonomous_slam_start_time_s);
    c.prelive_autonomous_slam_step_s = get_double(kv, "prelive_autonomous_slam.step_s", c.prelive_autonomous_slam_step_s);
    c.prelive_autonomous_slam_require_adapter_acceptance = get_bool(kv, "prelive_autonomous_slam.require_adapter_acceptance", c.prelive_autonomous_slam_require_adapter_acceptance);
    c.prelive_autonomous_slam_require_coordinator_completed = get_bool(kv, "prelive_autonomous_slam.require_coordinator_completed", c.prelive_autonomous_slam_require_coordinator_completed);
    c.prelive_autonomous_slam_require_no_motion_rejection = get_bool(kv, "prelive_autonomous_slam.require_no_motion_rejection", c.prelive_autonomous_slam_require_no_motion_rejection);
    c.prelive_autonomous_slam_require_stop_command_seen = get_bool(kv, "prelive_autonomous_slam.require_stop_command_seen", c.prelive_autonomous_slam_require_stop_command_seen);
    c.prelive_autonomous_slam_require_active_scan_command_seen = get_bool(kv, "prelive_autonomous_slam.require_active_scan_command_seen", c.prelive_autonomous_slam_require_active_scan_command_seen);
    c.prelive_autonomous_slam_require_map_quality_good = get_bool(kv, "prelive_autonomous_slam.require_map_quality_good", c.prelive_autonomous_slam_require_map_quality_good);
    c.prelive_autonomous_slam_allow_coordinator_incomplete_for_shadow = get_bool(kv, "prelive_autonomous_slam.allow_coordinator_incomplete_for_shadow", c.prelive_autonomous_slam_allow_coordinator_incomplete_for_shadow);
    c.slam_backend_binding_enabled = get_bool(kv, "slam_backend_binding.enabled", c.slam_backend_binding_enabled);
    c.slam_backend_binding_require_tof = get_bool(kv, "slam_backend_binding.require_tof", c.slam_backend_binding_require_tof);
    c.slam_backend_binding_require_imu_or_wheel = get_bool(kv, "slam_backend_binding.require_imu_or_wheel", c.slam_backend_binding_require_imu_or_wheel);
    c.slam_backend_binding_allow_predicted_pose_missing = get_bool(kv, "slam_backend_binding.allow_predicted_pose_missing", c.slam_backend_binding_allow_predicted_pose_missing);
    c.slam_backend_binding_max_input_age_s = get_double(kv, "slam_backend_binding.max_input_age_s", c.slam_backend_binding_max_input_age_s);
    c.slam_backend_binding_max_update_latency_s = get_double(kv, "slam_backend_binding.max_update_latency_s", c.slam_backend_binding_max_update_latency_s);
    c.slam_backend_binding_min_integrated_scan_count_for_quality = get_int(kv, "slam_backend_binding.min_integrated_scan_count_for_quality", c.slam_backend_binding_min_integrated_scan_count_for_quality);
    c.slam_backend_binding_require_ready_for_acceptance = get_bool(kv, "slam_backend_binding.require_ready_for_acceptance", c.slam_backend_binding_require_ready_for_acceptance);
    c.slam_backend_binding_require_update_accepted = get_bool(kv, "slam_backend_binding.require_update_accepted", c.slam_backend_binding_require_update_accepted);
    c.slam_backend_binding_require_quality_valid = get_bool(kv, "slam_backend_binding.require_quality_valid", c.slam_backend_binding_require_quality_valid);
    c.slam_backend_binding_require_save_map = get_bool(kv, "slam_backend_binding.require_save_map", c.slam_backend_binding_require_save_map);
    c.autonomous_slam_e2e_prelive_enabled = get_bool(kv, "autonomous_slam_e2e_prelive.enabled", c.autonomous_slam_e2e_prelive_enabled);
    c.autonomous_slam_e2e_prelive_scenario_kind = get_string(kv, "autonomous_slam_e2e_prelive.scenario_kind", c.autonomous_slam_e2e_prelive_scenario_kind);
    c.autonomous_slam_e2e_prelive_max_iterations = get_int(kv, "autonomous_slam_e2e_prelive.max_iterations", c.autonomous_slam_e2e_prelive_max_iterations);
    c.autonomous_slam_e2e_prelive_start_time_s = get_double(kv, "autonomous_slam_e2e_prelive.start_time_s", c.autonomous_slam_e2e_prelive_start_time_s);
    c.autonomous_slam_e2e_prelive_step_s = get_double(kv, "autonomous_slam_e2e_prelive.step_s", c.autonomous_slam_e2e_prelive_step_s);
    c.autonomous_slam_e2e_prelive_require_slam_backend_acceptance = get_bool(kv, "autonomous_slam_e2e_prelive.require_slam_backend_acceptance", c.autonomous_slam_e2e_prelive_require_slam_backend_acceptance);
    c.autonomous_slam_e2e_prelive_require_prelive_pass = get_bool(kv, "autonomous_slam_e2e_prelive.require_prelive_pass", c.autonomous_slam_e2e_prelive_require_prelive_pass);
    c.autonomous_slam_e2e_prelive_require_no_forward_backward = get_bool(kv, "autonomous_slam_e2e_prelive.require_no_forward_backward", c.autonomous_slam_e2e_prelive_require_no_forward_backward);
    c.autonomous_slam_e2e_prelive_require_stop_command_seen = get_bool(kv, "autonomous_slam_e2e_prelive.require_stop_command_seen", c.autonomous_slam_e2e_prelive_require_stop_command_seen);
    c.autonomous_slam_e2e_prelive_require_active_scan_when_map_poor = get_bool(kv, "autonomous_slam_e2e_prelive.require_active_scan_when_map_poor", c.autonomous_slam_e2e_prelive_require_active_scan_when_map_poor);
    c.autonomous_slam_e2e_prelive_require_map_quality_good_at_end = get_bool(kv, "autonomous_slam_e2e_prelive.require_map_quality_good_at_end", c.autonomous_slam_e2e_prelive_require_map_quality_good_at_end);
    c.real_adapter_stubs_enabled = get_bool(kv, "real_adapter_stubs.enabled", c.real_adapter_stubs_enabled);
    c.real_adapter_stubs_create_sensor_stub = get_bool(kv, "real_adapter_stubs.create_sensor_stub", c.real_adapter_stubs_create_sensor_stub);
    c.real_adapter_stubs_create_motion_stub = get_bool(kv, "real_adapter_stubs.create_motion_stub", c.real_adapter_stubs_create_motion_stub);
    c.real_adapter_stubs_create_slam_backend_stub = get_bool(kv, "real_adapter_stubs.create_slam_backend_stub", c.real_adapter_stubs_create_slam_backend_stub);
    c.real_adapter_stubs_allow_real_hardware_adapters = get_bool(kv, "real_adapter_stubs.allow_real_hardware_adapters", c.real_adapter_stubs_allow_real_hardware_adapters);
    c.real_adapter_stubs_require_explicit_live_enable = get_bool(kv, "real_adapter_stubs.require_explicit_live_enable", c.real_adapter_stubs_require_explicit_live_enable);
    c.live_handoff_readiness_enabled = get_bool(kv, "live_handoff_readiness.enabled", c.live_handoff_readiness_enabled);
    c.live_handoff_readiness_require_real_sensor_adapter = get_bool(kv, "live_handoff_readiness.require_real_sensor_adapter", c.live_handoff_readiness_require_real_sensor_adapter);
    c.live_handoff_readiness_require_real_motion_adapter = get_bool(kv, "live_handoff_readiness.require_real_motion_adapter", c.live_handoff_readiness_require_real_motion_adapter);
    c.live_handoff_readiness_require_real_slam_backend = get_bool(kv, "live_handoff_readiness.require_real_slam_backend", c.live_handoff_readiness_require_real_slam_backend);
    c.live_handoff_readiness_require_software_transport_acceptance = get_bool(kv, "live_handoff_readiness.require_software_transport_acceptance", c.live_handoff_readiness_require_software_transport_acceptance);
    c.live_handoff_readiness_require_e2e_prelive_pass = get_bool(kv, "live_handoff_readiness.require_e2e_prelive_pass", c.live_handoff_readiness_require_e2e_prelive_pass);
    c.live_handoff_readiness_require_direction_probe = get_bool(kv, "live_handoff_readiness.require_direction_probe", c.live_handoff_readiness_require_direction_probe);
    c.live_handoff_readiness_require_stop_estop_ttl = get_bool(kv, "live_handoff_readiness.require_stop_estop_ttl", c.live_handoff_readiness_require_stop_estop_ttl);
    c.live_handoff_readiness_require_hardware_safety = get_bool(kv, "live_handoff_readiness.require_hardware_safety", c.live_handoff_readiness_require_hardware_safety);
    c.live_handoff_readiness_allow_forward_backward = get_bool(kv, "live_handoff_readiness.allow_forward_backward", c.live_handoff_readiness_allow_forward_backward);
    c.real_sensor_adapter_data_contract_enabled = get_bool(kv, "real_sensor_adapter_data_contract.enabled", c.real_sensor_adapter_data_contract_enabled);
    c.real_sensor_adapter_data_contract_require_tof = get_bool(kv, "real_sensor_adapter_data_contract.require_tof", c.real_sensor_adapter_data_contract_require_tof);
    c.real_sensor_adapter_data_contract_require_imu_or_wheel = get_bool(kv, "real_sensor_adapter_data_contract.require_imu_or_wheel", c.real_sensor_adapter_data_contract_require_imu_or_wheel);
    c.real_sensor_adapter_data_contract_require_frame_id = get_bool(kv, "real_sensor_adapter_data_contract.require_frame_id", c.real_sensor_adapter_data_contract_require_frame_id);
    c.real_sensor_adapter_data_contract_require_request_timing = get_bool(kv, "real_sensor_adapter_data_contract.require_request_timing", c.real_sensor_adapter_data_contract_require_request_timing);
    c.real_sensor_adapter_data_contract_allow_nan_ranges = get_bool(kv, "real_sensor_adapter_data_contract.allow_nan_ranges", c.real_sensor_adapter_data_contract_allow_nan_ranges);
    c.real_sensor_adapter_data_contract_max_packet_age_s = get_double(kv, "real_sensor_adapter_data_contract.max_packet_age_s", c.real_sensor_adapter_data_contract_max_packet_age_s);
    c.real_sensor_adapter_data_contract_max_sensor_sync_dt_s = get_double(kv, "real_sensor_adapter_data_contract.max_sensor_sync_dt_s", c.real_sensor_adapter_data_contract_max_sensor_sync_dt_s);
    c.real_sensor_adapter_data_contract_max_request_latency_s = get_double(kv, "real_sensor_adapter_data_contract.max_request_latency_s", c.real_sensor_adapter_data_contract_max_request_latency_s);
    c.real_sensor_adapter_data_contract_max_tof_nan_ratio = get_double(kv, "real_sensor_adapter_data_contract.max_tof_nan_ratio", c.real_sensor_adapter_data_contract_max_tof_nan_ratio);
    c.real_sensor_adapter_data_contract_min_tof_range_m = get_double(kv, "real_sensor_adapter_data_contract.min_tof_range_m", c.real_sensor_adapter_data_contract_min_tof_range_m);
    c.real_sensor_adapter_data_contract_max_tof_range_m = get_double(kv, "real_sensor_adapter_data_contract.max_tof_range_m", c.real_sensor_adapter_data_contract_max_tof_range_m);
    c.real_sensor_adapter_data_contract_max_abs_yaw_rate_rad_s = get_double(kv, "real_sensor_adapter_data_contract.max_abs_yaw_rate_rad_s", c.real_sensor_adapter_data_contract_max_abs_yaw_rate_rad_s);
    c.real_sensor_adapter_data_contract_max_accel_norm_mps2 = get_double(kv, "real_sensor_adapter_data_contract.max_accel_norm_mps2", c.real_sensor_adapter_data_contract_max_accel_norm_mps2);
    c.real_sensor_adapter_data_contract_max_abs_wheel_linear_mps = get_double(kv, "real_sensor_adapter_data_contract.max_abs_wheel_linear_mps", c.real_sensor_adapter_data_contract_max_abs_wheel_linear_mps);
    c.real_sensor_adapter_data_contract_max_abs_wheel_angular_rad_s = get_double(kv, "real_sensor_adapter_data_contract.max_abs_wheel_angular_rad_s", c.real_sensor_adapter_data_contract_max_abs_wheel_angular_rad_s);
    c.real_sensor_adapter_data_contract_default_tof_frame_id = get_string(kv, "real_sensor_adapter_data_contract.default_tof_frame_id", c.real_sensor_adapter_data_contract_default_tof_frame_id);
    c.real_sensor_adapter_data_contract_default_imu_frame_id = get_string(kv, "real_sensor_adapter_data_contract.default_imu_frame_id", c.real_sensor_adapter_data_contract_default_imu_frame_id);
    c.real_sensor_adapter_data_contract_default_wheel_frame_id = get_string(kv, "real_sensor_adapter_data_contract.default_wheel_frame_id", c.real_sensor_adapter_data_contract_default_wheel_frame_id);
    c.real_sensor_adapter_data_contract_run_acceptance_on_startup = get_bool(kv, "real_sensor_adapter_data_contract.run_acceptance_on_startup", c.real_sensor_adapter_data_contract_run_acceptance_on_startup);
    c.real_sensor_adapter_data_contract_max_request_latency_mismatch_s = get_double(kv, "real_sensor_adapter_data_contract.max_request_latency_mismatch_s", c.real_sensor_adapter_data_contract_max_request_latency_mismatch_s);
    c.real_sensor_adapter_data_contract_max_estimated_sample_time_midpoint_error_s = get_double(kv, "real_sensor_adapter_data_contract.max_estimated_sample_time_midpoint_error_s", c.real_sensor_adapter_data_contract_max_estimated_sample_time_midpoint_error_s);
    c.real_sensor_adapter_data_contract_max_future_timestamp_skew_s = get_double(kv, "real_sensor_adapter_data_contract.max_future_timestamp_skew_s", c.real_sensor_adapter_data_contract_max_future_timestamp_skew_s);
    c.real_sensor_adapter_data_contract_max_packet_sensor_time_dt_s = get_double(kv, "real_sensor_adapter_data_contract.max_packet_sensor_time_dt_s", c.real_sensor_adapter_data_contract_max_packet_sensor_time_dt_s);
    c.real_sensor_adapter_data_contract_reject_request_latency_mismatch = get_bool(kv, "real_sensor_adapter_data_contract.reject_request_latency_mismatch", c.real_sensor_adapter_data_contract_reject_request_latency_mismatch);
    c.real_sensor_adapter_data_contract_require_estimated_sample_time_in_window = get_bool(kv, "real_sensor_adapter_data_contract.require_estimated_sample_time_in_window", c.real_sensor_adapter_data_contract_require_estimated_sample_time_in_window);
    c.real_sensor_adapter_data_contract_require_estimated_sample_time_midpoint = get_bool(kv, "real_sensor_adapter_data_contract.require_estimated_sample_time_midpoint", c.real_sensor_adapter_data_contract_require_estimated_sample_time_midpoint);
    c.real_sensor_adapter_data_contract_reject_future_sensor_time = get_bool(kv, "real_sensor_adapter_data_contract.reject_future_sensor_time", c.real_sensor_adapter_data_contract_reject_future_sensor_time);
    c.real_sensor_replay_enabled = get_bool(kv, "real_sensor_replay.enabled", c.real_sensor_replay_enabled);
    c.real_sensor_replay_loop = get_bool(kv, "real_sensor_replay.loop", c.real_sensor_replay_loop);
    c.real_sensor_replay_fail_on_contract_error = get_bool(kv, "real_sensor_replay.fail_on_contract_error", c.real_sensor_replay_fail_on_contract_error);
    c.real_sensor_replay_require_non_empty_log = get_bool(kv, "real_sensor_replay.require_non_empty_log", c.real_sensor_replay_require_non_empty_log);
    c.real_sensor_replay_run_acceptance_on_startup = get_bool(kv, "real_sensor_replay.run_acceptance_on_startup", c.real_sensor_replay_run_acceptance_on_startup);
    c.real_sensor_replay_start_time_s = get_double(kv, "real_sensor_replay.start_time_s", c.real_sensor_replay_start_time_s);
    c.real_sensor_replay_time_mode = get_string(kv, "real_sensor_replay.time_mode", c.real_sensor_replay_time_mode);
    c.real_sensor_replay_reject_invalid_records = get_bool(kv, "real_sensor_replay.reject_invalid_records", c.real_sensor_replay_reject_invalid_records);
    c.real_sensor_replay_require_packet_records = get_bool(kv, "real_sensor_replay.require_packet_records", c.real_sensor_replay_require_packet_records);
    c.real_sensor_replay_preserve_parse_errors = get_bool(kv, "real_sensor_replay.preserve_parse_errors", c.real_sensor_replay_preserve_parse_errors);
    c.real_sensor_replay_max_records_per_run = get_int(kv, "real_sensor_replay.max_records_per_run", c.real_sensor_replay_max_records_per_run);
    c.real_sensor_replay_regression_enabled = get_bool(kv, "real_sensor_replay_regression.enabled", c.real_sensor_replay_regression_enabled);
    c.real_sensor_replay_regression_require_valid_log_pass = get_bool(kv, "real_sensor_replay_regression.require_valid_log_pass", c.real_sensor_replay_regression_require_valid_log_pass);
    c.real_sensor_replay_regression_require_invalid_logs_fail = get_bool(kv, "real_sensor_replay_regression.require_invalid_logs_fail", c.real_sensor_replay_regression_require_invalid_logs_fail);
    c.real_sensor_replay_regression_require_parse_errors_detected = get_bool(kv, "real_sensor_replay_regression.require_parse_errors_detected", c.real_sensor_replay_regression_require_parse_errors_detected);
    c.real_sensor_replay_regression_require_comment_only_log_rejected = get_bool(kv, "real_sensor_replay_regression.require_comment_only_log_rejected", c.real_sensor_replay_regression_require_comment_only_log_rejected);
    c.real_sensor_replay_regression_run_on_startup = get_bool(kv, "real_sensor_replay_regression.run_on_startup", c.real_sensor_replay_regression_run_on_startup);
    c.deterministic_slam_backend_enabled = get_bool(kv, "deterministic_slam_backend.enabled", c.deterministic_slam_backend_enabled);
    c.deterministic_slam_backend_ready = get_bool(kv, "deterministic_slam_backend.ready", c.deterministic_slam_backend_ready);
    c.deterministic_slam_backend_require_tof = get_bool(kv, "deterministic_slam_backend.require_tof", c.deterministic_slam_backend_require_tof);
    c.deterministic_slam_backend_require_imu_or_wheel = get_bool(kv, "deterministic_slam_backend.require_imu_or_wheel", c.deterministic_slam_backend_require_imu_or_wheel);
    c.deterministic_slam_backend_allow_save_map = get_bool(kv, "deterministic_slam_backend.allow_save_map", c.deterministic_slam_backend_allow_save_map);
    c.deterministic_slam_backend_min_valid_ranges = get_int(kv, "deterministic_slam_backend.min_valid_ranges", c.deterministic_slam_backend_min_valid_ranges);
    c.deterministic_slam_backend_min_valid_scan_count_for_good = get_int(kv, "deterministic_slam_backend.min_valid_scan_count_for_good", c.deterministic_slam_backend_min_valid_scan_count_for_good);
    c.deterministic_slam_backend_min_valid_range_ratio = get_double(kv, "deterministic_slam_backend.min_valid_range_ratio", c.deterministic_slam_backend_min_valid_range_ratio);
    c.deterministic_slam_backend_min_coverage_ratio_for_good = get_double(kv, "deterministic_slam_backend.min_coverage_ratio_for_good", c.deterministic_slam_backend_min_coverage_ratio_for_good);
    c.deterministic_slam_backend_min_yaw_coverage_ratio_for_good = get_double(kv, "deterministic_slam_backend.min_yaw_coverage_ratio_for_good", c.deterministic_slam_backend_min_yaw_coverage_ratio_for_good);
    c.deterministic_slam_backend_keyframe_yaw_delta_rad = get_double(kv, "deterministic_slam_backend.keyframe_yaw_delta_rad", c.deterministic_slam_backend_keyframe_yaw_delta_rad);
    c.deterministic_slam_backend_min_range_m = get_double(kv, "deterministic_slam_backend.min_range_m", c.deterministic_slam_backend_min_range_m);
    c.deterministic_slam_backend_max_range_m = get_double(kv, "deterministic_slam_backend.max_range_m", c.deterministic_slam_backend_max_range_m);
    c.deterministic_slam_backend_max_update_latency_s = get_double(kv, "deterministic_slam_backend.max_update_latency_s", c.deterministic_slam_backend_max_update_latency_s);
    c.deterministic_slam_backend_assumed_scan_yaw_span_rad = get_double(kv, "deterministic_slam_backend.assumed_scan_yaw_span_rad", c.deterministic_slam_backend_assumed_scan_yaw_span_rad);
    c.deterministic_slam_backend_yaw_bin_size_rad = get_double(kv, "deterministic_slam_backend.yaw_bin_size_rad", c.deterministic_slam_backend_yaw_bin_size_rad);
    c.deterministic_slam_backend_run_regression_on_startup = get_bool(kv, "deterministic_slam_backend.run_regression_on_startup", c.deterministic_slam_backend_run_regression_on_startup);
    c.replay_to_slam_backend_regression_enabled = get_bool(kv, "replay_to_slam_backend_regression.enabled", c.replay_to_slam_backend_regression_enabled);
    c.replay_to_slam_backend_regression_require_valid_replay_updates_map = get_bool(kv, "replay_to_slam_backend_regression.require_valid_replay_updates_map", c.replay_to_slam_backend_regression_require_valid_replay_updates_map);
    c.replay_to_slam_backend_regression_require_invalid_replay_rejected = get_bool(kv, "replay_to_slam_backend_regression.require_invalid_replay_rejected", c.replay_to_slam_backend_regression_require_invalid_replay_rejected);
    c.replay_to_slam_backend_regression_min_accepted_updates = get_int(kv, "replay_to_slam_backend_regression.min_accepted_updates", c.replay_to_slam_backend_regression_min_accepted_updates);
    c.replay_to_slam_backend_regression_run_on_startup = get_bool(kv, "replay_to_slam_backend_regression.run_on_startup", c.replay_to_slam_backend_regression_run_on_startup);
    c.full_autonomous_slam_fake_pipeline_enabled = get_bool(kv, "full_autonomous_slam_fake_pipeline.enabled", c.full_autonomous_slam_fake_pipeline_enabled);
    c.full_autonomous_slam_fake_pipeline_run_on_startup = get_bool(kv, "full_autonomous_slam_fake_pipeline.run_on_startup", c.full_autonomous_slam_fake_pipeline_run_on_startup);
    c.full_autonomous_slam_fake_pipeline_max_steps = get_int(kv, "full_autonomous_slam_fake_pipeline.max_steps", c.full_autonomous_slam_fake_pipeline_max_steps);
    c.full_autonomous_slam_fake_pipeline_max_active_scan_commands = get_int(kv, "full_autonomous_slam_fake_pipeline.max_active_scan_commands", c.full_autonomous_slam_fake_pipeline_max_active_scan_commands);
    c.full_autonomous_slam_fake_pipeline_min_backend_accepted_updates = get_int(kv, "full_autonomous_slam_fake_pipeline.min_backend_accepted_updates", c.full_autonomous_slam_fake_pipeline_min_backend_accepted_updates);
    c.full_autonomous_slam_fake_pipeline_require_completion = get_bool(kv, "full_autonomous_slam_fake_pipeline.require_completion", c.full_autonomous_slam_fake_pipeline_require_completion);
    c.full_autonomous_slam_fake_pipeline_require_shadow_motion_only = get_bool(kv, "full_autonomous_slam_fake_pipeline.require_shadow_motion_only", c.full_autonomous_slam_fake_pipeline_require_shadow_motion_only);
    c.full_autonomous_slam_fake_pipeline_require_no_forward_backward = get_bool(kv, "full_autonomous_slam_fake_pipeline.require_no_forward_backward", c.full_autonomous_slam_fake_pipeline_require_no_forward_backward);
    c.full_autonomous_slam_fake_pipeline_require_map_quality_good = get_bool(kv, "full_autonomous_slam_fake_pipeline.require_map_quality_good", c.full_autonomous_slam_fake_pipeline_require_map_quality_good);
    c.full_autonomous_slam_fake_pipeline_motion_settle_s = get_double(kv, "full_autonomous_slam_fake_pipeline.motion_settle_s", c.full_autonomous_slam_fake_pipeline_motion_settle_s);
    c.full_autonomous_slam_fake_pipeline_build_fake_map_on_completed = get_bool(kv, "full_autonomous_slam_fake_pipeline.build_fake_map_on_completed", c.full_autonomous_slam_fake_pipeline_build_fake_map_on_completed);
    c.full_autonomous_slam_fake_pipeline_save_fake_map_on_completed = get_bool(kv, "full_autonomous_slam_fake_pipeline.save_fake_map_on_completed", c.full_autonomous_slam_fake_pipeline_save_fake_map_on_completed);
    c.full_autonomous_slam_fake_pipeline_require_fake_map_saved = get_bool(kv, "full_autonomous_slam_fake_pipeline.require_fake_map_saved", c.full_autonomous_slam_fake_pipeline_require_fake_map_saved);
    c.full_autonomous_slam_fake_pipeline_fake_map_id_prefix = get_string(kv, "full_autonomous_slam_fake_pipeline.fake_map_id_prefix", c.full_autonomous_slam_fake_pipeline_fake_map_id_prefix);
    c.full_autonomous_slam_fake_pipeline_phase_aware_sensor_consumption = get_bool(kv, "full_autonomous_slam_fake_pipeline.phase_aware_sensor_consumption", c.full_autonomous_slam_fake_pipeline_phase_aware_sensor_consumption);
    c.full_autonomous_slam_fake_pipeline_require_phase_aware_sensor_consumption = get_bool(kv, "full_autonomous_slam_fake_pipeline.require_phase_aware_sensor_consumption", c.full_autonomous_slam_fake_pipeline_require_phase_aware_sensor_consumption);
    c.fake_map_artifact_enabled = get_bool(kv, "fake_map_artifact.enabled", c.fake_map_artifact_enabled);
    c.fake_map_artifact_allow_overwrite = get_bool(kv, "fake_map_artifact.allow_overwrite", c.fake_map_artifact_allow_overwrite);
    c.fake_map_artifact_require_quality_good = get_bool(kv, "fake_map_artifact.require_quality_good", c.fake_map_artifact_require_quality_good);
    c.fake_map_artifact_require_completed_pipeline = get_bool(kv, "fake_map_artifact.require_completed_pipeline", c.fake_map_artifact_require_completed_pipeline);
    c.fake_map_artifact_load_enabled = get_bool(kv, "fake_map_artifact.load_enabled", c.fake_map_artifact_load_enabled);
    c.fake_relocalization_enabled = get_bool(kv, "fake_relocalization.enabled", c.fake_relocalization_enabled);
    c.fake_relocalization_allow_pose_writeback = get_bool(kv, "fake_relocalization.allow_pose_writeback", c.fake_relocalization_allow_pose_writeback);
    c.fake_relocalization_require_map_quality_good = get_bool(kv, "fake_relocalization.require_map_quality_good", c.fake_relocalization_require_map_quality_good);
    c.fake_relocalization_require_tof = get_bool(kv, "fake_relocalization.require_tof", c.fake_relocalization_require_tof);
    c.fake_relocalization_min_valid_ranges = get_int(kv, "fake_relocalization.min_valid_ranges", c.fake_relocalization_min_valid_ranges);
    c.fake_relocalization_min_confidence = get_double(kv, "fake_relocalization.min_confidence", c.fake_relocalization_min_confidence);
    c.fake_relocalization_high_confidence_threshold = get_double(kv, "fake_relocalization.high_confidence_threshold", c.fake_relocalization_high_confidence_threshold);
    c.fake_relocalization_min_map_coverage_ratio = get_double(kv, "fake_relocalization.min_map_coverage_ratio", c.fake_relocalization_min_map_coverage_ratio);
    c.fake_relocalization_min_map_yaw_coverage_ratio = get_double(kv, "fake_relocalization.min_map_yaw_coverage_ratio", c.fake_relocalization_min_map_yaw_coverage_ratio);
    c.fake_relocalization_run_on_startup = get_bool(kv, "fake_relocalization.run_on_startup", c.fake_relocalization_run_on_startup);
    c.fake_map_relocalization_runner_enabled = get_bool(kv, "fake_map_relocalization_runner.enabled", c.fake_map_relocalization_runner_enabled);
    c.fake_map_relocalization_runner_require_pipeline_map_artifact = get_bool(kv, "fake_map_relocalization_runner.require_pipeline_map_artifact", c.fake_map_relocalization_runner_require_pipeline_map_artifact);
    c.fake_map_relocalization_runner_require_relocalization_success = get_bool(kv, "fake_map_relocalization_runner.require_relocalization_success", c.fake_map_relocalization_runner_require_relocalization_success);
    c.fake_map_relocalization_runner_require_no_pose_writeback = get_bool(kv, "fake_map_relocalization_runner.require_no_pose_writeback", c.fake_map_relocalization_runner_require_no_pose_writeback);
    c.fake_map_relocalization_runner_run_on_startup = get_bool(kv, "fake_map_relocalization_runner.run_on_startup", c.fake_map_relocalization_runner_run_on_startup);
    c.fake_relocalization_readiness_gate_enabled = get_bool(kv, "fake_relocalization_readiness_gate.enabled", c.fake_relocalization_readiness_gate_enabled);
    c.fake_relocalization_readiness_gate_require_binding_ready = get_bool(kv, "fake_relocalization_readiness_gate.require_binding_ready", c.fake_relocalization_readiness_gate_require_binding_ready);
    c.fake_relocalization_readiness_gate_require_no_pose_writeback = get_bool(kv, "fake_relocalization_readiness_gate.require_no_pose_writeback", c.fake_relocalization_readiness_gate_require_no_pose_writeback);
    c.fake_relocalization_readiness_gate_require_map_quality_good = get_bool(kv, "fake_relocalization_readiness_gate.require_map_quality_good", c.fake_relocalization_readiness_gate_require_map_quality_good);
    c.fake_relocalization_readiness_gate_min_confidence = get_double(kv, "fake_relocalization_readiness_gate.min_confidence", c.fake_relocalization_readiness_gate_min_confidence);
    c.fake_relocalization_readiness_gate_min_map_coverage_ratio = get_double(kv, "fake_relocalization_readiness_gate.min_map_coverage_ratio", c.fake_relocalization_readiness_gate_min_map_coverage_ratio);
    c.fake_relocalization_readiness_gate_min_map_yaw_coverage_ratio = get_double(kv, "fake_relocalization_readiness_gate.min_map_yaw_coverage_ratio", c.fake_relocalization_readiness_gate_min_map_yaw_coverage_ratio);
    c.fake_autonomous_slam_product_acceptance_enabled = get_bool(kv, "fake_autonomous_slam_product_acceptance.enabled", c.fake_autonomous_slam_product_acceptance_enabled);
    c.fake_autonomous_slam_product_acceptance_run_on_startup = get_bool(kv, "fake_autonomous_slam_product_acceptance.run_on_startup", c.fake_autonomous_slam_product_acceptance_run_on_startup);
    c.fake_autonomous_slam_product_acceptance_require_mapping_pipeline_success = get_bool(kv, "fake_autonomous_slam_product_acceptance.require_mapping_pipeline_success", c.fake_autonomous_slam_product_acceptance_require_mapping_pipeline_success);
    c.fake_autonomous_slam_product_acceptance_require_fake_map_saved = get_bool(kv, "fake_autonomous_slam_product_acceptance.require_fake_map_saved", c.fake_autonomous_slam_product_acceptance_require_fake_map_saved);
    c.fake_autonomous_slam_product_acceptance_require_relocalization_success = get_bool(kv, "fake_autonomous_slam_product_acceptance.require_relocalization_success", c.fake_autonomous_slam_product_acceptance_require_relocalization_success);
    c.fake_autonomous_slam_product_acceptance_require_relocalization_readiness = get_bool(kv, "fake_autonomous_slam_product_acceptance.require_relocalization_readiness", c.fake_autonomous_slam_product_acceptance_require_relocalization_readiness);
    c.fake_autonomous_slam_product_acceptance_require_no_pose_writeback = get_bool(kv, "fake_autonomous_slam_product_acceptance.require_no_pose_writeback", c.fake_autonomous_slam_product_acceptance_require_no_pose_writeback);
    c.fake_autonomous_slam_product_acceptance_require_no_forward_backward = get_bool(kv, "fake_autonomous_slam_product_acceptance.require_no_forward_backward", c.fake_autonomous_slam_product_acceptance_require_no_forward_backward);
    c.fake_autonomous_slam_product_acceptance_require_adapter_manifest = get_bool(kv, "fake_autonomous_slam_product_acceptance.require_adapter_manifest", c.fake_autonomous_slam_product_acceptance_require_adapter_manifest);
    c.multi_tof_raw_data_contract_enabled = get_bool(kv, "multi_tof_raw_data_contract.enabled", c.multi_tof_raw_data_contract_enabled);
    c.multi_tof_raw_data_contract_require_front = get_bool(kv, "multi_tof_raw_data_contract.require_front", c.multi_tof_raw_data_contract_require_front);
    c.multi_tof_raw_data_contract_require_left = get_bool(kv, "multi_tof_raw_data_contract.require_left", c.multi_tof_raw_data_contract_require_left);
    c.multi_tof_raw_data_contract_require_right = get_bool(kv, "multi_tof_raw_data_contract.require_right", c.multi_tof_raw_data_contract_require_right);
    c.multi_tof_raw_data_contract_require_unique_mount_ids = get_bool(kv, "multi_tof_raw_data_contract.require_unique_mount_ids", c.multi_tof_raw_data_contract_require_unique_mount_ids);
    c.multi_tof_raw_data_contract_require_unique_frame_ids = get_bool(kv, "multi_tof_raw_data_contract.require_unique_frame_ids", c.multi_tof_raw_data_contract_require_unique_frame_ids);
    c.multi_tof_raw_data_contract_require_request_timing = get_bool(kv, "multi_tof_raw_data_contract.require_request_timing", c.multi_tof_raw_data_contract_require_request_timing);
    c.multi_tof_raw_data_contract_allow_nan_ranges = get_bool(kv, "multi_tof_raw_data_contract.allow_nan_ranges", c.multi_tof_raw_data_contract_allow_nan_ranges);
    c.multi_tof_raw_data_contract_min_required_tof_count = get_int(kv, "multi_tof_raw_data_contract.min_required_tof_count", c.multi_tof_raw_data_contract_min_required_tof_count);
    c.multi_tof_raw_data_contract_front_mount_yaw_rad = get_double(kv, "multi_tof_raw_data_contract.front_mount_yaw_rad", c.multi_tof_raw_data_contract_front_mount_yaw_rad);
    c.multi_tof_raw_data_contract_left_mount_yaw_rad = get_double(kv, "multi_tof_raw_data_contract.left_mount_yaw_rad", c.multi_tof_raw_data_contract_left_mount_yaw_rad);
    c.multi_tof_raw_data_contract_right_mount_yaw_rad = get_double(kv, "multi_tof_raw_data_contract.right_mount_yaw_rad", c.multi_tof_raw_data_contract_right_mount_yaw_rad);
    c.multi_tof_raw_data_contract_max_mount_yaw_error_rad = get_double(kv, "multi_tof_raw_data_contract.max_mount_yaw_error_rad", c.multi_tof_raw_data_contract_max_mount_yaw_error_rad);
    c.multi_tof_raw_data_contract_max_packet_age_s = get_double(kv, "multi_tof_raw_data_contract.max_packet_age_s", c.multi_tof_raw_data_contract_max_packet_age_s);
    c.multi_tof_raw_data_contract_max_request_latency_s = get_double(kv, "multi_tof_raw_data_contract.max_request_latency_s", c.multi_tof_raw_data_contract_max_request_latency_s);
    c.multi_tof_raw_data_contract_max_request_latency_mismatch_s = get_double(kv, "multi_tof_raw_data_contract.max_request_latency_mismatch_s", c.multi_tof_raw_data_contract_max_request_latency_mismatch_s);
    c.multi_tof_raw_data_contract_max_estimated_sample_time_midpoint_error_s = get_double(kv, "multi_tof_raw_data_contract.max_estimated_sample_time_midpoint_error_s", c.multi_tof_raw_data_contract_max_estimated_sample_time_midpoint_error_s);
    c.multi_tof_raw_data_contract_max_future_timestamp_skew_s = get_double(kv, "multi_tof_raw_data_contract.max_future_timestamp_skew_s", c.multi_tof_raw_data_contract_max_future_timestamp_skew_s);
    c.multi_tof_raw_data_contract_max_nan_ratio = get_double(kv, "multi_tof_raw_data_contract.max_nan_ratio", c.multi_tof_raw_data_contract_max_nan_ratio);
    c.multi_tof_raw_data_contract_min_range_m = get_double(kv, "multi_tof_raw_data_contract.min_range_m", c.multi_tof_raw_data_contract_min_range_m);
    c.multi_tof_raw_data_contract_max_range_m = get_double(kv, "multi_tof_raw_data_contract.max_range_m", c.multi_tof_raw_data_contract_max_range_m);
    c.multi_tof_raw_data_contract_front_frame_id = get_string(kv, "multi_tof_raw_data_contract.front_frame_id", c.multi_tof_raw_data_contract_front_frame_id);
    c.multi_tof_raw_data_contract_left_frame_id = get_string(kv, "multi_tof_raw_data_contract.left_frame_id", c.multi_tof_raw_data_contract_left_frame_id);
    c.multi_tof_raw_data_contract_right_frame_id = get_string(kv, "multi_tof_raw_data_contract.right_frame_id", c.multi_tof_raw_data_contract_right_frame_id);
    c.multi_tof_raw_data_contract_run_acceptance_on_startup = get_bool(kv, "multi_tof_raw_data_contract.run_acceptance_on_startup", c.multi_tof_raw_data_contract_run_acceptance_on_startup);
    c.multi_tof_sync_enabled = get_bool(kv, "multi_tof_sync.enabled", c.multi_tof_sync_enabled);
    c.multi_tof_sync_require_raw_contract_pass = get_bool(kv, "multi_tof_sync.require_raw_contract_pass", c.multi_tof_sync_require_raw_contract_pass);
    c.multi_tof_sync_require_imu = get_bool(kv, "multi_tof_sync.require_imu", c.multi_tof_sync_require_imu);
    c.multi_tof_sync_require_wheel = get_bool(kv, "multi_tof_sync.require_wheel", c.multi_tof_sync_require_wheel);
    c.multi_tof_sync_time_reference = get_string(kv, "multi_tof_sync.time_reference", c.multi_tof_sync_time_reference);
    c.multi_tof_sync_degradation_mode = get_string(kv, "multi_tof_sync.degradation_mode", c.multi_tof_sync_degradation_mode);
    c.multi_tof_sync_min_required_tof_count = get_int(kv, "multi_tof_sync.min_required_tof_count", c.multi_tof_sync_min_required_tof_count);
    c.multi_tof_sync_max_pairwise_tof_sync_dt_s = get_double(kv, "multi_tof_sync.max_pairwise_tof_sync_dt_s", c.multi_tof_sync_max_pairwise_tof_sync_dt_s);
    c.multi_tof_sync_max_multi_tof_imu_sync_dt_s = get_double(kv, "multi_tof_sync.max_multi_tof_imu_sync_dt_s", c.multi_tof_sync_max_multi_tof_imu_sync_dt_s);
    c.multi_tof_sync_max_multi_tof_wheel_sync_dt_s = get_double(kv, "multi_tof_sync.max_multi_tof_wheel_sync_dt_s", c.multi_tof_sync_max_multi_tof_wheel_sync_dt_s);
    c.multi_tof_sync_max_effective_time_future_skew_s = get_double(kv, "multi_tof_sync.max_effective_time_future_skew_s", c.multi_tof_sync_max_effective_time_future_skew_s);
    c.multi_tof_sync_run_acceptance_on_startup = get_bool(kv, "multi_tof_sync.run_acceptance_on_startup", c.multi_tof_sync_run_acceptance_on_startup);
    c.multi_tof_snapshot_builder_enabled = get_bool(kv, "multi_tof_snapshot_builder.enabled", c.multi_tof_snapshot_builder_enabled);
    c.multi_tof_snapshot_builder_require_sync_pass = get_bool(kv, "multi_tof_snapshot_builder.require_sync_pass", c.multi_tof_snapshot_builder_require_sync_pass);
    c.multi_tof_snapshot_builder_populate_legacy_tof = get_bool(kv, "multi_tof_snapshot_builder.populate_legacy_tof", c.multi_tof_snapshot_builder_populate_legacy_tof);
    c.multi_tof_snapshot_builder_require_legacy_primary = get_bool(kv, "multi_tof_snapshot_builder.require_legacy_primary", c.multi_tof_snapshot_builder_require_legacy_primary);
    c.multi_tof_snapshot_builder_legacy_primary_mode = get_string(kv, "multi_tof_snapshot_builder.legacy_primary_mode", c.multi_tof_snapshot_builder_legacy_primary_mode);
    c.multi_tof_snapshot_builder_min_required_tof_count = get_int(kv, "multi_tof_snapshot_builder.min_required_tof_count", c.multi_tof_snapshot_builder_min_required_tof_count);
    c.multi_tof_snapshot_builder_run_acceptance_on_startup = get_bool(kv, "multi_tof_snapshot_builder.run_acceptance_on_startup", c.multi_tof_snapshot_builder_run_acceptance_on_startup);
    c.multi_tof_replay_enabled = get_bool(kv, "multi_tof_replay.enabled", c.multi_tof_replay_enabled);
    c.multi_tof_replay_reject_invalid_records = get_bool(kv, "multi_tof_replay.reject_invalid_records", c.multi_tof_replay_reject_invalid_records);
    c.multi_tof_replay_require_snapshot_build = get_bool(kv, "multi_tof_replay.require_snapshot_build", c.multi_tof_replay_require_snapshot_build);
    c.multi_tof_replay_time_mode = get_string(kv, "multi_tof_replay.time_mode", c.multi_tof_replay_time_mode);
    c.multi_tof_replay_run_acceptance_on_startup = get_bool(kv, "multi_tof_replay.run_acceptance_on_startup", c.multi_tof_replay_run_acceptance_on_startup);
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
        non_negative("encoder.max_pair_skew_us", c.encoder_max_pair_skew_us);
        non_negative("encoder.max_read_latency_us", c.encoder_max_read_latency_us);
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
    if (c.confidence_min < 0 || c.confidence_min > 100) errors.push_back("tof.confidence_min must be in [0, 100]");
    if (!std::isfinite(c.tof_protocol_min_range_m) || c.tof_protocol_min_range_m <= 0.0) errors.push_back("tof.protocol_min_range_m must be finite and > 0");
    if (!std::isfinite(c.tof_protocol_max_range_m) || c.tof_protocol_max_range_m <= c.tof_protocol_min_range_m) errors.push_back("tof.protocol_max_range_m must be > protocol_min_range_m");
    if (!std::isfinite(c.tof_mapping_min_range_m) || c.tof_mapping_min_range_m < c.tof_protocol_min_range_m) errors.push_back("tof.mapping_min_range_m must be >= protocol_min_range_m");
    if (!std::isfinite(c.tof_mapping_max_range_m) || c.tof_mapping_max_range_m > c.tof_protocol_max_range_m || c.tof_mapping_max_range_m <= c.tof_mapping_min_range_m) errors.push_back("tof.mapping_max_range_m must be <= protocol_max_range_m and > mapping_min_range_m");
    if (c.tof_mapping_min_confidence < 0 || c.tof_mapping_min_confidence > 100) errors.push_back("tof.mapping_min_confidence must be in [0, 100]");
    if (!std::isfinite(c.tof_full_fov_deg) || c.tof_full_fov_deg <= 0.0) errors.push_back("tof.full_fov_deg must be finite and > 0");
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

    if (!one_of(c.slam_runtime_mode, {"legacy", "sparse_shadow"})) errors.push_back("slam_runtime.mode must be legacy or sparse_shadow");
    if (!one_of(c.sparse_shadow_sensor_source, {"deterministic_simulation", "hardware", "replay"})) errors.push_back("slam_runtime.sparse_shadow_sensor_source must be deterministic_simulation, replay, or hardware");
    if (c.slam_runtime_mode == "sparse_shadow" && c.sparse_shadow_sensor_source != "deterministic_simulation") errors.push_back("SparseShadow supports only deterministic_simulation source in M3-D1.1; hardware and replay are fail-closed");
    if (!one_of(c.sparse_slam_map_startup_mode, {"create_new", "load_existing"})) errors.push_back("sparse_slam.map_startup_mode must be create_new or load_existing");
    if (!one_of(c.sparse_slam_initial_pose_mode, {"startup_zero", "configured_pose", "relocalization"})) errors.push_back("sparse_slam.initial_pose_mode must be startup_zero, configured_pose, or relocalization");
    if (c.sparse_slam_initial_pose_mode == "configured_pose" && !c.sparse_slam_has_configured_pose) errors.push_back("sparse_slam configured_pose mode requires has_configured_pose=true");
    if (c.sparse_slam_initial_pose_mode == "startup_zero" && c.sparse_slam_has_configured_pose) errors.push_back("sparse_slam startup_zero must not also configure a map pose");
    if (!std::isfinite(c.sparse_slam_configured_pose_x_m)) errors.push_back("sparse_slam.configured_pose_x_m must be finite");
    if (!std::isfinite(c.sparse_slam_configured_pose_y_m)) errors.push_back("sparse_slam.configured_pose_y_m must be finite");
    if (!std::isfinite(c.sparse_slam_configured_pose_yaw_rad)) errors.push_back("sparse_slam.configured_pose_yaw_rad must be finite");
    if (c.sparse_slam_pose_buffer_capacity <= 1) errors.push_back("sparse_slam.pose_buffer_capacity must be > 1");
    if (c.sparse_slam_pose_buffer_capacity > 4096) errors.push_back("sparse_slam.pose_buffer_capacity must be <= 4096");
    if (!std::isfinite(c.sparse_slam_pose_buffer_max_age_s) || c.sparse_slam_pose_buffer_max_age_s < 0.0) errors.push_back("sparse_slam.pose_buffer_max_age_s must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_pose_interpolation_max_gap_s) || c.sparse_slam_pose_interpolation_max_gap_s < 0.0) errors.push_back("sparse_slam.pose_interpolation_max_gap_s must be finite and >= 0");
    if (c.sparse_slam_startup_min_imu_samples <= 0) errors.push_back("sparse_slam.startup_min_imu_samples must be > 0");
    if (!std::isfinite(c.sparse_slam_startup_max_abs_linear_speed_mps) || c.sparse_slam_startup_max_abs_linear_speed_mps < 0.0) errors.push_back("sparse_slam.startup_max_abs_linear_speed_mps must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_startup_max_abs_wheel_yaw_rate_rad_s) || c.sparse_slam_startup_max_abs_wheel_yaw_rate_rad_s < 0.0) errors.push_back("sparse_slam.startup_max_abs_wheel_yaw_rate_rad_s must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_startup_max_abs_imu_yaw_rate_rad_s) || c.sparse_slam_startup_max_abs_imu_yaw_rate_rad_s < 0.0) errors.push_back("sparse_slam.startup_max_abs_imu_yaw_rate_rad_s must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_startup_max_gyro_bias_abs_rad_s) || c.sparse_slam_startup_max_gyro_bias_abs_rad_s < 0.0) errors.push_back("sparse_slam.startup_max_gyro_bias_abs_rad_s must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_startup_max_gyro_spread_rad_s) || c.sparse_slam_startup_max_gyro_spread_rad_s < 0.0) errors.push_back("sparse_slam.startup_max_gyro_spread_rad_s must be finite and >= 0");
    if (c.sparse_slam_active_bundle_max_snapshots <= 0) errors.push_back("sparse_slam.active_bundle_max_snapshots must be > 0");
    if (c.sparse_slam_active_bundle_max_snapshots > 1024) errors.push_back("sparse_slam.active_bundle_max_snapshots must be <= 1024");
    if (c.sparse_slam_active_bundle_max_rays <= 0) errors.push_back("sparse_slam.active_bundle_max_rays must be > 0");
    if (c.sparse_slam_active_bundle_max_rays > 3072) errors.push_back("sparse_slam.active_bundle_max_rays must be <= 3072");
    if (c.sparse_slam_active_bundle_max_rays < c.sparse_slam_active_bundle_max_snapshots * 3) errors.push_back("sparse_slam.active_bundle_max_rays must cover max_snapshots * 3");
    if (!std::isfinite(c.sparse_slam_active_bundle_max_duration_s) || c.sparse_slam_active_bundle_max_duration_s <= 0.0) errors.push_back("sparse_slam.active_bundle_max_duration_s must be finite and > 0");
    if (!std::isfinite(c.sparse_slam_active_bundle_max_snapshot_gap_s) || c.sparse_slam_active_bundle_max_snapshot_gap_s <= 0.0) errors.push_back("sparse_slam.active_bundle_max_snapshot_gap_s must be finite and > 0");
    if (c.sparse_slam_active_bundle_min_snapshots < 0) errors.push_back("sparse_slam.active_bundle_min_snapshots must be >= 0");
    if (c.sparse_slam_active_bundle_min_snapshots > c.sparse_slam_active_bundle_max_snapshots) errors.push_back("sparse_slam.active_bundle_min_snapshots must be <= max_snapshots");
    if (c.sparse_slam_active_bundle_min_matchable_rays < 0) errors.push_back("sparse_slam.active_bundle_min_matchable_rays must be >= 0");
    if (c.sparse_slam_active_bundle_min_matchable_rays > c.sparse_slam_active_bundle_max_rays) errors.push_back("sparse_slam.active_bundle_min_matchable_rays must be <= max_rays");
    if (!std::isfinite(c.sparse_slam_active_bundle_min_yaw_span_rad) || c.sparse_slam_active_bundle_min_yaw_span_rad < 0.0 || c.sparse_slam_active_bundle_min_yaw_span_rad > 6.28318530717958647692) errors.push_back("sparse_slam.active_bundle_min_yaw_span_rad must be finite and within [0, 2pi]");
    if (!std::isfinite(c.sparse_slam_settle_max_abs_linear_speed_mps) || c.sparse_slam_settle_max_abs_linear_speed_mps < 0.0) errors.push_back("sparse_slam.settle_max_abs_linear_speed_mps must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s) || c.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s < 0.0) errors.push_back("sparse_slam.settle_max_abs_wheel_yaw_rate_rad_s must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s) || c.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s < 0.0) errors.push_back("sparse_slam.settle_max_abs_imu_yaw_rate_rad_s must be finite and >= 0");
    if (c.sparse_slam_settle_min_consecutive_samples <= 0) errors.push_back("sparse_slam.settle_min_consecutive_samples must be > 0");
    if (!std::isfinite(c.sparse_slam_settle_min_stable_duration_s) || c.sparse_slam_settle_min_stable_duration_s < 0.0) errors.push_back("sparse_slam.settle_min_stable_duration_s must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_settle_max_sample_gap_s) || c.sparse_slam_settle_max_sample_gap_s <= 0.0) errors.push_back("sparse_slam.settle_max_sample_gap_s must be finite and > 0");
    if (c.sparse_slam_reference_snapshot_max_cells <= 0 || c.sparse_slam_reference_snapshot_max_cells > 100000) errors.push_back("sparse_slam.reference_snapshot_max_cells must be in [1, 100000]");
    if (!one_of(c.sparse_slam_local_match_mode, {"yaw_only", "full_se2"})) errors.push_back("sparse_slam.local_match_mode must be yaw_only or full_se2");
    if (!std::isfinite(c.sparse_slam_local_match_max_abs_yaw_rad) || c.sparse_slam_local_match_max_abs_yaw_rad < 0.0 || c.sparse_slam_local_match_max_abs_yaw_rad > 3.14159265358979323846) errors.push_back("sparse_slam.local_match_max_abs_yaw_rad must be finite and within [0, pi]");
    if (!std::isfinite(c.sparse_slam_local_match_coarse_yaw_step_rad) || c.sparse_slam_local_match_coarse_yaw_step_rad <= 0.0) errors.push_back("sparse_slam.local_match_coarse_yaw_step_rad must be finite and > 0");
    if (!std::isfinite(c.sparse_slam_local_match_fine_yaw_step_rad) || c.sparse_slam_local_match_fine_yaw_step_rad <= 0.0 || c.sparse_slam_local_match_fine_yaw_step_rad > c.sparse_slam_local_match_coarse_yaw_step_rad) errors.push_back("sparse_slam.local_match_fine_yaw_step_rad must be in (0, coarse_step]");
    if (!std::isfinite(c.sparse_slam_local_match_max_abs_translation_x_m) || c.sparse_slam_local_match_max_abs_translation_x_m < 0.0) errors.push_back("sparse_slam.local_match_max_abs_translation_x_m must be finite and >= 0");
    if (!std::isfinite(c.sparse_slam_local_match_max_abs_translation_y_m) || c.sparse_slam_local_match_max_abs_translation_y_m < 0.0) errors.push_back("sparse_slam.local_match_max_abs_translation_y_m must be finite and >= 0");
    if (c.sparse_slam_local_match_mode == "yaw_only" && (c.sparse_slam_local_match_max_abs_translation_x_m != 0.0 || c.sparse_slam_local_match_max_abs_translation_y_m != 0.0)) errors.push_back("sparse_slam yaw_only mode requires zero translation ranges");
    if (c.sparse_slam_local_match_max_candidate_count <= 0 || c.sparse_slam_local_match_max_candidate_count > 100000) errors.push_back("sparse_slam.local_match_max_candidate_count must be in [1, 100000]");
    if (c.sparse_slam_local_match_min_bundle_frames <= 0) errors.push_back("sparse_slam.local_match_min_bundle_frames must be > 0");
    if (c.sparse_slam_local_match_min_matchable_rays <= 0) errors.push_back("sparse_slam.local_match_min_matchable_rays must be > 0");
    if (c.sparse_slam_local_match_min_reference_cells <= 0) errors.push_back("sparse_slam.local_match_min_reference_cells must be > 0");
    if (c.sparse_slam_local_match_min_reference_occupied_cells < 0 || c.sparse_slam_local_match_min_reference_occupied_cells > c.sparse_slam_local_match_min_reference_cells) errors.push_back("sparse_slam.local_match_min_reference_occupied_cells invalid");
    if (c.sparse_slam_local_match_min_reference_free_cells < 0 || c.sparse_slam_local_match_min_reference_free_cells > c.sparse_slam_local_match_min_reference_cells) errors.push_back("sparse_slam.local_match_min_reference_free_cells invalid");
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

    if (!one_of(c.active_scan_execution_mode, {"disabled", "dry_run", "command_log_only"})) errors.push_back("active_scan_execution.mode must be disabled, dry_run, or command_log_only; hardware execution unsupported in stage4");
    if (c.active_scan_execution_mode == "hardware") errors.push_back("active_scan_execution.mode=hardware unsupported in stage4");
    if (c.active_scan_execution_hardware_write_enabled) errors.push_back("active_scan_execution.hardware_write_enabled=true unsupported in stage4");
    positive("active_scan_execution.log_hz", c.active_scan_execution_log_hz);
    non_negative("active_scan_execution.target_scan_angle_deg", c.active_scan_execution_target_scan_angle_deg);
    non_negative("active_scan_execution.complete_scan_angle_deg", c.active_scan_execution_complete_scan_angle_deg);
    non_negative("active_scan_execution.min_useful_scan_angle_deg", c.active_scan_execution_min_useful_scan_angle_deg);
    positive("active_scan_execution.target_yaw_rate_dps", c.active_scan_execution_target_yaw_rate_dps);
    non_negative("active_scan_execution.min_observed_yaw_rate_dps", c.active_scan_execution_min_observed_yaw_rate_dps);
    non_negative("active_scan_execution.max_observed_yaw_rate_dps", c.active_scan_execution_max_observed_yaw_rate_dps);
    non_negative("active_scan_execution.max_linear_speed_mps", c.active_scan_execution_max_linear_speed_mps);
    non_negative("active_scan_execution.precheck_hold_s", c.active_scan_execution_precheck_hold_s);
    non_negative("active_scan_execution.command_timeout_s", c.active_scan_execution_command_timeout_s);
    non_negative("active_scan_execution.cooldown_s", c.active_scan_execution_cooldown_s);
    if (c.active_scan_execution_complete_scan_angle_deg < c.active_scan_execution_min_useful_scan_angle_deg) errors.push_back("active_scan_execution.complete_scan_angle_deg must be >= min_useful_scan_angle_deg");
    if (c.active_scan_execution_target_scan_angle_deg < c.active_scan_execution_min_useful_scan_angle_deg) errors.push_back("active_scan_execution.target_scan_angle_deg must be >= min_useful_scan_angle_deg");
    if (c.active_scan_execution_max_observed_yaw_rate_dps < c.active_scan_execution_min_observed_yaw_rate_dps) errors.push_back("active_scan_execution.max_observed_yaw_rate_dps must be >= min_observed_yaw_rate_dps");

    positive("sparse_scan.log_hz", c.sparse_scan_log_hz);
    non_negative("sparse_scan.min_yaw_rate_dps", c.sparse_scan_min_yaw_rate_dps);
    non_negative("sparse_scan.max_yaw_rate_dps", c.sparse_scan_max_yaw_rate_dps);
    if (c.sparse_scan_max_yaw_rate_dps < c.sparse_scan_min_yaw_rate_dps) errors.push_back("sparse_scan.max_yaw_rate_dps must be >= min_yaw_rate_dps");
    non_negative("sparse_scan.max_linear_speed_mps", c.sparse_scan_max_linear_speed_mps);
    non_negative("sparse_scan.min_range_m", c.sparse_scan_min_range_m);
    positive("sparse_scan.max_range_m", c.sparse_scan_max_range_m);
    if (c.sparse_scan_max_range_m <= c.sparse_scan_min_range_m) errors.push_back("sparse_scan.max_range_m must be > min_range_m");
    if (c.sparse_scan_min_confidence < 0) errors.push_back("sparse_scan.min_confidence must be >= 0");
    positive("sparse_scan.bin_angle_deg", c.sparse_scan_bin_angle_deg);
    if (c.sparse_scan_bin_angle_deg > 90.0) errors.push_back("sparse_scan.bin_angle_deg must be <= 90");
    non_negative("sparse_scan.min_session_angle_deg", c.sparse_scan_min_session_angle_deg);
    non_negative("sparse_scan.useful_session_angle_deg", c.sparse_scan_useful_session_angle_deg);
    non_negative("sparse_scan.complete_session_angle_deg", c.sparse_scan_complete_session_angle_deg);
    if (c.sparse_scan_complete_session_angle_deg < c.sparse_scan_useful_session_angle_deg) errors.push_back("sparse_scan.complete_session_angle_deg must be >= useful_session_angle_deg");
    if (c.sparse_scan_useful_session_angle_deg < c.sparse_scan_min_session_angle_deg) errors.push_back("sparse_scan.useful_session_angle_deg must be >= min_session_angle_deg");
    positive("sparse_scan.session_timeout_s", c.sparse_scan_session_timeout_s);
    positive("sparse_scan.max_gap_s", c.sparse_scan_max_gap_s);
    non_negative("sparse_scan.cooldown_s", c.sparse_scan_cooldown_s);
    if (c.sparse_scan_max_samples_per_session <= 0) errors.push_back("sparse_scan.max_samples_per_session must be > 0");

    positive("sparse_scan_yaw_match.log_hz", c.sparse_scan_yaw_match_log_hz);
    positive("sparse_scan_yaw_match.max_yaw_search_deg", c.sparse_scan_yaw_match_max_yaw_search_deg);
    positive("sparse_scan_yaw_match.coarse_step_deg", c.sparse_scan_yaw_match_coarse_step_deg);
    if (c.sparse_scan_yaw_match_coarse_step_deg > c.sparse_scan_yaw_match_max_yaw_search_deg) errors.push_back("sparse_scan_yaw_match.coarse_step_deg must be <= max_yaw_search_deg");
    non_negative("sparse_scan_yaw_match.refine_window_deg", c.sparse_scan_yaw_match_refine_window_deg);
    positive("sparse_scan_yaw_match.refine_step_deg", c.sparse_scan_yaw_match_refine_step_deg);
    if (c.sparse_scan_yaw_match_min_valid_samples < 0) errors.push_back("sparse_scan_yaw_match.min_valid_samples must be >= 0");
    if (c.sparse_scan_yaw_match_min_valid_bins < 0) errors.push_back("sparse_scan_yaw_match.min_valid_bins must be >= 0");
    non_negative("sparse_scan_yaw_match.min_observed_yaw_delta_deg", c.sparse_scan_yaw_match_min_observed_yaw_delta_deg);
    probability("sparse_scan_yaw_match.min_valid_bin_ratio", c.sparse_scan_yaw_match_min_valid_bin_ratio);
    non_negative("sparse_scan_yaw_match.occupied_search_radius_m", c.sparse_scan_yaw_match_occupied_search_radius_m);
    if (!std::isfinite(c.sparse_scan_yaw_match_occupied_hit_reward)) errors.push_back("sparse_scan_yaw_match.occupied_hit_reward must be finite");
    if (!std::isfinite(c.sparse_scan_yaw_match_free_hit_penalty)) errors.push_back("sparse_scan_yaw_match.free_hit_penalty must be finite");
    if (!std::isfinite(c.sparse_scan_yaw_match_unknown_hit_penalty)) errors.push_back("sparse_scan_yaw_match.unknown_hit_penalty must be finite");
    positive("sparse_scan_yaw_match.distance_penalty_scale_m", c.sparse_scan_yaw_match_distance_penalty_scale_m);
    if (!std::isfinite(c.sparse_scan_yaw_match_min_best_score) || c.sparse_scan_yaw_match_min_best_score < -10.0 || c.sparse_scan_yaw_match_min_best_score > 10.0) errors.push_back("sparse_scan_yaw_match.min_best_score must be finite and in [-10, 10]");
    non_negative("sparse_scan_yaw_match.min_score_margin", c.sparse_scan_yaw_match_min_score_margin);
    probability("sparse_scan_yaw_match.min_inlier_ratio", c.sparse_scan_yaw_match_min_inlier_ratio);
    probability("sparse_scan_yaw_match.max_curve_flatness", c.sparse_scan_yaw_match_max_curve_flatness);
    non_negative("sparse_scan_yaw_match.multimodal_peak_separation_deg", c.sparse_scan_yaw_match_multimodal_peak_separation_deg);
    probability("sparse_scan_yaw_match.max_second_peak_ratio", c.sparse_scan_yaw_match_max_second_peak_ratio);
    if (c.sparse_scan_yaw_match_max_samples_per_match <= 0) errors.push_back("sparse_scan_yaw_match.max_samples_per_match must be > 0");

    if (!one_of(c.yaw_correction_mode, {"disabled", "dry_run", "writeback"})) errors.push_back("yaw_correction.mode must be disabled, dry_run, or writeback");
    if (c.yaw_correction_mode == "writeback" && !c.yaw_correction_writeback_enabled) errors.push_back("yaw_correction.mode=writeback requires writeback_enabled=true");
    if (c.yaw_correction_writeback_enabled && c.yaw_correction_writeback_acknowledgement != c.yaw_correction_required_writeback_acknowledgement) errors.push_back("yaw_correction.writeback_acknowledgement must match required_writeback_acknowledgement when writeback_enabled=true");
    if (c.yaw_correction_mode == "writeback" && c.yaw_correction_writeback_acknowledgement != c.yaw_correction_required_writeback_acknowledgement) errors.push_back("yaw_correction.mode=writeback requires matching writeback acknowledgement");
    if (!one_of(c.yaw_correction_scan_completion_source, {"active_scan_only", "yaw_match_only", "either"})) errors.push_back("yaw_correction.scan_completion_source must be active_scan_only, yaw_match_only, or either");
    positive("yaw_correction.log_hz", c.yaw_correction_log_hz);
    non_negative("yaw_correction.max_linear_speed_mps", c.yaw_correction_max_linear_speed_mps);
    non_negative("yaw_correction.max_abs_yaw_rate_dps", c.yaw_correction_max_abs_yaw_rate_dps);
    if (!std::isfinite(c.yaw_correction_min_best_score) || c.yaw_correction_min_best_score < -10.0 || c.yaw_correction_min_best_score > 10.0) errors.push_back("yaw_correction.min_best_score must be finite and in [-10, 10]");
    non_negative("yaw_correction.min_score_margin", c.yaw_correction_min_score_margin);
    probability("yaw_correction.min_inlier_ratio", c.yaw_correction_min_inlier_ratio);
    probability("yaw_correction.max_curve_flatness", c.yaw_correction_max_curve_flatness);
    positive("yaw_correction.max_candidate_abs_deg", c.yaw_correction_max_candidate_abs_deg);
    non_negative("yaw_correction.min_candidate_abs_deg", c.yaw_correction_min_candidate_abs_deg);
    if (c.yaw_correction_min_candidate_abs_deg > c.yaw_correction_max_candidate_abs_deg) errors.push_back("yaw_correction.min_candidate_abs_deg must be <= max_candidate_abs_deg");
    if (!std::isfinite(c.yaw_correction_gain) || c.yaw_correction_gain <= 0.0 || c.yaw_correction_gain > 1.0) errors.push_back("yaw_correction.correction_gain must be > 0 and <= 1");
    positive("yaw_correction.max_step_deg", c.yaw_correction_max_step_deg);
    non_negative("yaw_correction.min_step_deg", c.yaw_correction_min_step_deg);
    if (c.yaw_correction_min_step_deg > c.yaw_correction_max_step_deg) errors.push_back("yaw_correction.min_step_deg must be <= max_step_deg");
    if (c.yaw_correction_consistency_window < 1) errors.push_back("yaw_correction.consistency_window must be >= 1");
    non_negative("yaw_correction.max_consistency_spread_deg", c.yaw_correction_max_consistency_spread_deg);
    non_negative("yaw_correction.cooldown_s", c.yaw_correction_cooldown_s);
    non_negative("yaw_correction.min_match_observed_yaw_delta_deg", c.yaw_correction_min_match_observed_yaw_delta_deg);
    if (c.yaw_correction_min_match_valid_samples < 0) errors.push_back("yaw_correction.min_match_valid_samples must be >= 0");
    if (c.yaw_correction_min_match_valid_bins < 0) errors.push_back("yaw_correction.min_match_valid_bins must be >= 0");
    probability("yaw_correction.min_match_valid_bin_ratio", c.yaw_correction_min_match_valid_bin_ratio);
    positive("yaw_correction.max_writeback_abs_deg", c.yaw_correction_max_writeback_abs_deg);
    positive("yaw_correction.max_total_writeback_per_session_deg", c.yaw_correction_max_total_writeback_per_session_deg);
    if (c.yaw_correction_max_writeback_count_per_session <= 0) errors.push_back("yaw_correction.max_writeback_count_per_session must be > 0");
    non_negative("yaw_correction.min_seconds_between_writebacks", c.yaw_correction_min_seconds_between_writebacks);
    positive("yaw_correction_post_apply.timeout_s", c.yaw_correction_post_apply_timeout_s);
    non_negative("yaw_correction_post_apply.min_improvement_deg", c.yaw_correction_post_apply_min_improvement_deg);
    non_negative("yaw_correction_post_apply.min_improvement_fraction_of_applied_delta", c.yaw_correction_post_apply_min_improvement_fraction_of_applied_delta);
    non_negative("yaw_correction_post_apply.min_absolute_improvement_deg", c.yaw_correction_post_apply_min_absolute_improvement_deg);
    non_negative("yaw_correction_post_apply.max_allowed_worse_deg", c.yaw_correction_post_apply_max_allowed_worse_deg);
    positive("yaw_correction_post_apply.max_post_apply_candidate_abs_deg", c.yaw_correction_post_apply_max_post_apply_candidate_abs_deg);
    if (!one_of(c.recovery_mode, {"observe_only", "disabled"})) errors.push_back("recovery.mode must be observe_only or disabled");
    positive("recovery.log_hz", c.recovery_log_hz);
    non_negative("recovery.startup_grace_s", c.recovery_startup_grace_s);
    non_negative("recovery.lost_confirm_s", c.recovery_lost_confirm_s);
    non_negative("recovery.degraded_confirm_s", c.recovery_degraded_confirm_s);
    if (c.recovery_require_min_known_cells < 0) errors.push_back("recovery.require_min_known_cells must be >= 0");
    if (c.recovery_require_min_occupied_cells < 0) errors.push_back("recovery.require_min_occupied_cells must be >= 0");
    non_negative("recovery.max_tof_age_s", c.recovery_max_tof_age_s);
    non_negative("recovery.recovery_scan_cooldown_s", c.recovery_recovery_scan_cooldown_s);
    if (c.recovery_max_recent_yaw_apply_count < 0) errors.push_back("recovery.max_recent_yaw_apply_count must be >= 0");
    if (c.recovery_min_post_apply_validated_count < 0) errors.push_back("recovery.min_post_apply_validated_count must be >= 0");
    validate_motion_execution_config(c, errors);
    if (c.autonomous_slam_max_iterations <= 0) {
        errors.push_back("autonomous_slam.max_iterations must be > 0");
    } else if (c.autonomous_slam_max_iterations > 1000) {
        errors.push_back("autonomous_slam.max_iterations must be <= 1000");
    }
    if (!std::isfinite(c.autonomous_slam_sensor_timeout_s) ||
        c.autonomous_slam_sensor_timeout_s <= 0.0) {
        errors.push_back("autonomous_slam.sensor_timeout_s must be > 0");
    } else if (c.autonomous_slam_sensor_timeout_s > 5.0) {
        errors.push_back("autonomous_slam.sensor_timeout_s must be <= 5.0");
    }
    if (!std::isfinite(c.autonomous_slam_motion_settle_timeout_s) ||
        c.autonomous_slam_motion_settle_timeout_s <= 0.0) {
        errors.push_back("autonomous_slam.motion_settle_timeout_s must be > 0");
    } else if (c.autonomous_slam_motion_settle_timeout_s > 10.0) {
        errors.push_back("autonomous_slam.motion_settle_timeout_s must be <= 10.0");
    }
    if (!std::isfinite(c.autonomous_slam_active_scan_speed) ||
        c.autonomous_slam_active_scan_speed <= 0.0) {
        errors.push_back("autonomous_slam.active_scan_speed must be > 0");
    } else if (c.autonomous_slam_active_scan_speed > 0.05) {
        errors.push_back("autonomous_slam.active_scan_speed must be <= 0.05");
    }
    if (!std::isfinite(c.autonomous_slam_active_scan_duration_s) ||
        c.autonomous_slam_active_scan_duration_s <= 0.0) {
        errors.push_back("autonomous_slam.active_scan_duration_s must be > 0");
    } else if (c.autonomous_slam_active_scan_duration_s > 0.50) {
        errors.push_back("autonomous_slam.active_scan_duration_s must be <= 0.50");
    }
    if (c.autonomous_slam_max_active_scan_commands <= 0) {
        errors.push_back("autonomous_slam.max_active_scan_commands must be > 0");
    } else if (c.autonomous_slam_max_active_scan_commands > 100) {
        errors.push_back("autonomous_slam.max_active_scan_commands must be <= 100");
    }
    if (c.autonomous_slam_allow_forward_backward) {
        errors.push_back("autonomous_slam.allow_forward_backward=true unsupported before grounded live validation");
    }
    if (!std::isfinite(c.real_adapter_contract_tof_max_frame_age_s) ||
        c.real_adapter_contract_tof_max_frame_age_s <= 0.0) {
        errors.push_back("real_adapter_contract.tof_max_frame_age_s must be > 0");
    } else if (c.real_adapter_contract_tof_max_frame_age_s > 5.0) {
        errors.push_back("real_adapter_contract.tof_max_frame_age_s must be <= 5.0");
    }
    if (!std::isfinite(c.real_adapter_contract_imu_max_frame_age_s) ||
        c.real_adapter_contract_imu_max_frame_age_s <= 0.0) {
        errors.push_back("real_adapter_contract.imu_max_frame_age_s must be > 0");
    } else if (c.real_adapter_contract_imu_max_frame_age_s > 5.0) {
        errors.push_back("real_adapter_contract.imu_max_frame_age_s must be <= 5.0");
    }
    if (!std::isfinite(c.real_adapter_contract_wheel_max_frame_age_s) ||
        c.real_adapter_contract_wheel_max_frame_age_s <= 0.0) {
        errors.push_back("real_adapter_contract.wheel_max_frame_age_s must be > 0");
    } else if (c.real_adapter_contract_wheel_max_frame_age_s > 5.0) {
        errors.push_back("real_adapter_contract.wheel_max_frame_age_s must be <= 5.0");
    }
    if (c.real_adapter_contract_tof_min_range_count < 1) {
        errors.push_back("real_adapter_contract.tof_min_range_count must be >= 1");
    }
    if (c.real_adapter_contract_tof_max_range_count < c.real_adapter_contract_tof_min_range_count) {
        errors.push_back("real_adapter_contract.tof_max_range_count must be >= tof_min_range_count");
    } else if (c.real_adapter_contract_tof_max_range_count > 65536) {
        errors.push_back("real_adapter_contract.tof_max_range_count must be <= 65536");
    }
    if (!std::isfinite(c.real_adapter_contract_tof_min_range_m) ||
        !std::isfinite(c.real_adapter_contract_tof_max_range_m) ||
        c.real_adapter_contract_tof_min_range_m <= 0.0 ||
        c.real_adapter_contract_tof_min_range_m >= c.real_adapter_contract_tof_max_range_m) {
        errors.push_back("real_adapter_contract.tof_min_range_m must be > 0 and < tof_max_range_m");
    } else if (c.real_adapter_contract_tof_max_range_m > 100.0) {
        errors.push_back("real_adapter_contract.tof_max_range_m must be <= 100.0");
    }
    if (!std::isfinite(c.real_adapter_contract_tof_max_allowed_nan_ratio) ||
        c.real_adapter_contract_tof_max_allowed_nan_ratio < 0.0 ||
        c.real_adapter_contract_tof_max_allowed_nan_ratio > 1.0) {
        errors.push_back("real_adapter_contract.tof_max_allowed_nan_ratio must be in [0,1]");
    }
    if (!std::isfinite(c.real_adapter_contract_imu_max_abs_yaw_rate_rad_s) ||
        c.real_adapter_contract_imu_max_abs_yaw_rate_rad_s <= 0.0) {
        errors.push_back("real_adapter_contract.imu_max_abs_yaw_rate_rad_s must be > 0");
    } else if (c.real_adapter_contract_imu_max_abs_yaw_rate_rad_s > 100.0) {
        errors.push_back("real_adapter_contract.imu_max_abs_yaw_rate_rad_s must be <= 100.0");
    }
    if (!std::isfinite(c.real_adapter_contract_imu_max_abs_acc_mps2) ||
        c.real_adapter_contract_imu_max_abs_acc_mps2 <= 0.0) {
        errors.push_back("real_adapter_contract.imu_max_abs_acc_mps2 must be > 0");
    } else if (c.real_adapter_contract_imu_max_abs_acc_mps2 > 200.0) {
        errors.push_back("real_adapter_contract.imu_max_abs_acc_mps2 must be <= 200.0");
    }
    if (!std::isfinite(c.real_adapter_contract_wheel_max_abs_linear_mps) ||
        c.real_adapter_contract_wheel_max_abs_linear_mps <= 0.0) {
        errors.push_back("real_adapter_contract.wheel_max_abs_linear_mps must be > 0");
    } else if (c.real_adapter_contract_wheel_max_abs_linear_mps > 10.0) {
        errors.push_back("real_adapter_contract.wheel_max_abs_linear_mps must be <= 10.0");
    }
    if (!std::isfinite(c.real_adapter_contract_wheel_max_abs_angular_rad_s) ||
        c.real_adapter_contract_wheel_max_abs_angular_rad_s <= 0.0) {
        errors.push_back("real_adapter_contract.wheel_max_abs_angular_rad_s must be > 0");
    } else if (c.real_adapter_contract_wheel_max_abs_angular_rad_s > 50.0) {
        errors.push_back("real_adapter_contract.wheel_max_abs_angular_rad_s must be <= 50.0");
    }
    if (c.prelive_autonomous_slam_max_iterations <= 0) {
        errors.push_back("prelive_autonomous_slam.max_iterations must be > 0");
    } else if (c.prelive_autonomous_slam_max_iterations > 200) {
        errors.push_back("prelive_autonomous_slam.max_iterations must be <= 200");
    }
    if (!std::isfinite(c.prelive_autonomous_slam_start_time_s)) {
        errors.push_back("prelive_autonomous_slam.start_time_s must be finite");
    }
    if (!std::isfinite(c.prelive_autonomous_slam_step_s) ||
        c.prelive_autonomous_slam_step_s <= 0.0) {
        errors.push_back("prelive_autonomous_slam.step_s must be > 0");
    } else if (c.prelive_autonomous_slam_step_s > 1.0) {
        errors.push_back("prelive_autonomous_slam.step_s must be <= 1.0");
    }
    if (c.prelive_autonomous_slam_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("prelive_autonomous_slam.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.prelive_autonomous_slam_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("prelive_autonomous_slam.enabled=true requires production_interface_enabled=false");
    }
    if (!std::isfinite(c.slam_backend_binding_max_input_age_s) ||
        c.slam_backend_binding_max_input_age_s <= 0.0) {
        errors.push_back("slam_backend_binding.max_input_age_s must be > 0");
    } else if (c.slam_backend_binding_max_input_age_s > 5.0) {
        errors.push_back("slam_backend_binding.max_input_age_s must be <= 5.0");
    }
    if (!std::isfinite(c.slam_backend_binding_max_update_latency_s) ||
        c.slam_backend_binding_max_update_latency_s <= 0.0) {
        errors.push_back("slam_backend_binding.max_update_latency_s must be > 0");
    } else if (c.slam_backend_binding_max_update_latency_s > 10.0) {
        errors.push_back("slam_backend_binding.max_update_latency_s must be <= 10.0");
    }
    if (c.slam_backend_binding_min_integrated_scan_count_for_quality < 0) {
        errors.push_back("slam_backend_binding.min_integrated_scan_count_for_quality must be >= 0");
    }
    if (c.slam_backend_binding_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("slam_backend_binding.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.slam_backend_binding_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("slam_backend_binding.enabled=true requires production_interface_enabled=false");
    }
    const std::unordered_set<std::string> allowed_e2e_scenarios{
        "minimal_map_already_good",
        "active_scan_then_map_good",
        "sensor_contract_failure",
        "slam_backend_update_rejected",
        "slam_backend_quality_invalid",
        "motion_rejected",
        "map_quality_stuck",
    };
    if (!allowed_e2e_scenarios.count(c.autonomous_slam_e2e_prelive_scenario_kind)) {
        errors.push_back("autonomous_slam_e2e_prelive.scenario_kind is invalid");
    }
    if (c.autonomous_slam_e2e_prelive_max_iterations <= 0) {
        errors.push_back("autonomous_slam_e2e_prelive.max_iterations must be > 0");
    } else if (c.autonomous_slam_e2e_prelive_max_iterations > 200) {
        errors.push_back("autonomous_slam_e2e_prelive.max_iterations must be <= 200");
    }
    if (!std::isfinite(c.autonomous_slam_e2e_prelive_start_time_s)) {
        errors.push_back("autonomous_slam_e2e_prelive.start_time_s must be finite");
    }
    if (!std::isfinite(c.autonomous_slam_e2e_prelive_step_s) ||
        c.autonomous_slam_e2e_prelive_step_s <= 0.0) {
        errors.push_back("autonomous_slam_e2e_prelive.step_s must be > 0");
    } else if (c.autonomous_slam_e2e_prelive_step_s > 1.0) {
        errors.push_back("autonomous_slam_e2e_prelive.step_s must be <= 1.0");
    }
    if (c.autonomous_slam_e2e_prelive_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("autonomous_slam_e2e_prelive.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.autonomous_slam_e2e_prelive_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("autonomous_slam_e2e_prelive.enabled=true requires production_interface_enabled=false");
    }
    if (c.real_adapter_stubs_allow_real_hardware_adapters) {
        errors.push_back("real_adapter_stubs.allow_real_hardware_adapters must remain false");
    }
    if (!c.real_adapter_stubs_require_explicit_live_enable) {
        errors.push_back("real_adapter_stubs.require_explicit_live_enable must remain true");
    }
    if (c.live_handoff_readiness_allow_forward_backward) {
        errors.push_back("live_handoff_readiness.allow_forward_backward must remain false");
    }
    if (c.real_adapter_stubs_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("real_adapter_stubs.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.real_adapter_stubs_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("real_adapter_stubs.enabled=true requires production_interface_enabled=false");
    }
    if (c.live_handoff_readiness_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("live_handoff_readiness.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.live_handoff_readiness_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("live_handoff_readiness.enabled=true requires production_interface_enabled=false");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_packet_age_s) ||
        c.real_sensor_adapter_data_contract_max_packet_age_s <= 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_packet_age_s must be > 0");
    } else if (c.real_sensor_adapter_data_contract_max_packet_age_s > 5.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_packet_age_s must be <= 5.0");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_sensor_sync_dt_s) ||
        c.real_sensor_adapter_data_contract_max_sensor_sync_dt_s <= 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_sensor_sync_dt_s must be > 0");
    } else if (c.real_sensor_adapter_data_contract_max_sensor_sync_dt_s > 1.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_sensor_sync_dt_s must be <= 1.0");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_request_latency_s) ||
        c.real_sensor_adapter_data_contract_max_request_latency_s <= 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_request_latency_s must be > 0");
    } else if (c.real_sensor_adapter_data_contract_max_request_latency_s > 2.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_request_latency_s must be <= 2.0");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_tof_nan_ratio) ||
        c.real_sensor_adapter_data_contract_max_tof_nan_ratio < 0.0 ||
        c.real_sensor_adapter_data_contract_max_tof_nan_ratio > 1.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_tof_nan_ratio must be in [0,1]");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_min_tof_range_m) ||
        c.real_sensor_adapter_data_contract_min_tof_range_m <= 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.min_tof_range_m must be > 0");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_tof_range_m) ||
        c.real_sensor_adapter_data_contract_max_tof_range_m <=
            c.real_sensor_adapter_data_contract_min_tof_range_m) {
        errors.push_back("real_sensor_adapter_data_contract.max_tof_range_m must be > min_tof_range_m");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_abs_yaw_rate_rad_s) ||
        c.real_sensor_adapter_data_contract_max_abs_yaw_rate_rad_s <= 0.0 ||
        c.real_sensor_adapter_data_contract_max_abs_yaw_rate_rad_s > 50.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_abs_yaw_rate_rad_s must be in (0,50]");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_accel_norm_mps2) ||
        c.real_sensor_adapter_data_contract_max_accel_norm_mps2 <= 0.0 ||
        c.real_sensor_adapter_data_contract_max_accel_norm_mps2 > 200.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_accel_norm_mps2 must be in (0,200]");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_abs_wheel_linear_mps) ||
        c.real_sensor_adapter_data_contract_max_abs_wheel_linear_mps <= 0.0 ||
        c.real_sensor_adapter_data_contract_max_abs_wheel_linear_mps > 10.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_abs_wheel_linear_mps must be in (0,10]");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_abs_wheel_angular_rad_s) ||
        c.real_sensor_adapter_data_contract_max_abs_wheel_angular_rad_s <= 0.0 ||
        c.real_sensor_adapter_data_contract_max_abs_wheel_angular_rad_s > 50.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_abs_wheel_angular_rad_s must be in (0,50]");
    }
    if (c.real_sensor_adapter_data_contract_require_frame_id &&
        (c.real_sensor_adapter_data_contract_default_tof_frame_id.empty() ||
         c.real_sensor_adapter_data_contract_default_imu_frame_id.empty() ||
         c.real_sensor_adapter_data_contract_default_wheel_frame_id.empty())) {
        errors.push_back("real_sensor_adapter_data_contract default frame ids must be non-empty");
    }
    if (!c.real_sensor_adapter_data_contract_require_request_timing) {
        errors.push_back("real_sensor_adapter_data_contract.require_request_timing must remain true");
    }
    if (c.real_sensor_adapter_data_contract_run_acceptance_on_startup) {
        errors.push_back("real_sensor_adapter_data_contract.run_acceptance_on_startup must remain false");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_request_latency_mismatch_s) ||
        c.real_sensor_adapter_data_contract_max_request_latency_mismatch_s <= 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_request_latency_mismatch_s must be > 0");
    } else if (c.real_sensor_adapter_data_contract_max_request_latency_mismatch_s > 0.1) {
        errors.push_back("real_sensor_adapter_data_contract.max_request_latency_mismatch_s must be <= 0.1");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_estimated_sample_time_midpoint_error_s) ||
        c.real_sensor_adapter_data_contract_max_estimated_sample_time_midpoint_error_s <= 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_estimated_sample_time_midpoint_error_s must be > 0");
    } else if (c.real_sensor_adapter_data_contract_max_estimated_sample_time_midpoint_error_s > 0.1) {
        errors.push_back("real_sensor_adapter_data_contract.max_estimated_sample_time_midpoint_error_s must be <= 0.1");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_future_timestamp_skew_s) ||
        c.real_sensor_adapter_data_contract_max_future_timestamp_skew_s < 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_future_timestamp_skew_s must be >= 0");
    } else if (c.real_sensor_adapter_data_contract_max_future_timestamp_skew_s > 1.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_future_timestamp_skew_s must be <= 1.0");
    }
    if (!std::isfinite(c.real_sensor_adapter_data_contract_max_packet_sensor_time_dt_s) ||
        c.real_sensor_adapter_data_contract_max_packet_sensor_time_dt_s <= 0.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_packet_sensor_time_dt_s must be > 0");
    } else if (c.real_sensor_adapter_data_contract_max_packet_sensor_time_dt_s > 2.0) {
        errors.push_back("real_sensor_adapter_data_contract.max_packet_sensor_time_dt_s must be <= 2.0");
    }
    if (!c.real_sensor_adapter_data_contract_reject_request_latency_mismatch) {
        errors.push_back("real_sensor_adapter_data_contract.reject_request_latency_mismatch must remain true");
    }
    if (!c.real_sensor_adapter_data_contract_require_estimated_sample_time_in_window) {
        errors.push_back("real_sensor_adapter_data_contract.require_estimated_sample_time_in_window must remain true");
    }
    if (!c.real_sensor_adapter_data_contract_require_estimated_sample_time_midpoint) {
        errors.push_back("real_sensor_adapter_data_contract.require_estimated_sample_time_midpoint must remain true");
    }
    if (!c.real_sensor_adapter_data_contract_reject_future_sensor_time) {
        errors.push_back("real_sensor_adapter_data_contract.reject_future_sensor_time must remain true");
    }
    if (c.real_sensor_adapter_data_contract_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("real_sensor_adapter_data_contract.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.real_sensor_adapter_data_contract_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("real_sensor_adapter_data_contract.enabled=true requires production_interface_enabled=false");
    }
    if (!std::isfinite(c.real_sensor_replay_start_time_s)) {
        errors.push_back("real_sensor_replay.start_time_s must be finite");
    }
    if (c.real_sensor_replay_run_acceptance_on_startup) {
        errors.push_back("real_sensor_replay.run_acceptance_on_startup must remain false");
    }
    if (c.real_sensor_replay_time_mode != "external_now" &&
        c.real_sensor_replay_time_mode != "record_packet_time" &&
        c.real_sensor_replay_time_mode != "record_sensor_max_time") {
        errors.push_back("real_sensor_replay.time_mode must be external_now, record_packet_time, or record_sensor_max_time");
    }
    if (c.real_sensor_replay_time_mode != "record_packet_time") {
        errors.push_back("real_sensor_replay.time_mode must default to record_packet_time");
    }
    if (!c.real_sensor_replay_reject_invalid_records) {
        errors.push_back("real_sensor_replay.reject_invalid_records must remain true");
    }
    if (!c.real_sensor_replay_require_packet_records) {
        errors.push_back("real_sensor_replay.require_packet_records must remain true");
    }
    if (!c.real_sensor_replay_preserve_parse_errors) {
        errors.push_back("real_sensor_replay.preserve_parse_errors must remain true");
    }
    if (c.real_sensor_replay_max_records_per_run <= 0 ||
        c.real_sensor_replay_max_records_per_run > 100000) {
        errors.push_back("real_sensor_replay.max_records_per_run must be in (0,100000]");
    }
    if (c.real_sensor_replay_regression_run_on_startup) {
        errors.push_back("real_sensor_replay_regression.run_on_startup must remain false");
    }
    if (c.real_sensor_replay_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("real_sensor_replay.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.real_sensor_replay_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("real_sensor_replay.enabled=true requires production_interface_enabled=false");
    }
    if (c.real_sensor_replay_regression_enabled && c.motion_execution_hardware_write_enabled) {
        errors.push_back("real_sensor_replay_regression.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.real_sensor_replay_regression_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("real_sensor_replay_regression.enabled=true requires production_interface_enabled=false");
    }

    if (c.deterministic_slam_backend_run_regression_on_startup) {
        errors.push_back("deterministic_slam_backend.run_regression_on_startup must remain false");
    }
    if (c.replay_to_slam_backend_regression_run_on_startup) {
        errors.push_back("replay_to_slam_backend_regression.run_on_startup must remain false");
    }
    if (c.deterministic_slam_backend_allow_save_map) {
        errors.push_back("deterministic_slam_backend.allow_save_map must remain false");
    }
    if (c.deterministic_slam_backend_min_valid_ranges <= 0) {
        errors.push_back("deterministic_slam_backend.min_valid_ranges must be > 0");
    }
    if (c.deterministic_slam_backend_min_valid_scan_count_for_good <= 0) {
        errors.push_back("deterministic_slam_backend.min_valid_scan_count_for_good must be > 0");
    }
    if (c.deterministic_slam_backend_min_valid_range_ratio < 0.0 ||
        c.deterministic_slam_backend_min_valid_range_ratio > 1.0) {
        errors.push_back("deterministic_slam_backend.min_valid_range_ratio must be in [0,1]");
    }
    if (c.deterministic_slam_backend_min_coverage_ratio_for_good < 0.0 ||
        c.deterministic_slam_backend_min_coverage_ratio_for_good > 1.0) {
        errors.push_back("deterministic_slam_backend.min_coverage_ratio_for_good must be in [0,1]");
    }
    if (c.deterministic_slam_backend_min_yaw_coverage_ratio_for_good < 0.0 ||
        c.deterministic_slam_backend_min_yaw_coverage_ratio_for_good > 1.0) {
        errors.push_back("deterministic_slam_backend.min_yaw_coverage_ratio_for_good must be in [0,1]");
    }
    if (c.deterministic_slam_backend_keyframe_yaw_delta_rad <= 0.0 ||
        c.deterministic_slam_backend_keyframe_yaw_delta_rad > 3.14) {
        errors.push_back("deterministic_slam_backend.keyframe_yaw_delta_rad must be in (0,3.14]");
    }
    if (c.deterministic_slam_backend_min_range_m <= 0.0 ||
        c.deterministic_slam_backend_max_range_m <= c.deterministic_slam_backend_min_range_m) {
        errors.push_back("deterministic_slam_backend range limits are invalid");
    }
    if (c.deterministic_slam_backend_max_update_latency_s <= 0.0) {
        errors.push_back("deterministic_slam_backend.max_update_latency_s must be > 0");
    }
    if (c.deterministic_slam_backend_yaw_bin_size_rad <= 0.0 ||
        c.deterministic_slam_backend_yaw_bin_size_rad > 3.14) {
        errors.push_back("deterministic_slam_backend.yaw_bin_size_rad must be in (0,3.14]");
    }
    if (c.replay_to_slam_backend_regression_min_accepted_updates <= 0) {
        errors.push_back("replay_to_slam_backend_regression.min_accepted_updates must be > 0");
    }
    if ((c.deterministic_slam_backend_enabled || c.replay_to_slam_backend_regression_enabled) &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("deterministic slam backend requires motion_execution.hardware_write_enabled=false");
    }
    if ((c.deterministic_slam_backend_enabled || c.replay_to_slam_backend_regression_enabled) &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("deterministic slam backend requires production_interface_enabled=false");
    }

    if (c.full_autonomous_slam_fake_pipeline_run_on_startup) {
        errors.push_back("full_autonomous_slam_fake_pipeline.run_on_startup must remain false");
    }
    if (c.full_autonomous_slam_fake_pipeline_max_steps <= 0 ||
        c.full_autonomous_slam_fake_pipeline_max_steps > 1000) {
        errors.push_back("full_autonomous_slam_fake_pipeline.max_steps must be in (0,1000]");
    }
    if (c.full_autonomous_slam_fake_pipeline_max_active_scan_commands < 0 ||
        c.full_autonomous_slam_fake_pipeline_max_active_scan_commands > 100) {
        errors.push_back("full_autonomous_slam_fake_pipeline.max_active_scan_commands must be in [0,100]");
    }
    if (c.full_autonomous_slam_fake_pipeline_min_backend_accepted_updates <= 0) {
        errors.push_back("full_autonomous_slam_fake_pipeline.min_backend_accepted_updates must be > 0");
    }
    if (!c.full_autonomous_slam_fake_pipeline_require_shadow_motion_only) {
        errors.push_back("full_autonomous_slam_fake_pipeline.require_shadow_motion_only must remain true");
    }
    if (!c.full_autonomous_slam_fake_pipeline_require_no_forward_backward) {
        errors.push_back("full_autonomous_slam_fake_pipeline.require_no_forward_backward must remain true");
    }
    if (c.full_autonomous_slam_fake_pipeline_motion_settle_s < 0.0 ||
        c.full_autonomous_slam_fake_pipeline_motion_settle_s > 10.0 ||
        !std::isfinite(c.full_autonomous_slam_fake_pipeline_motion_settle_s)) {
        errors.push_back("full_autonomous_slam_fake_pipeline.motion_settle_s must be in [0,10]");
    }
    if (!c.full_autonomous_slam_fake_pipeline_phase_aware_sensor_consumption) {
        errors.push_back("full_autonomous_slam_fake_pipeline.phase_aware_sensor_consumption must remain true");
    }
    if (!c.full_autonomous_slam_fake_pipeline_require_phase_aware_sensor_consumption) {
        errors.push_back("full_autonomous_slam_fake_pipeline.require_phase_aware_sensor_consumption must remain true");
    }
    if (!c.full_autonomous_slam_fake_pipeline_build_fake_map_on_completed) {
        errors.push_back("full_autonomous_slam_fake_pipeline.build_fake_map_on_completed must remain true");
    }
    if (!c.full_autonomous_slam_fake_pipeline_save_fake_map_on_completed) {
        errors.push_back("full_autonomous_slam_fake_pipeline.save_fake_map_on_completed must remain true");
    }
    if (!c.full_autonomous_slam_fake_pipeline_require_fake_map_saved) {
        errors.push_back("full_autonomous_slam_fake_pipeline.require_fake_map_saved must remain true");
    }
    if (c.full_autonomous_slam_fake_pipeline_fake_map_id_prefix.empty()) {
        errors.push_back("full_autonomous_slam_fake_pipeline.fake_map_id_prefix must be non-empty");
    }
    if (c.full_autonomous_slam_fake_pipeline_enabled &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("full_autonomous_slam_fake_pipeline.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.full_autonomous_slam_fake_pipeline_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("full_autonomous_slam_fake_pipeline.enabled=true requires production_interface_enabled=false");
    }
    if (c.full_autonomous_slam_fake_pipeline_enabled &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("full_autonomous_slam_fake_pipeline.enabled=true requires allow_writer_dispatch=false");
    }
    if (c.fake_map_artifact_enabled &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("fake_map_artifact.enabled=true requires motion_execution.hardware_write_enabled=false");
    }
    if (c.fake_map_artifact_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("fake_map_artifact.enabled=true requires production_interface_enabled=false");
    }
    if (c.fake_map_artifact_enabled &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("fake_map_artifact.enabled=true requires allow_writer_dispatch=false");
    }
    if (c.fake_relocalization_allow_pose_writeback) {
        errors.push_back("fake_relocalization.allow_pose_writeback must remain false");
    }
    if (c.fake_relocalization_run_on_startup) {
        errors.push_back("fake_relocalization.run_on_startup must remain false");
    }
    if (c.fake_relocalization_min_valid_ranges <= 0) {
        errors.push_back("fake_relocalization.min_valid_ranges must be > 0");
    }
    if (c.fake_relocalization_min_confidence < 0.0 ||
        c.fake_relocalization_min_confidence > 1.0 ||
        !std::isfinite(c.fake_relocalization_min_confidence)) {
        errors.push_back("fake_relocalization.min_confidence must be in [0,1]");
    }
    if (c.fake_relocalization_high_confidence_threshold < 0.0 ||
        c.fake_relocalization_high_confidence_threshold > 1.0 ||
        c.fake_relocalization_high_confidence_threshold < c.fake_relocalization_min_confidence ||
        !std::isfinite(c.fake_relocalization_high_confidence_threshold)) {
        errors.push_back("fake_relocalization.high_confidence_threshold must be in [min_confidence,1]");
    }
    if (c.fake_relocalization_min_map_coverage_ratio < 0.0 ||
        c.fake_relocalization_min_map_coverage_ratio > 1.0 ||
        !std::isfinite(c.fake_relocalization_min_map_coverage_ratio)) {
        errors.push_back("fake_relocalization.min_map_coverage_ratio must be in [0,1]");
    }
    if (c.fake_relocalization_min_map_yaw_coverage_ratio < 0.30 ||
        c.fake_relocalization_min_map_yaw_coverage_ratio > 1.0 ||
        !std::isfinite(c.fake_relocalization_min_map_yaw_coverage_ratio)) {
        errors.push_back("fake_relocalization.min_map_yaw_coverage_ratio must be in [0.30,1]");
    }
    if (c.fake_map_relocalization_runner_run_on_startup) {
        errors.push_back("fake_map_relocalization_runner.run_on_startup must remain false");
    }
    if (!c.fake_map_relocalization_runner_require_no_pose_writeback) {
        errors.push_back("fake_map_relocalization_runner.require_no_pose_writeback must remain true");
    }
    if ((c.fake_relocalization_enabled || c.fake_map_relocalization_runner_enabled) &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("fake relocalization requires motion_execution.hardware_write_enabled=false");
    }
    if ((c.fake_relocalization_enabled || c.fake_map_relocalization_runner_enabled) &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("fake relocalization requires production_interface_enabled=false");
    }
    if ((c.fake_relocalization_enabled || c.fake_map_relocalization_runner_enabled) &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("fake relocalization requires allow_writer_dispatch=false");
    }
    if (c.fake_relocalization_readiness_gate_min_confidence < 0.0 ||
        c.fake_relocalization_readiness_gate_min_confidence > 1.0 ||
        !std::isfinite(c.fake_relocalization_readiness_gate_min_confidence)) {
        errors.push_back("fake_relocalization_readiness_gate.min_confidence must be in [0,1]");
    }
    if (c.fake_relocalization_readiness_gate_min_map_coverage_ratio < 0.0 ||
        c.fake_relocalization_readiness_gate_min_map_coverage_ratio > 1.0 ||
        !std::isfinite(c.fake_relocalization_readiness_gate_min_map_coverage_ratio)) {
        errors.push_back("fake_relocalization_readiness_gate.min_map_coverage_ratio must be in [0,1]");
    }
    if (c.fake_relocalization_readiness_gate_min_map_yaw_coverage_ratio < 0.30 ||
        c.fake_relocalization_readiness_gate_min_map_yaw_coverage_ratio > 1.0 ||
        !std::isfinite(c.fake_relocalization_readiness_gate_min_map_yaw_coverage_ratio)) {
        errors.push_back("fake_relocalization_readiness_gate.min_map_yaw_coverage_ratio must be in [0.30,1]");
    }
    if (c.fake_autonomous_slam_product_acceptance_run_on_startup) {
        errors.push_back("fake_autonomous_slam_product_acceptance.run_on_startup must remain false");
    }
    if (!c.fake_autonomous_slam_product_acceptance_require_no_pose_writeback) {
        errors.push_back("fake_autonomous_slam_product_acceptance.require_no_pose_writeback must remain true");
    }
    if (!c.fake_autonomous_slam_product_acceptance_require_no_forward_backward) {
        errors.push_back("fake_autonomous_slam_product_acceptance.require_no_forward_backward must remain true");
    }
    if ((c.fake_relocalization_readiness_gate_enabled ||
         c.fake_autonomous_slam_product_acceptance_enabled) &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("fake product acceptance requires motion_execution.hardware_write_enabled=false");
    }
    if ((c.fake_relocalization_readiness_gate_enabled ||
         c.fake_autonomous_slam_product_acceptance_enabled) &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("fake product acceptance requires production_interface_enabled=false");
    }
    if ((c.fake_relocalization_readiness_gate_enabled ||
         c.fake_autonomous_slam_product_acceptance_enabled) &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("fake product acceptance requires allow_writer_dispatch=false");
    }

    if (c.multi_tof_raw_data_contract_run_acceptance_on_startup) {
        errors.push_back("multi_tof_raw_data_contract.run_acceptance_on_startup must remain false");
    }
    if (c.multi_tof_raw_data_contract_min_required_tof_count < 1 ||
        c.multi_tof_raw_data_contract_min_required_tof_count > 3) {
        errors.push_back("multi_tof_raw_data_contract.min_required_tof_count must be in [1,3]");
    }
    if (c.multi_tof_raw_data_contract_front_frame_id.empty() ||
        c.multi_tof_raw_data_contract_left_frame_id.empty() ||
        c.multi_tof_raw_data_contract_right_frame_id.empty()) {
        errors.push_back("multi_tof_raw_data_contract frame ids must not be empty");
    }
    if (c.multi_tof_raw_data_contract_front_frame_id == c.multi_tof_raw_data_contract_left_frame_id ||
        c.multi_tof_raw_data_contract_front_frame_id == c.multi_tof_raw_data_contract_right_frame_id ||
        c.multi_tof_raw_data_contract_left_frame_id == c.multi_tof_raw_data_contract_right_frame_id) {
        errors.push_back("multi_tof_raw_data_contract frame ids must be unique");
    }
    if (std::fabs(c.multi_tof_raw_data_contract_front_mount_yaw_rad - 0.0) >
        c.multi_tof_raw_data_contract_max_mount_yaw_error_rad) {
        errors.push_back("multi_tof_raw_data_contract.front_mount_yaw_rad must be near 0");
    }
    if (std::fabs(c.multi_tof_raw_data_contract_left_mount_yaw_rad - 1.5707963267948966) >
        c.multi_tof_raw_data_contract_max_mount_yaw_error_rad) {
        errors.push_back("multi_tof_raw_data_contract.left_mount_yaw_rad must be near +pi/2");
    }
    if (std::fabs(c.multi_tof_raw_data_contract_right_mount_yaw_rad + 1.5707963267948966) >
        c.multi_tof_raw_data_contract_max_mount_yaw_error_rad) {
        errors.push_back("multi_tof_raw_data_contract.right_mount_yaw_rad must be near -pi/2");
    }
    if (!(c.multi_tof_raw_data_contract_max_request_latency_s > 0.0) ||
        !std::isfinite(c.multi_tof_raw_data_contract_max_request_latency_s)) {
        errors.push_back("multi_tof_raw_data_contract.max_request_latency_s must be > 0");
    }
    if (!(c.multi_tof_raw_data_contract_max_request_latency_mismatch_s > 0.0) ||
        !std::isfinite(c.multi_tof_raw_data_contract_max_request_latency_mismatch_s)) {
        errors.push_back("multi_tof_raw_data_contract.max_request_latency_mismatch_s must be > 0");
    }
    if (c.multi_tof_raw_data_contract_max_nan_ratio < 0.0 ||
        c.multi_tof_raw_data_contract_max_nan_ratio > 1.0 ||
        !std::isfinite(c.multi_tof_raw_data_contract_max_nan_ratio)) {
        errors.push_back("multi_tof_raw_data_contract.max_nan_ratio must be in [0,1]");
    }
    if (!(c.multi_tof_raw_data_contract_max_range_m > c.multi_tof_raw_data_contract_min_range_m)) {
        errors.push_back("multi_tof_raw_data_contract.max_range_m must be greater than min_range_m");
    }
    if (c.multi_tof_raw_data_contract_enabled &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("multi_tof_raw_data_contract requires motion_execution.hardware_write_enabled=false");
    }
    if (c.multi_tof_raw_data_contract_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("multi_tof_raw_data_contract requires production_interface_enabled=false");
    }
    if (c.multi_tof_raw_data_contract_enabled &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("multi_tof_raw_data_contract requires allow_writer_dispatch=false");
    }

    if (c.multi_tof_sync_run_acceptance_on_startup) {
        errors.push_back("multi_tof_sync.run_acceptance_on_startup must remain false");
    }
    if (!one_of(c.multi_tof_sync_time_reference, {"front", "median_present_tof", "mean_present_tof"})) {
        errors.push_back("multi_tof_sync.time_reference must be front, median_present_tof, or mean_present_tof");
    }
    if (!one_of(c.multi_tof_sync_degradation_mode, {"require_all", "allow_missing_one", "allow_any_two", "allow_front_only"})) {
        errors.push_back("multi_tof_sync.degradation_mode must be require_all, allow_missing_one, allow_any_two, or allow_front_only");
    }
    if (c.multi_tof_sync_min_required_tof_count < 1 ||
        c.multi_tof_sync_min_required_tof_count > 3) {
        errors.push_back("multi_tof_sync.min_required_tof_count must be in [1,3]");
    }
    if (!c.multi_tof_sync_require_raw_contract_pass) {
        errors.push_back("multi_tof_sync.require_raw_contract_pass must remain true");
    }
    if (!(c.multi_tof_sync_max_pairwise_tof_sync_dt_s > 0.0) ||
        c.multi_tof_sync_max_pairwise_tof_sync_dt_s > 1.0 ||
        !std::isfinite(c.multi_tof_sync_max_pairwise_tof_sync_dt_s)) {
        errors.push_back("multi_tof_sync.max_pairwise_tof_sync_dt_s must be in (0,1]");
    }
    if (!(c.multi_tof_sync_max_multi_tof_imu_sync_dt_s > 0.0) ||
        c.multi_tof_sync_max_multi_tof_imu_sync_dt_s > 1.0 ||
        !std::isfinite(c.multi_tof_sync_max_multi_tof_imu_sync_dt_s)) {
        errors.push_back("multi_tof_sync.max_multi_tof_imu_sync_dt_s must be in (0,1]");
    }
    if (!(c.multi_tof_sync_max_multi_tof_wheel_sync_dt_s > 0.0) ||
        c.multi_tof_sync_max_multi_tof_wheel_sync_dt_s > 1.0 ||
        !std::isfinite(c.multi_tof_sync_max_multi_tof_wheel_sync_dt_s)) {
        errors.push_back("multi_tof_sync.max_multi_tof_wheel_sync_dt_s must be in (0,1]");
    }
    if (!(c.multi_tof_sync_max_effective_time_future_skew_s >= 0.0) ||
        c.multi_tof_sync_max_effective_time_future_skew_s > 1.0 ||
        !std::isfinite(c.multi_tof_sync_max_effective_time_future_skew_s)) {
        errors.push_back("multi_tof_sync.max_effective_time_future_skew_s must be in [0,1]");
    }
    if (c.multi_tof_sync_enabled &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("multi_tof_sync requires motion_execution.hardware_write_enabled=false");
    }
    if (c.multi_tof_sync_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("multi_tof_sync requires production_interface_enabled=false");
    }
    if (c.multi_tof_sync_enabled &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("multi_tof_sync requires allow_writer_dispatch=false");
    }

    if (c.multi_tof_snapshot_builder_run_acceptance_on_startup) {
        errors.push_back("multi_tof_snapshot_builder.run_acceptance_on_startup must remain false");
    }
    if (!c.multi_tof_snapshot_builder_require_sync_pass) {
        errors.push_back("multi_tof_snapshot_builder.require_sync_pass must remain true");
    }
    if (!one_of(c.multi_tof_snapshot_builder_legacy_primary_mode, {"front", "median_time_closest", "first_present"})) {
        errors.push_back("multi_tof_snapshot_builder.legacy_primary_mode must be front, median_time_closest, or first_present");
    }
    if (c.multi_tof_snapshot_builder_min_required_tof_count < 1 ||
        c.multi_tof_snapshot_builder_min_required_tof_count > 3) {
        errors.push_back("multi_tof_snapshot_builder.min_required_tof_count must be in [1,3]");
    }
    if (c.multi_tof_snapshot_builder_enabled &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("multi_tof_snapshot_builder requires motion_execution.hardware_write_enabled=false");
    }
    if (c.multi_tof_snapshot_builder_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("multi_tof_snapshot_builder requires production_interface_enabled=false");
    }
    if (c.multi_tof_snapshot_builder_enabled &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("multi_tof_snapshot_builder requires allow_writer_dispatch=false");
    }

    if (c.multi_tof_replay_run_acceptance_on_startup) {
        errors.push_back("multi_tof_replay.run_acceptance_on_startup must remain false");
    }
    if (!c.multi_tof_replay_reject_invalid_records) {
        errors.push_back("multi_tof_replay.reject_invalid_records must remain true");
    }
    if (!c.multi_tof_replay_require_snapshot_build) {
        errors.push_back("multi_tof_replay.require_snapshot_build must remain true");
    }
    if (!one_of(c.multi_tof_replay_time_mode, {"external_now", "record_packet_time", "record_sensor_max_time"})) {
        errors.push_back("multi_tof_replay.time_mode must be external_now, record_packet_time, or record_sensor_max_time");
    }
    if (c.multi_tof_replay_enabled &&
        c.motion_execution_hardware_write_enabled) {
        errors.push_back("multi_tof_replay requires motion_execution.hardware_write_enabled=false");
    }
    if (c.multi_tof_replay_enabled &&
        c.motion_execution_software_motion_production_interface_enabled) {
        errors.push_back("multi_tof_replay requires production_interface_enabled=false");
    }
    if (c.multi_tof_replay_enabled &&
        c.motion_execution_allow_writer_dispatch) {
        errors.push_back("multi_tof_replay requires allow_writer_dispatch=false");
    }

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
      << "  consecutive_error_limit: " << c.encoder_consecutive_error_limit << "\n"
      << "  max_pair_skew_us: " << c.encoder_max_pair_skew_us << "\n"
      << "  max_read_latency_us: " << c.encoder_max_read_latency_us << "\n"
      << "  require_status_zero: " << bool_yaml(c.encoder_require_status_zero) << "\n"
      << "  require_pair_skew_ok_for_odometry: " << bool_yaml(c.encoder_require_pair_skew_ok_for_odometry) << "\n";
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
    o << "slam_runtime:\n"
      << "  mode: " << c.slam_runtime_mode << "\n"
      << "  sparse_shadow_sensor_source: " << c.sparse_shadow_sensor_source << "\n";
    o << "sparse_slam:\n"
      << "  map_startup_mode: " << c.sparse_slam_map_startup_mode << "\n"
      << "  initial_pose_mode: " << c.sparse_slam_initial_pose_mode << "\n"
      << "  has_configured_pose: " << bool_yaml(c.sparse_slam_has_configured_pose) << "\n"
      << "  configured_pose_x_m: " << c.sparse_slam_configured_pose_x_m << "\n"
      << "  configured_pose_y_m: " << c.sparse_slam_configured_pose_y_m << "\n"
      << "  configured_pose_yaw_rad: " << c.sparse_slam_configured_pose_yaw_rad << "\n"
      << "  pose_buffer_capacity: " << c.sparse_slam_pose_buffer_capacity << "\n"
      << "  pose_buffer_max_age_s: " << c.sparse_slam_pose_buffer_max_age_s << "\n"
      << "  pose_interpolation_max_gap_s: " << c.sparse_slam_pose_interpolation_max_gap_s << "\n"
      << "  require_all_measurement_poses: " << bool_yaml(c.sparse_slam_require_all_measurement_poses) << "\n"
      << "  startup_min_imu_samples: " << c.sparse_slam_startup_min_imu_samples << "\n"
      << "  startup_max_abs_linear_speed_mps: " << c.sparse_slam_startup_max_abs_linear_speed_mps << "\n"
      << "  startup_max_abs_wheel_yaw_rate_rad_s: " << c.sparse_slam_startup_max_abs_wheel_yaw_rate_rad_s << "\n"
      << "  startup_max_abs_imu_yaw_rate_rad_s: " << c.sparse_slam_startup_max_abs_imu_yaw_rate_rad_s << "\n"
      << "  startup_max_gyro_bias_abs_rad_s: " << c.sparse_slam_startup_max_gyro_bias_abs_rad_s << "\n"
      << "  startup_max_gyro_spread_rad_s: " << c.sparse_slam_startup_max_gyro_spread_rad_s << "\n"
      << "  active_bundle_max_snapshots: " << c.sparse_slam_active_bundle_max_snapshots << "\n"
      << "  active_bundle_max_rays: " << c.sparse_slam_active_bundle_max_rays << "\n"
      << "  active_bundle_max_duration_s: " << c.sparse_slam_active_bundle_max_duration_s << "\n"
      << "  active_bundle_max_snapshot_gap_s: " << c.sparse_slam_active_bundle_max_snapshot_gap_s << "\n"
      << "  active_bundle_min_snapshots: " << c.sparse_slam_active_bundle_min_snapshots << "\n"
      << "  active_bundle_min_matchable_rays: " << c.sparse_slam_active_bundle_min_matchable_rays << "\n"
      << "  active_bundle_min_yaw_span_rad: " << c.sparse_slam_active_bundle_min_yaw_span_rad << "\n"
      << "  active_bundle_require_all_measurement_poses: " << bool_yaml(c.sparse_slam_active_bundle_require_all_measurement_poses) << "\n"
      << "  settle_max_abs_linear_speed_mps: " << c.sparse_slam_settle_max_abs_linear_speed_mps << "\n"
      << "  settle_max_abs_wheel_yaw_rate_rad_s: " << c.sparse_slam_settle_max_abs_wheel_yaw_rate_rad_s << "\n"
      << "  settle_max_abs_imu_yaw_rate_rad_s: " << c.sparse_slam_settle_max_abs_imu_yaw_rate_rad_s << "\n"
      << "  settle_min_consecutive_samples: " << c.sparse_slam_settle_min_consecutive_samples << "\n"
      << "  settle_min_stable_duration_s: " << c.sparse_slam_settle_min_stable_duration_s << "\n"
      << "  settle_max_sample_gap_s: " << c.sparse_slam_settle_max_sample_gap_s << "\n"
      << "  reference_snapshot_max_cells: " << c.sparse_slam_reference_snapshot_max_cells << "\n"
      << "  local_match_mode: " << c.sparse_slam_local_match_mode << "\n"
      << "  local_match_max_abs_yaw_rad: " << c.sparse_slam_local_match_max_abs_yaw_rad << "\n"
      << "  local_match_coarse_yaw_step_rad: " << c.sparse_slam_local_match_coarse_yaw_step_rad << "\n"
      << "  local_match_fine_yaw_step_rad: " << c.sparse_slam_local_match_fine_yaw_step_rad << "\n"
      << "  local_match_max_abs_translation_x_m: " << c.sparse_slam_local_match_max_abs_translation_x_m << "\n"
      << "  local_match_max_abs_translation_y_m: " << c.sparse_slam_local_match_max_abs_translation_y_m << "\n"
      << "  local_match_max_candidate_count: " << c.sparse_slam_local_match_max_candidate_count << "\n"
      << "  local_match_min_bundle_frames: " << c.sparse_slam_local_match_min_bundle_frames << "\n"
      << "  local_match_min_matchable_rays: " << c.sparse_slam_local_match_min_matchable_rays << "\n"
      << "  local_match_min_reference_cells: " << c.sparse_slam_local_match_min_reference_cells << "\n"
      << "  local_match_min_reference_occupied_cells: " << c.sparse_slam_local_match_min_reference_occupied_cells << "\n"
      << "  local_match_min_reference_free_cells: " << c.sparse_slam_local_match_min_reference_free_cells << "\n"
      << "  local_match_require_revision_match: " << bool_yaml(c.sparse_slam_local_match_require_revision_match) << "\n"
      << "  local_match_require_immutable_snapshot: " << bool_yaml(c.sparse_slam_local_match_require_immutable_snapshot) << "\n";
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
      << "  encoder_max_pair_skew_us: " << c.encoder_max_pair_skew_us << "\n"
      << "  encoder_max_read_latency_us: " << c.encoder_max_read_latency_us << "\n"
      << "  encoder_require_status_zero: " << bool_yaml(c.encoder_require_status_zero) << "\n"
      << "  encoder_require_pair_skew_ok_for_odometry: " << bool_yaml(c.encoder_require_pair_skew_ok_for_odometry) << "\n"
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
      << "  protocol_min_range_m: " << c.tof_protocol_min_range_m << "\n"
      << "  protocol_max_range_m: " << c.tof_protocol_max_range_m << "\n"
      << "  mapping_min_range_m: " << c.tof_mapping_min_range_m << "\n"
      << "  mapping_max_range_m: " << c.tof_mapping_max_range_m << "\n"
      << "  confidence_min: " << c.confidence_min << "\n"
      << "  mapping_min_confidence: " << c.tof_mapping_min_confidence << "\n"
      << "  full_fov_deg: " << c.tof_full_fov_deg << "\n"
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
    o << "active_scan_execution:\n"
      << "  enabled: " << bool_yaml(c.active_scan_execution_enabled) << "\n"
      << "  mode: " << c.active_scan_execution_mode << "\n"
      << "  log_hz: " << c.active_scan_execution_log_hz << "\n"
      << "  require_active_scan_would_start: " << bool_yaml(c.active_scan_execution_require_active_scan_would_start) << "\n"
      << "  require_scan_recommended: " << bool_yaml(c.active_scan_execution_require_scan_recommended) << "\n"
      << "  require_localization_valid: " << bool_yaml(c.active_scan_execution_require_localization_valid) << "\n"
      << "  target_scan_angle_deg: " << c.active_scan_execution_target_scan_angle_deg << "\n"
      << "  complete_scan_angle_deg: " << c.active_scan_execution_complete_scan_angle_deg << "\n"
      << "  min_useful_scan_angle_deg: " << c.active_scan_execution_min_useful_scan_angle_deg << "\n"
      << "  target_yaw_rate_dps: " << c.active_scan_execution_target_yaw_rate_dps << "\n"
      << "  min_observed_yaw_rate_dps: " << c.active_scan_execution_min_observed_yaw_rate_dps << "\n"
      << "  max_observed_yaw_rate_dps: " << c.active_scan_execution_max_observed_yaw_rate_dps << "\n"
      << "  max_linear_speed_mps: " << c.active_scan_execution_max_linear_speed_mps << "\n"
      << "  precheck_hold_s: " << c.active_scan_execution_precheck_hold_s << "\n"
      << "  command_timeout_s: " << c.active_scan_execution_command_timeout_s << "\n"
      << "  cooldown_s: " << c.active_scan_execution_cooldown_s << "\n"
      << "  emit_zero_command_on_stop: " << bool_yaml(c.active_scan_execution_emit_zero_command_on_stop) << "\n"
      << "  command_log_only: " << bool_yaml(c.active_scan_execution_command_log_only) << "\n"
      << "  hardware_write_enabled: " << bool_yaml(c.active_scan_execution_hardware_write_enabled) << "\n";
    o << "sparse_scan:\n"
      << "  enabled: " << bool_yaml(c.sparse_scan_enabled) << "\n"
      << "  log_hz: " << c.sparse_scan_log_hz << "\n"
      << "  collect_on_active_scan: " << bool_yaml(c.sparse_scan_collect_on_active_scan) << "\n"
      << "  collect_on_command_verifying: " << bool_yaml(c.sparse_scan_collect_on_command_verifying) << "\n"
      << "  collect_on_natural_rotation: " << bool_yaml(c.sparse_scan_collect_on_natural_rotation) << "\n"
      << "  require_active_scan_recommended: " << bool_yaml(c.sparse_scan_require_active_scan_recommended) << "\n"
      << "  require_mapping_allowed: " << bool_yaml(c.sparse_scan_require_mapping_allowed) << "\n"
      << "  min_yaw_rate_dps: " << c.sparse_scan_min_yaw_rate_dps << "\n"
      << "  max_yaw_rate_dps: " << c.sparse_scan_max_yaw_rate_dps << "\n"
      << "  max_linear_speed_mps: " << c.sparse_scan_max_linear_speed_mps << "\n"
      << "  min_range_m: " << c.sparse_scan_min_range_m << "\n"
      << "  max_range_m: " << c.sparse_scan_max_range_m << "\n"
      << "  min_confidence: " << c.sparse_scan_min_confidence << "\n"
      << "  bin_angle_deg: " << c.sparse_scan_bin_angle_deg << "\n"
      << "  min_session_angle_deg: " << c.sparse_scan_min_session_angle_deg << "\n"
      << "  useful_session_angle_deg: " << c.sparse_scan_useful_session_angle_deg << "\n"
      << "  complete_session_angle_deg: " << c.sparse_scan_complete_session_angle_deg << "\n"
      << "  session_timeout_s: " << c.sparse_scan_session_timeout_s << "\n"
      << "  max_gap_s: " << c.sparse_scan_max_gap_s << "\n"
      << "  cooldown_s: " << c.sparse_scan_cooldown_s << "\n"
      << "  keep_invalid_samples: " << bool_yaml(c.sparse_scan_keep_invalid_samples) << "\n"
      << "  compute_hit_points: " << bool_yaml(c.sparse_scan_compute_hit_points) << "\n"
      << "  write_sample_log: " << bool_yaml(c.sparse_scan_write_sample_log) << "\n"
      << "  write_bin_log: " << bool_yaml(c.sparse_scan_write_bin_log) << "\n"
      << "  write_session_log: " << bool_yaml(c.sparse_scan_write_session_log) << "\n"
      << "  max_samples_per_session: " << c.sparse_scan_max_samples_per_session << "\n";
    o << "sparse_scan_yaw_match:\n"
      << "  enabled: " << bool_yaml(c.sparse_scan_yaw_match_enabled) << "\n"
      << "  log_hz: " << c.sparse_scan_yaw_match_log_hz << "\n"
      << "  run_on_completed_sparse_scan: " << bool_yaml(c.sparse_scan_yaw_match_run_on_completed_sparse_scan) << "\n"
      << "  run_on_useful_sparse_scan: " << bool_yaml(c.sparse_scan_yaw_match_run_on_useful_sparse_scan) << "\n"
      << "  require_complete_session: " << bool_yaml(c.sparse_scan_yaw_match_require_complete_session) << "\n"
      << "  max_yaw_search_deg: " << c.sparse_scan_yaw_match_max_yaw_search_deg << "\n"
      << "  coarse_step_deg: " << c.sparse_scan_yaw_match_coarse_step_deg << "\n"
      << "  refine_enabled: " << bool_yaml(c.sparse_scan_yaw_match_refine_enabled) << "\n"
      << "  refine_window_deg: " << c.sparse_scan_yaw_match_refine_window_deg << "\n"
      << "  refine_step_deg: " << c.sparse_scan_yaw_match_refine_step_deg << "\n"
      << "  min_valid_samples: " << c.sparse_scan_yaw_match_min_valid_samples << "\n"
      << "  min_valid_bins: " << c.sparse_scan_yaw_match_min_valid_bins << "\n"
      << "  min_observed_yaw_delta_deg: " << c.sparse_scan_yaw_match_min_observed_yaw_delta_deg << "\n"
      << "  min_valid_bin_ratio: " << c.sparse_scan_yaw_match_min_valid_bin_ratio << "\n"
      << "  occupied_search_radius_m: " << c.sparse_scan_yaw_match_occupied_search_radius_m << "\n"
      << "  occupied_hit_reward: " << c.sparse_scan_yaw_match_occupied_hit_reward << "\n"
      << "  free_hit_penalty: " << c.sparse_scan_yaw_match_free_hit_penalty << "\n"
      << "  unknown_hit_penalty: " << c.sparse_scan_yaw_match_unknown_hit_penalty << "\n"
      << "  distance_penalty_scale_m: " << c.sparse_scan_yaw_match_distance_penalty_scale_m << "\n"
      << "  min_best_score: " << c.sparse_scan_yaw_match_min_best_score << "\n"
      << "  min_score_margin: " << c.sparse_scan_yaw_match_min_score_margin << "\n"
      << "  min_inlier_ratio: " << c.sparse_scan_yaw_match_min_inlier_ratio << "\n"
      << "  max_curve_flatness: " << c.sparse_scan_yaw_match_max_curve_flatness << "\n"
      << "  multimodal_check_enabled: " << bool_yaml(c.sparse_scan_yaw_match_multimodal_check_enabled) << "\n"
      << "  multimodal_peak_separation_deg: " << c.sparse_scan_yaw_match_multimodal_peak_separation_deg << "\n"
      << "  max_second_peak_ratio: " << c.sparse_scan_yaw_match_max_second_peak_ratio << "\n"
      << "  max_samples_per_match: " << c.sparse_scan_yaw_match_max_samples_per_match << "\n"
      << "  write_curve_log: " << bool_yaml(c.sparse_scan_yaw_match_write_curve_log) << "\n"
      << "  write_summary_log: " << bool_yaml(c.sparse_scan_yaw_match_write_summary_log) << "\n";
    o << "yaw_correction:\n"
      << "  enabled: " << bool_yaml(c.yaw_correction_enabled) << "\n"
      << "  mode: " << c.yaw_correction_mode << "\n"
      << "  log_hz: " << c.yaw_correction_log_hz << "\n"
      << "  require_yaw_match_usable: " << bool_yaml(c.yaw_correction_require_yaw_match_usable) << "\n"
      << "  require_mapping_state_ok: " << bool_yaml(c.yaw_correction_require_mapping_state_ok) << "\n"
      << "  require_low_speed_or_static: " << bool_yaml(c.yaw_correction_require_low_speed_or_static) << "\n"
      << "  require_active_scan_complete: " << bool_yaml(c.yaw_correction_require_active_scan_complete) << "\n"
      << "  scan_completion_source: " << c.yaw_correction_scan_completion_source << "\n"
      << "  allow_yaw_match_evidence_complete: " << bool_yaml(c.yaw_correction_allow_yaw_match_evidence_complete) << "\n"
      << "  min_match_observed_yaw_delta_deg: " << c.yaw_correction_min_match_observed_yaw_delta_deg << "\n"
      << "  min_match_valid_samples: " << c.yaw_correction_min_match_valid_samples << "\n"
      << "  min_match_valid_bins: " << c.yaw_correction_min_match_valid_bins << "\n"
      << "  min_match_valid_bin_ratio: " << c.yaw_correction_min_match_valid_bin_ratio << "\n"
      << "  require_yaw_match_attempted: " << bool_yaml(c.yaw_correction_require_yaw_match_attempted) << "\n"
      << "  max_linear_speed_mps: " << c.yaw_correction_max_linear_speed_mps << "\n"
      << "  max_abs_yaw_rate_dps: " << c.yaw_correction_max_abs_yaw_rate_dps << "\n"
      << "  min_best_score: " << c.yaw_correction_min_best_score << "\n"
      << "  min_score_margin: " << c.yaw_correction_min_score_margin << "\n"
      << "  min_inlier_ratio: " << c.yaw_correction_min_inlier_ratio << "\n"
      << "  max_curve_flatness: " << c.yaw_correction_max_curve_flatness << "\n"
      << "  max_candidate_abs_deg: " << c.yaw_correction_max_candidate_abs_deg << "\n"
      << "  min_candidate_abs_deg: " << c.yaw_correction_min_candidate_abs_deg << "\n"
      << "  correction_gain: " << c.yaw_correction_gain << "\n"
      << "  max_step_deg: " << c.yaw_correction_max_step_deg << "\n"
      << "  min_step_deg: " << c.yaw_correction_min_step_deg << "\n"
      << "  consistency_window: " << c.yaw_correction_consistency_window << "\n"
      << "  max_consistency_spread_deg: " << c.yaw_correction_max_consistency_spread_deg << "\n"
      << "  cooldown_s: " << c.yaw_correction_cooldown_s << "\n"
      << "  writeback_enabled: " << bool_yaml(c.yaw_correction_writeback_enabled) << "\n"
      << "  acknowledgement: " << c.yaw_correction_acknowledgement << "\n"
      << "  writeback_acknowledgement: " << c.yaw_correction_writeback_acknowledgement << "\n"
      << "  required_writeback_acknowledgement: " << c.yaw_correction_required_writeback_acknowledgement << "\n"
      << "  max_writeback_abs_deg: " << c.yaw_correction_max_writeback_abs_deg << "\n"
      << "  max_total_writeback_per_session_deg: " << c.yaw_correction_max_total_writeback_per_session_deg << "\n"
      << "  max_writeback_count_per_session: " << c.yaw_correction_max_writeback_count_per_session << "\n"
      << "  min_seconds_between_writebacks: " << c.yaw_correction_min_seconds_between_writebacks << "\n"
      << "  require_gate_would_apply: " << bool_yaml(c.yaw_correction_require_gate_would_apply) << "\n"
      << "  require_gate_reason: " << c.yaw_correction_require_gate_reason << "\n"
      << "  require_scan_evidence_ok: " << bool_yaml(c.yaw_correction_require_scan_evidence_ok) << "\n"
      << "  require_yaw_match_evidence_ok: " << bool_yaml(c.yaw_correction_require_yaw_match_evidence_ok) << "\n"
      << "  apply_log_enabled: " << bool_yaml(c.yaw_correction_apply_log_enabled) << "\n";
    o << "yaw_correction_post_apply:\n"
      << "  enabled: " << bool_yaml(c.yaw_correction_post_apply_enabled) << "\n"
      << "  timeout_s: " << c.yaw_correction_post_apply_timeout_s << "\n"
      << "  min_improvement_deg: " << c.yaw_correction_post_apply_min_improvement_deg << "\n"
      << "  min_improvement_fraction_of_applied_delta: " << c.yaw_correction_post_apply_min_improvement_fraction_of_applied_delta << "\n"
      << "  min_absolute_improvement_deg: " << c.yaw_correction_post_apply_min_absolute_improvement_deg << "\n"
      << "  max_allowed_worse_deg: " << c.yaw_correction_post_apply_max_allowed_worse_deg << "\n"
      << "  require_new_scan_id: " << bool_yaml(c.yaw_correction_post_apply_require_new_scan_id) << "\n"
      << "  require_newer_match_timestamp: " << bool_yaml(c.yaw_correction_post_apply_require_newer_match_timestamp) << "\n"
      << "  max_post_apply_candidate_abs_deg: " << c.yaw_correction_post_apply_max_post_apply_candidate_abs_deg << "\n"
      << "  log_enabled: " << bool_yaml(c.yaw_correction_post_apply_log_enabled) << "\n";
    o << "recovery:\n"
      << "  enabled: " << bool_yaml(c.recovery_enabled) << "\n"
      << "  mode: " << c.recovery_mode << "\n"
      << "  log_hz: " << c.recovery_log_hz << "\n"
      << "  startup_recovery_enabled: " << bool_yaml(c.recovery_startup_recovery_enabled) << "\n"
      << "  lost_recovery_enabled: " << bool_yaml(c.recovery_lost_recovery_enabled) << "\n"
      << "  degraded_recovery_enabled: " << bool_yaml(c.recovery_degraded_recovery_enabled) << "\n"
      << "  startup_grace_s: " << c.recovery_startup_grace_s << "\n"
      << "  lost_confirm_s: " << c.recovery_lost_confirm_s << "\n"
      << "  degraded_confirm_s: " << c.recovery_degraded_confirm_s << "\n"
      << "  require_map_quality_not_invalid: " << bool_yaml(c.recovery_require_map_quality_not_invalid) << "\n"
      << "  require_min_known_cells: " << c.recovery_require_min_known_cells << "\n"
      << "  require_min_occupied_cells: " << c.recovery_require_min_occupied_cells << "\n"
      << "  require_tof_recent: " << bool_yaml(c.recovery_require_tof_recent) << "\n"
      << "  max_tof_age_s: " << c.recovery_max_tof_age_s << "\n"
      << "  require_localizer_initialized_for_local_recovery: " << bool_yaml(c.recovery_require_localizer_initialized_for_local_recovery) << "\n"
      << "  allow_uninitialized_startup_recovery_observe_only: " << bool_yaml(c.recovery_allow_uninitialized_startup_recovery_observe_only) << "\n"
      << "  request_recovery_scan: " << bool_yaml(c.recovery_request_recovery_scan) << "\n"
      << "  recovery_scan_cooldown_s: " << c.recovery_recovery_scan_cooldown_s << "\n"
      << "  require_yaw_correction_stable: " << bool_yaml(c.recovery_require_yaw_correction_stable) << "\n"
      << "  max_recent_yaw_apply_count: " << c.recovery_max_recent_yaw_apply_count << "\n"
      << "  min_post_apply_validated_count: " << c.recovery_min_post_apply_validated_count << "\n"
      << "  block_if_post_apply_failed_recent: " << bool_yaml(c.recovery_block_if_post_apply_failed_recent) << "\n"
      << "  write_candidate_log: " << bool_yaml(c.recovery_write_candidate_log) << "\n"
      << "  write_event_log: " << bool_yaml(c.recovery_write_event_log) << "\n";
    o << "autonomous_slam:\n"
      << "  enabled: " << bool_yaml(c.autonomous_slam_enabled) << "\n"
      << "  max_iterations: " << c.autonomous_slam_max_iterations << "\n"
      << "  sensor_timeout_s: " << c.autonomous_slam_sensor_timeout_s << "\n"
      << "  motion_settle_timeout_s: " << c.autonomous_slam_motion_settle_timeout_s << "\n"
      << "  active_scan_speed: " << c.autonomous_slam_active_scan_speed << "\n"
      << "  active_scan_duration_s: " << c.autonomous_slam_active_scan_duration_s << "\n"
      << "  max_active_scan_commands: " << c.autonomous_slam_max_active_scan_commands << "\n"
      << "  prefer_left_turn: " << bool_yaml(c.autonomous_slam_prefer_left_turn) << "\n"
      << "  require_tof: " << bool_yaml(c.autonomous_slam_require_tof) << "\n"
      << "  require_imu_or_wheel: " << bool_yaml(c.autonomous_slam_require_imu_or_wheel) << "\n"
      << "  allow_forward_backward: " << bool_yaml(c.autonomous_slam_allow_forward_backward) << "\n";
    o << "real_adapter_contract:\n"
      << "  enabled: " << bool_yaml(c.real_adapter_contract_enabled) << "\n"
      << "  require_tof: " << bool_yaml(c.real_adapter_contract_require_tof) << "\n"
      << "  require_imu_or_wheel: " << bool_yaml(c.real_adapter_contract_require_imu_or_wheel) << "\n"
      << "  tof_max_frame_age_s: " << c.real_adapter_contract_tof_max_frame_age_s << "\n"
      << "  imu_max_frame_age_s: " << c.real_adapter_contract_imu_max_frame_age_s << "\n"
      << "  wheel_max_frame_age_s: " << c.real_adapter_contract_wheel_max_frame_age_s << "\n"
      << "  tof_min_range_count: " << c.real_adapter_contract_tof_min_range_count << "\n"
      << "  tof_max_range_count: " << c.real_adapter_contract_tof_max_range_count << "\n"
      << "  tof_min_range_m: " << c.real_adapter_contract_tof_min_range_m << "\n"
      << "  tof_max_range_m: " << c.real_adapter_contract_tof_max_range_m << "\n"
      << "  tof_max_allowed_nan_ratio: " << c.real_adapter_contract_tof_max_allowed_nan_ratio << "\n"
      << "  require_tof_frame_id: " << bool_yaml(c.real_adapter_contract_require_tof_frame_id) << "\n"
      << "  imu_max_abs_yaw_rate_rad_s: " << c.real_adapter_contract_imu_max_abs_yaw_rate_rad_s << "\n"
      << "  imu_max_abs_acc_mps2: " << c.real_adapter_contract_imu_max_abs_acc_mps2 << "\n"
      << "  wheel_max_abs_linear_mps: " << c.real_adapter_contract_wheel_max_abs_linear_mps << "\n"
      << "  wheel_max_abs_angular_rad_s: " << c.real_adapter_contract_wheel_max_abs_angular_rad_s << "\n"
      << "  require_shadow_before_live: " << bool_yaml(c.real_adapter_contract_require_shadow_before_live) << "\n";
    o << "prelive_autonomous_slam:\n"
      << "  enabled: " << bool_yaml(c.prelive_autonomous_slam_enabled) << "\n"
      << "  max_iterations: " << c.prelive_autonomous_slam_max_iterations << "\n"
      << "  start_time_s: " << c.prelive_autonomous_slam_start_time_s << "\n"
      << "  step_s: " << c.prelive_autonomous_slam_step_s << "\n"
      << "  require_adapter_acceptance: " << bool_yaml(c.prelive_autonomous_slam_require_adapter_acceptance) << "\n"
      << "  require_coordinator_completed: " << bool_yaml(c.prelive_autonomous_slam_require_coordinator_completed) << "\n"
      << "  require_no_motion_rejection: " << bool_yaml(c.prelive_autonomous_slam_require_no_motion_rejection) << "\n"
      << "  require_stop_command_seen: " << bool_yaml(c.prelive_autonomous_slam_require_stop_command_seen) << "\n"
      << "  require_active_scan_command_seen: " << bool_yaml(c.prelive_autonomous_slam_require_active_scan_command_seen) << "\n"
      << "  require_map_quality_good: " << bool_yaml(c.prelive_autonomous_slam_require_map_quality_good) << "\n"
      << "  allow_coordinator_incomplete_for_shadow: " << bool_yaml(c.prelive_autonomous_slam_allow_coordinator_incomplete_for_shadow) << "\n";
    o << "slam_backend_binding:\n"
      << "  enabled: " << bool_yaml(c.slam_backend_binding_enabled) << "\n"
      << "  require_tof: " << bool_yaml(c.slam_backend_binding_require_tof) << "\n"
      << "  require_imu_or_wheel: " << bool_yaml(c.slam_backend_binding_require_imu_or_wheel) << "\n"
      << "  allow_predicted_pose_missing: " << bool_yaml(c.slam_backend_binding_allow_predicted_pose_missing) << "\n"
      << "  max_input_age_s: " << c.slam_backend_binding_max_input_age_s << "\n"
      << "  max_update_latency_s: " << c.slam_backend_binding_max_update_latency_s << "\n"
      << "  min_integrated_scan_count_for_quality: " << c.slam_backend_binding_min_integrated_scan_count_for_quality << "\n"
      << "  require_ready_for_acceptance: " << bool_yaml(c.slam_backend_binding_require_ready_for_acceptance) << "\n"
      << "  require_update_accepted: " << bool_yaml(c.slam_backend_binding_require_update_accepted) << "\n"
      << "  require_quality_valid: " << bool_yaml(c.slam_backend_binding_require_quality_valid) << "\n"
      << "  require_save_map: " << bool_yaml(c.slam_backend_binding_require_save_map) << "\n";
    o << "autonomous_slam_e2e_prelive:\n"
      << "  enabled: " << bool_yaml(c.autonomous_slam_e2e_prelive_enabled) << "\n"
      << "  scenario_kind: " << c.autonomous_slam_e2e_prelive_scenario_kind << "\n"
      << "  max_iterations: " << c.autonomous_slam_e2e_prelive_max_iterations << "\n"
      << "  start_time_s: " << c.autonomous_slam_e2e_prelive_start_time_s << "\n"
      << "  step_s: " << c.autonomous_slam_e2e_prelive_step_s << "\n"
      << "  require_slam_backend_acceptance: " << bool_yaml(c.autonomous_slam_e2e_prelive_require_slam_backend_acceptance) << "\n"
      << "  require_prelive_pass: " << bool_yaml(c.autonomous_slam_e2e_prelive_require_prelive_pass) << "\n"
      << "  require_no_forward_backward: " << bool_yaml(c.autonomous_slam_e2e_prelive_require_no_forward_backward) << "\n"
      << "  require_stop_command_seen: " << bool_yaml(c.autonomous_slam_e2e_prelive_require_stop_command_seen) << "\n"
      << "  require_active_scan_when_map_poor: " << bool_yaml(c.autonomous_slam_e2e_prelive_require_active_scan_when_map_poor) << "\n"
      << "  require_map_quality_good_at_end: " << bool_yaml(c.autonomous_slam_e2e_prelive_require_map_quality_good_at_end) << "\n";
    o << "real_adapter_stubs:\n"
      << "  enabled: " << bool_yaml(c.real_adapter_stubs_enabled) << "\n"
      << "  create_sensor_stub: " << bool_yaml(c.real_adapter_stubs_create_sensor_stub) << "\n"
      << "  create_motion_stub: " << bool_yaml(c.real_adapter_stubs_create_motion_stub) << "\n"
      << "  create_slam_backend_stub: " << bool_yaml(c.real_adapter_stubs_create_slam_backend_stub) << "\n"
      << "  allow_real_hardware_adapters: " << bool_yaml(c.real_adapter_stubs_allow_real_hardware_adapters) << "\n"
      << "  require_explicit_live_enable: " << bool_yaml(c.real_adapter_stubs_require_explicit_live_enable) << "\n";
    o << "live_handoff_readiness:\n"
      << "  enabled: " << bool_yaml(c.live_handoff_readiness_enabled) << "\n"
      << "  require_real_sensor_adapter: " << bool_yaml(c.live_handoff_readiness_require_real_sensor_adapter) << "\n"
      << "  require_real_motion_adapter: " << bool_yaml(c.live_handoff_readiness_require_real_motion_adapter) << "\n"
      << "  require_real_slam_backend: " << bool_yaml(c.live_handoff_readiness_require_real_slam_backend) << "\n"
      << "  require_software_transport_acceptance: " << bool_yaml(c.live_handoff_readiness_require_software_transport_acceptance) << "\n"
      << "  require_e2e_prelive_pass: " << bool_yaml(c.live_handoff_readiness_require_e2e_prelive_pass) << "\n"
      << "  require_direction_probe: " << bool_yaml(c.live_handoff_readiness_require_direction_probe) << "\n"
      << "  require_stop_estop_ttl: " << bool_yaml(c.live_handoff_readiness_require_stop_estop_ttl) << "\n"
      << "  require_hardware_safety: " << bool_yaml(c.live_handoff_readiness_require_hardware_safety) << "\n"
      << "  allow_forward_backward: " << bool_yaml(c.live_handoff_readiness_allow_forward_backward) << "\n";
    o << "real_sensor_adapter_data_contract:\n"
      << "  enabled: " << bool_yaml(c.real_sensor_adapter_data_contract_enabled) << "\n"
      << "  require_tof: " << bool_yaml(c.real_sensor_adapter_data_contract_require_tof) << "\n"
      << "  require_imu_or_wheel: " << bool_yaml(c.real_sensor_adapter_data_contract_require_imu_or_wheel) << "\n"
      << "  require_frame_id: " << bool_yaml(c.real_sensor_adapter_data_contract_require_frame_id) << "\n"
      << "  require_request_timing: " << bool_yaml(c.real_sensor_adapter_data_contract_require_request_timing) << "\n"
      << "  allow_nan_ranges: " << bool_yaml(c.real_sensor_adapter_data_contract_allow_nan_ranges) << "\n"
      << "  max_packet_age_s: " << c.real_sensor_adapter_data_contract_max_packet_age_s << "\n"
      << "  max_sensor_sync_dt_s: " << c.real_sensor_adapter_data_contract_max_sensor_sync_dt_s << "\n"
      << "  max_request_latency_s: " << c.real_sensor_adapter_data_contract_max_request_latency_s << "\n"
      << "  max_tof_nan_ratio: " << c.real_sensor_adapter_data_contract_max_tof_nan_ratio << "\n"
      << "  min_tof_range_m: " << c.real_sensor_adapter_data_contract_min_tof_range_m << "\n"
      << "  max_tof_range_m: " << c.real_sensor_adapter_data_contract_max_tof_range_m << "\n"
      << "  max_abs_yaw_rate_rad_s: " << c.real_sensor_adapter_data_contract_max_abs_yaw_rate_rad_s << "\n"
      << "  max_accel_norm_mps2: " << c.real_sensor_adapter_data_contract_max_accel_norm_mps2 << "\n"
      << "  max_abs_wheel_linear_mps: " << c.real_sensor_adapter_data_contract_max_abs_wheel_linear_mps << "\n"
      << "  max_abs_wheel_angular_rad_s: " << c.real_sensor_adapter_data_contract_max_abs_wheel_angular_rad_s << "\n"
      << "  default_tof_frame_id: " << c.real_sensor_adapter_data_contract_default_tof_frame_id << "\n"
      << "  default_imu_frame_id: " << c.real_sensor_adapter_data_contract_default_imu_frame_id << "\n"
      << "  default_wheel_frame_id: " << c.real_sensor_adapter_data_contract_default_wheel_frame_id << "\n"
      << "  run_acceptance_on_startup: " << bool_yaml(c.real_sensor_adapter_data_contract_run_acceptance_on_startup) << "\n"
      << "  max_request_latency_mismatch_s: " << c.real_sensor_adapter_data_contract_max_request_latency_mismatch_s << "\n"
      << "  max_estimated_sample_time_midpoint_error_s: " << c.real_sensor_adapter_data_contract_max_estimated_sample_time_midpoint_error_s << "\n"
      << "  max_future_timestamp_skew_s: " << c.real_sensor_adapter_data_contract_max_future_timestamp_skew_s << "\n"
      << "  max_packet_sensor_time_dt_s: " << c.real_sensor_adapter_data_contract_max_packet_sensor_time_dt_s << "\n"
      << "  reject_request_latency_mismatch: " << bool_yaml(c.real_sensor_adapter_data_contract_reject_request_latency_mismatch) << "\n"
      << "  require_estimated_sample_time_in_window: " << bool_yaml(c.real_sensor_adapter_data_contract_require_estimated_sample_time_in_window) << "\n"
      << "  require_estimated_sample_time_midpoint: " << bool_yaml(c.real_sensor_adapter_data_contract_require_estimated_sample_time_midpoint) << "\n"
      << "  reject_future_sensor_time: " << bool_yaml(c.real_sensor_adapter_data_contract_reject_future_sensor_time) << "\n";
    o << "real_sensor_replay:\n"
      << "  enabled: " << bool_yaml(c.real_sensor_replay_enabled) << "\n"
      << "  loop: " << bool_yaml(c.real_sensor_replay_loop) << "\n"
      << "  fail_on_contract_error: " << bool_yaml(c.real_sensor_replay_fail_on_contract_error) << "\n"
      << "  require_non_empty_log: " << bool_yaml(c.real_sensor_replay_require_non_empty_log) << "\n"
      << "  run_acceptance_on_startup: " << bool_yaml(c.real_sensor_replay_run_acceptance_on_startup) << "\n"
      << "  start_time_s: " << c.real_sensor_replay_start_time_s << "\n"
      << "  time_mode: " << c.real_sensor_replay_time_mode << "\n"
      << "  reject_invalid_records: " << bool_yaml(c.real_sensor_replay_reject_invalid_records) << "\n"
      << "  require_packet_records: " << bool_yaml(c.real_sensor_replay_require_packet_records) << "\n"
      << "  preserve_parse_errors: " << bool_yaml(c.real_sensor_replay_preserve_parse_errors) << "\n"
      << "  max_records_per_run: " << c.real_sensor_replay_max_records_per_run << "\n";
    o << "real_sensor_replay_regression:\n"
      << "  enabled: " << bool_yaml(c.real_sensor_replay_regression_enabled) << "\n"
      << "  require_valid_log_pass: " << bool_yaml(c.real_sensor_replay_regression_require_valid_log_pass) << "\n"
      << "  require_invalid_logs_fail: " << bool_yaml(c.real_sensor_replay_regression_require_invalid_logs_fail) << "\n"
      << "  require_parse_errors_detected: " << bool_yaml(c.real_sensor_replay_regression_require_parse_errors_detected) << "\n"
      << "  require_comment_only_log_rejected: " << bool_yaml(c.real_sensor_replay_regression_require_comment_only_log_rejected) << "\n"
      << "  run_on_startup: " << bool_yaml(c.real_sensor_replay_regression_run_on_startup) << "\n";
    o << "deterministic_slam_backend:\n"
      << "  enabled: " << bool_yaml(c.deterministic_slam_backend_enabled) << "\n"
      << "  ready: " << bool_yaml(c.deterministic_slam_backend_ready) << "\n"
      << "  require_tof: " << bool_yaml(c.deterministic_slam_backend_require_tof) << "\n"
      << "  require_imu_or_wheel: " << bool_yaml(c.deterministic_slam_backend_require_imu_or_wheel) << "\n"
      << "  allow_save_map: " << bool_yaml(c.deterministic_slam_backend_allow_save_map) << "\n"
      << "  min_valid_ranges: " << c.deterministic_slam_backend_min_valid_ranges << "\n"
      << "  min_valid_scan_count_for_good: " << c.deterministic_slam_backend_min_valid_scan_count_for_good << "\n"
      << "  min_valid_range_ratio: " << c.deterministic_slam_backend_min_valid_range_ratio << "\n"
      << "  min_coverage_ratio_for_good: " << c.deterministic_slam_backend_min_coverage_ratio_for_good << "\n"
      << "  min_yaw_coverage_ratio_for_good: " << c.deterministic_slam_backend_min_yaw_coverage_ratio_for_good << "\n"
      << "  keyframe_yaw_delta_rad: " << c.deterministic_slam_backend_keyframe_yaw_delta_rad << "\n"
      << "  min_range_m: " << c.deterministic_slam_backend_min_range_m << "\n"
      << "  max_range_m: " << c.deterministic_slam_backend_max_range_m << "\n"
      << "  max_update_latency_s: " << c.deterministic_slam_backend_max_update_latency_s << "\n"
      << "  assumed_scan_yaw_span_rad: " << c.deterministic_slam_backend_assumed_scan_yaw_span_rad << "\n"
      << "  yaw_bin_size_rad: " << c.deterministic_slam_backend_yaw_bin_size_rad << "\n"
      << "  run_regression_on_startup: " << bool_yaml(c.deterministic_slam_backend_run_regression_on_startup) << "\n";
    o << "replay_to_slam_backend_regression:\n"
      << "  enabled: " << bool_yaml(c.replay_to_slam_backend_regression_enabled) << "\n"
      << "  require_valid_replay_updates_map: " << bool_yaml(c.replay_to_slam_backend_regression_require_valid_replay_updates_map) << "\n"
      << "  require_invalid_replay_rejected: " << bool_yaml(c.replay_to_slam_backend_regression_require_invalid_replay_rejected) << "\n"
      << "  min_accepted_updates: " << c.replay_to_slam_backend_regression_min_accepted_updates << "\n"
      << "  run_on_startup: " << bool_yaml(c.replay_to_slam_backend_regression_run_on_startup) << "\n";
    o << "full_autonomous_slam_fake_pipeline:\n"
      << "  enabled: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_enabled) << "\n"
      << "  run_on_startup: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_run_on_startup) << "\n"
      << "  max_steps: " << c.full_autonomous_slam_fake_pipeline_max_steps << "\n"
      << "  max_active_scan_commands: " << c.full_autonomous_slam_fake_pipeline_max_active_scan_commands << "\n"
      << "  min_backend_accepted_updates: " << c.full_autonomous_slam_fake_pipeline_min_backend_accepted_updates << "\n"
      << "  require_completion: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_require_completion) << "\n"
      << "  require_shadow_motion_only: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_require_shadow_motion_only) << "\n"
      << "  require_no_forward_backward: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_require_no_forward_backward) << "\n"
      << "  require_map_quality_good: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_require_map_quality_good) << "\n"
      << "  motion_settle_s: " << c.full_autonomous_slam_fake_pipeline_motion_settle_s << "\n"
      << "  build_fake_map_on_completed: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_build_fake_map_on_completed) << "\n"
      << "  save_fake_map_on_completed: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_save_fake_map_on_completed) << "\n"
      << "  require_fake_map_saved: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_require_fake_map_saved) << "\n"
      << "  fake_map_id_prefix: " << c.full_autonomous_slam_fake_pipeline_fake_map_id_prefix << "\n"
      << "  phase_aware_sensor_consumption: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_phase_aware_sensor_consumption) << "\n"
      << "  require_phase_aware_sensor_consumption: " << bool_yaml(c.full_autonomous_slam_fake_pipeline_require_phase_aware_sensor_consumption) << "\n";
    o << "fake_map_artifact:\n"
      << "  enabled: " << bool_yaml(c.fake_map_artifact_enabled) << "\n"
      << "  allow_overwrite: " << bool_yaml(c.fake_map_artifact_allow_overwrite) << "\n"
      << "  require_quality_good: " << bool_yaml(c.fake_map_artifact_require_quality_good) << "\n"
      << "  require_completed_pipeline: " << bool_yaml(c.fake_map_artifact_require_completed_pipeline) << "\n"
      << "  load_enabled: " << bool_yaml(c.fake_map_artifact_load_enabled) << "\n";
    o << "fake_relocalization:\n"
      << "  enabled: " << bool_yaml(c.fake_relocalization_enabled) << "\n"
      << "  allow_pose_writeback: " << bool_yaml(c.fake_relocalization_allow_pose_writeback) << "\n"
      << "  require_map_quality_good: " << bool_yaml(c.fake_relocalization_require_map_quality_good) << "\n"
      << "  require_tof: " << bool_yaml(c.fake_relocalization_require_tof) << "\n"
      << "  min_valid_ranges: " << c.fake_relocalization_min_valid_ranges << "\n"
      << "  min_confidence: " << c.fake_relocalization_min_confidence << "\n"
      << "  high_confidence_threshold: " << c.fake_relocalization_high_confidence_threshold << "\n"
      << "  min_map_coverage_ratio: " << c.fake_relocalization_min_map_coverage_ratio << "\n"
      << "  min_map_yaw_coverage_ratio: " << c.fake_relocalization_min_map_yaw_coverage_ratio << "\n"
      << "  run_on_startup: " << bool_yaml(c.fake_relocalization_run_on_startup) << "\n";
    o << "fake_map_relocalization_runner:\n"
      << "  enabled: " << bool_yaml(c.fake_map_relocalization_runner_enabled) << "\n"
      << "  require_pipeline_map_artifact: " << bool_yaml(c.fake_map_relocalization_runner_require_pipeline_map_artifact) << "\n"
      << "  require_relocalization_success: " << bool_yaml(c.fake_map_relocalization_runner_require_relocalization_success) << "\n"
      << "  require_no_pose_writeback: " << bool_yaml(c.fake_map_relocalization_runner_require_no_pose_writeback) << "\n"
      << "  run_on_startup: " << bool_yaml(c.fake_map_relocalization_runner_run_on_startup) << "\n";
    o << "fake_relocalization_readiness_gate:\n"
      << "  enabled: " << bool_yaml(c.fake_relocalization_readiness_gate_enabled) << "\n"
      << "  require_binding_ready: " << bool_yaml(c.fake_relocalization_readiness_gate_require_binding_ready) << "\n"
      << "  require_no_pose_writeback: " << bool_yaml(c.fake_relocalization_readiness_gate_require_no_pose_writeback) << "\n"
      << "  require_map_quality_good: " << bool_yaml(c.fake_relocalization_readiness_gate_require_map_quality_good) << "\n"
      << "  min_confidence: " << c.fake_relocalization_readiness_gate_min_confidence << "\n"
      << "  min_map_coverage_ratio: " << c.fake_relocalization_readiness_gate_min_map_coverage_ratio << "\n"
      << "  min_map_yaw_coverage_ratio: " << c.fake_relocalization_readiness_gate_min_map_yaw_coverage_ratio << "\n";
    o << "fake_autonomous_slam_product_acceptance:\n"
      << "  enabled: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_enabled) << "\n"
      << "  run_on_startup: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_run_on_startup) << "\n"
      << "  require_mapping_pipeline_success: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_require_mapping_pipeline_success) << "\n"
      << "  require_fake_map_saved: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_require_fake_map_saved) << "\n"
      << "  require_relocalization_success: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_require_relocalization_success) << "\n"
      << "  require_relocalization_readiness: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_require_relocalization_readiness) << "\n"
      << "  require_no_pose_writeback: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_require_no_pose_writeback) << "\n"
      << "  require_no_forward_backward: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_require_no_forward_backward) << "\n"
      << "  require_adapter_manifest: " << bool_yaml(c.fake_autonomous_slam_product_acceptance_require_adapter_manifest) << "\n";
    o << "multi_tof_raw_data_contract:\n"
      << "  enabled: " << bool_yaml(c.multi_tof_raw_data_contract_enabled) << "\n"
      << "  require_front: " << bool_yaml(c.multi_tof_raw_data_contract_require_front) << "\n"
      << "  require_left: " << bool_yaml(c.multi_tof_raw_data_contract_require_left) << "\n"
      << "  require_right: " << bool_yaml(c.multi_tof_raw_data_contract_require_right) << "\n"
      << "  require_unique_mount_ids: " << bool_yaml(c.multi_tof_raw_data_contract_require_unique_mount_ids) << "\n"
      << "  require_unique_frame_ids: " << bool_yaml(c.multi_tof_raw_data_contract_require_unique_frame_ids) << "\n"
      << "  require_request_timing: " << bool_yaml(c.multi_tof_raw_data_contract_require_request_timing) << "\n"
      << "  allow_nan_ranges: " << bool_yaml(c.multi_tof_raw_data_contract_allow_nan_ranges) << "\n"
      << "  min_required_tof_count: " << c.multi_tof_raw_data_contract_min_required_tof_count << "\n"
      << "  front_mount_yaw_rad: " << c.multi_tof_raw_data_contract_front_mount_yaw_rad << "\n"
      << "  left_mount_yaw_rad: " << c.multi_tof_raw_data_contract_left_mount_yaw_rad << "\n"
      << "  right_mount_yaw_rad: " << c.multi_tof_raw_data_contract_right_mount_yaw_rad << "\n"
      << "  max_mount_yaw_error_rad: " << c.multi_tof_raw_data_contract_max_mount_yaw_error_rad << "\n"
      << "  max_packet_age_s: " << c.multi_tof_raw_data_contract_max_packet_age_s << "\n"
      << "  max_request_latency_s: " << c.multi_tof_raw_data_contract_max_request_latency_s << "\n"
      << "  max_request_latency_mismatch_s: " << c.multi_tof_raw_data_contract_max_request_latency_mismatch_s << "\n"
      << "  max_estimated_sample_time_midpoint_error_s: " << c.multi_tof_raw_data_contract_max_estimated_sample_time_midpoint_error_s << "\n"
      << "  max_future_timestamp_skew_s: " << c.multi_tof_raw_data_contract_max_future_timestamp_skew_s << "\n"
      << "  max_nan_ratio: " << c.multi_tof_raw_data_contract_max_nan_ratio << "\n"
      << "  min_range_m: " << c.multi_tof_raw_data_contract_min_range_m << "\n"
      << "  max_range_m: " << c.multi_tof_raw_data_contract_max_range_m << "\n"
      << "  front_frame_id: " << c.multi_tof_raw_data_contract_front_frame_id << "\n"
      << "  left_frame_id: " << c.multi_tof_raw_data_contract_left_frame_id << "\n"
      << "  right_frame_id: " << c.multi_tof_raw_data_contract_right_frame_id << "\n"
      << "  run_acceptance_on_startup: " << bool_yaml(c.multi_tof_raw_data_contract_run_acceptance_on_startup) << "\n";
    o << "multi_tof_sync:\n"
      << "  enabled: " << bool_yaml(c.multi_tof_sync_enabled) << "\n"
      << "  require_raw_contract_pass: " << bool_yaml(c.multi_tof_sync_require_raw_contract_pass) << "\n"
      << "  require_imu: " << bool_yaml(c.multi_tof_sync_require_imu) << "\n"
      << "  require_wheel: " << bool_yaml(c.multi_tof_sync_require_wheel) << "\n"
      << "  time_reference: " << c.multi_tof_sync_time_reference << "\n"
      << "  degradation_mode: " << c.multi_tof_sync_degradation_mode << "\n"
      << "  min_required_tof_count: " << c.multi_tof_sync_min_required_tof_count << "\n"
      << "  max_pairwise_tof_sync_dt_s: " << c.multi_tof_sync_max_pairwise_tof_sync_dt_s << "\n"
      << "  max_multi_tof_imu_sync_dt_s: " << c.multi_tof_sync_max_multi_tof_imu_sync_dt_s << "\n"
      << "  max_multi_tof_wheel_sync_dt_s: " << c.multi_tof_sync_max_multi_tof_wheel_sync_dt_s << "\n"
      << "  max_effective_time_future_skew_s: " << c.multi_tof_sync_max_effective_time_future_skew_s << "\n"
      << "  run_acceptance_on_startup: " << bool_yaml(c.multi_tof_sync_run_acceptance_on_startup) << "\n";
    o << "multi_tof_snapshot_builder:\n"
      << "  enabled: " << bool_yaml(c.multi_tof_snapshot_builder_enabled) << "\n"
      << "  require_sync_pass: " << bool_yaml(c.multi_tof_snapshot_builder_require_sync_pass) << "\n"
      << "  populate_legacy_tof: " << bool_yaml(c.multi_tof_snapshot_builder_populate_legacy_tof) << "\n"
      << "  require_legacy_primary: " << bool_yaml(c.multi_tof_snapshot_builder_require_legacy_primary) << "\n"
      << "  legacy_primary_mode: " << c.multi_tof_snapshot_builder_legacy_primary_mode << "\n"
      << "  min_required_tof_count: " << c.multi_tof_snapshot_builder_min_required_tof_count << "\n"
      << "  run_acceptance_on_startup: " << bool_yaml(c.multi_tof_snapshot_builder_run_acceptance_on_startup) << "\n";
    o << "multi_tof_replay:\n"
      << "  enabled: " << bool_yaml(c.multi_tof_replay_enabled) << "\n"
      << "  reject_invalid_records: " << bool_yaml(c.multi_tof_replay_reject_invalid_records) << "\n"
      << "  require_snapshot_build: " << bool_yaml(c.multi_tof_replay_require_snapshot_build) << "\n"
      << "  time_mode: " << c.multi_tof_replay_time_mode << "\n"
      << "  run_acceptance_on_startup: " << bool_yaml(c.multi_tof_replay_run_acceptance_on_startup) << "\n";
    o << "motion_execution:\n"
      << "  enabled: " << bool_yaml(c.motion_execution_enabled) << "\n"
      << "  mode: " << c.motion_execution_mode << "\n"
      << "  log_hz: " << c.motion_execution_log_hz << "\n"
      << "  command_source: " << c.motion_execution_command_source << "\n"
      << "  allow_recovery_scan_request_as_reason: " << bool_yaml(c.motion_execution_allow_recovery_scan_request_as_reason) << "\n"
      << "  require_active_scan_command: " << bool_yaml(c.motion_execution_require_active_scan_command) << "\n"
      << "  require_command_planner_verifying_or_commanding: " << bool_yaml(c.motion_execution_require_command_planner_verifying_or_commanding) << "\n"
      << "  wheel_base_m: " << c.motion_execution_wheel_base_m << "\n"
      << "  wheel_radius_m: " << c.motion_execution_wheel_radius_m << "\n"
      << "  max_linear_speed_mps: " << c.motion_execution_max_linear_speed_mps << "\n"
      << "  max_abs_yaw_rate_dps: " << c.motion_execution_max_abs_yaw_rate_dps << "\n"
      << "  max_wheel_speed_rpm: " << c.motion_execution_max_wheel_speed_rpm << "\n"
      << "  min_wheel_speed_rpm: " << c.motion_execution_min_wheel_speed_rpm << "\n"
      << "  max_command_duration_s: " << c.motion_execution_max_command_duration_s << "\n"
      << "  deadman_timeout_s: " << c.motion_execution_deadman_timeout_s << "\n"
      << "  command_stale_timeout_s: " << c.motion_execution_command_stale_timeout_s << "\n"
      << "  require_localizer_initialized: " << bool_yaml(c.motion_execution_require_localizer_initialized) << "\n"
      << "  require_supervisor_not_lost: " << bool_yaml(c.motion_execution_require_supervisor_not_lost) << "\n"
      << "  require_tof_recent: " << bool_yaml(c.motion_execution_require_tof_recent) << "\n"
      << "  max_tof_age_s: " << c.motion_execution_max_tof_age_s << "\n"
      << "  obstacle_stop_enabled: " << bool_yaml(c.motion_execution_obstacle_stop_enabled) << "\n"
      << "  min_front_distance_m: " << c.motion_execution_min_front_distance_m << "\n"
      << "  min_side_distance_m: " << c.motion_execution_min_side_distance_m << "\n"
      << "  require_encoder_feedback_recent: " << bool_yaml(c.motion_execution_require_encoder_feedback_recent) << "\n"
      << "  max_encoder_age_s: " << c.motion_execution_max_encoder_age_s << "\n"
      << "  current_safety_enabled: " << bool_yaml(c.motion_execution_current_safety_enabled) << "\n"
      << "  max_motor_current_a: " << c.motion_execution_max_motor_current_a << "\n"
      << "  stall_detection_enabled: " << bool_yaml(c.motion_execution_stall_detection_enabled) << "\n"
      << "  stall_speed_rpm: " << c.motion_execution_stall_speed_rpm << "\n"
      << "  stall_current_a: " << c.motion_execution_stall_current_a << "\n"
      << "  stall_confirm_count: " << c.motion_execution_stall_confirm_count << "\n"
      << "  dry_run_write_motor_commands: " << bool_yaml(c.motion_execution_dry_run_write_motor_commands) << "\n"
      << "  hardware_write_enabled: " << bool_yaml(c.motion_execution_hardware_write_enabled) << "\n"
      << "  writeback_acknowledgement: " << c.motion_execution_writeback_acknowledgement << "\n"
      << "  required_writeback_acknowledgement: " << c.motion_execution_required_writeback_acknowledgement << "\n"
      << "  write_mode_acknowledgement: " << c.motion_execution_write_mode_acknowledgement << "\n"
      << "  required_write_mode_acknowledgement: " << c.motion_execution_required_write_mode_acknowledgement << "\n"
      << "  max_allowed_write_yaw_rate_dps: " << c.motion_execution_max_allowed_write_yaw_rate_dps << "\n"
      << "  max_allowed_write_duration_s: " << c.motion_execution_max_allowed_write_duration_s << "\n"
      << "  allow_writer_dispatch: " << bool_yaml(c.motion_execution_allow_writer_dispatch) << "\n"
      << "  writer_backend: " << c.motion_execution_writer_backend << "\n"
      << "  software_motion:\n"
      << "    max_internal_rpm_for_normalization: " << c.motion_execution_software_motion_max_internal_rpm_for_normalization << "\n"
      << "    max_speed_normalized: " << c.motion_execution_software_motion_max_speed_normalized << "\n"
      << "    command_ttl_s: " << c.motion_execution_software_motion_command_ttl_s << "\n"
      << "    allow_rotation_commands: " << bool_yaml(c.motion_execution_software_motion_allow_rotation_commands) << "\n"
      << "    allow_translation_commands: " << bool_yaml(c.motion_execution_software_motion_allow_translation_commands) << "\n"
      << "    allow_emergency_stop: " << bool_yaml(c.motion_execution_software_motion_allow_emergency_stop) << "\n"
      << "    require_opposite_wheel_sign_for_rotation: " << bool_yaml(c.motion_execution_software_motion_require_opposite_wheel_sign_for_rotation) << "\n"
      << "    production_interface_enabled: " << bool_yaml(c.motion_execution_software_motion_production_interface_enabled) << "\n"
      << "    production_transport_backend: " << c.motion_execution_software_motion_production_transport_backend << "\n"
      << "    loopback_shadow_mode: " << bool_yaml(c.motion_execution_software_motion_loopback_shadow_mode) << "\n"
      << "  algorithm_motion:\n"
      << "    enabled: " << bool_yaml(c.motion_execution_algorithm_motion_enabled) << "\n"
      << "    allow_rotation_commands: " << bool_yaml(c.motion_execution_algorithm_motion_allow_rotation_commands) << "\n"
      << "    allow_translation_commands: " << bool_yaml(c.motion_execution_algorithm_motion_allow_translation_commands) << "\n"
      << "    allow_recovery_commands: " << bool_yaml(c.motion_execution_algorithm_motion_allow_recovery_commands) << "\n"
      << "    allow_manual_test_commands: " << bool_yaml(c.motion_execution_algorithm_motion_allow_manual_test_commands) << "\n"
      << "    default_ttl_s: " << c.motion_execution_algorithm_motion_default_ttl_s << "\n"
      << "    direction_probe_speed: " << c.motion_execution_algorithm_motion_direction_probe_speed << "\n"
      << "    direction_probe_duration_s: " << c.motion_execution_algorithm_motion_direction_probe_duration_s << "\n"
      << "    active_scan_speed: " << c.motion_execution_algorithm_motion_active_scan_speed << "\n"
      << "    active_scan_duration_s: " << c.motion_execution_algorithm_motion_active_scan_duration_s << "\n"
      << "    recovery_speed: " << c.motion_execution_algorithm_motion_recovery_speed << "\n"
      << "    recovery_duration_s: " << c.motion_execution_algorithm_motion_recovery_duration_s << "\n"
      << "    manual_test_speed: " << c.motion_execution_algorithm_motion_manual_test_speed << "\n"
      << "    manual_test_duration_s: " << c.motion_execution_algorithm_motion_manual_test_duration_s << "\n"
      << "  software_transport_contract:\n"
      << "    enabled: " << bool_yaml(c.motion_execution_software_transport_contract_enabled) << "\n"
      << "    require_shadow_mode: " << bool_yaml(c.motion_execution_software_transport_contract_require_shadow_mode) << "\n"
      << "    allow_rotation_commands: " << bool_yaml(c.motion_execution_software_transport_contract_allow_rotation_commands) << "\n"
      << "    allow_translation_commands: " << bool_yaml(c.motion_execution_software_transport_contract_allow_translation_commands) << "\n"
      << "    allow_emergency_stop: " << bool_yaml(c.motion_execution_software_transport_contract_allow_emergency_stop) << "\n"
      << "    max_speed_normalized: " << c.motion_execution_software_transport_contract_max_speed_normalized << "\n"
      << "    max_direction_probe_speed: " << c.motion_execution_software_transport_contract_max_direction_probe_speed << "\n"
      << "    max_ttl_s: " << c.motion_execution_software_transport_contract_max_ttl_s << "\n"
      << "    max_command_age_s: " << c.motion_execution_software_transport_contract_max_command_age_s << "\n"
      << "  m2b1_preflight:\n"
      << "    enabled: " << bool_yaml(c.motion_execution_m2b1_preflight_enabled) << "\n"
      << "    mode: " << c.motion_execution_m2b1_preflight_mode << "\n"
      << "    require_operator_present: " << bool_yaml(c.motion_execution_m2b1_preflight_require_operator_present) << "\n"
      << "    require_robot_lifted_or_wheels_free: " << bool_yaml(c.motion_execution_m2b1_preflight_require_robot_lifted_or_wheels_free) << "\n"
      << "    require_emergency_stop_available: " << bool_yaml(c.motion_execution_m2b1_preflight_require_emergency_stop_available) << "\n"
      << "    max_live_speed_normalized: " << c.motion_execution_m2b1_preflight_max_live_speed_normalized << "\n"
      << "    max_live_duration_s: " << c.motion_execution_m2b1_preflight_max_live_duration_s << "\n"
      << "    direction_probe_max_speed_normalized: " << c.motion_execution_m2b1_preflight_direction_probe_max_speed_normalized << "\n"
      << "    direction_probe_max_duration_s: " << c.motion_execution_m2b1_preflight_direction_probe_max_duration_s << "\n"
      << "    require_shadow_transport_first: " << bool_yaml(c.motion_execution_m2b1_preflight_require_shadow_transport_first) << "\n"
      << "    require_left_right_direction_confirmation: " << bool_yaml(c.motion_execution_m2b1_preflight_require_left_right_direction_confirmation) << "\n"
      << "  manual_arm:\n"
      << "    enable_live_motion: " << bool_yaml(c.motion_execution_manual_arm_enable_live_motion) << "\n"
      << "    confirmation_phrase: " << c.motion_execution_manual_arm_confirmation_phrase << "\n"
      << "  apply_log_enabled: " << bool_yaml(c.motion_execution_apply_log_enabled) << "\n";
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
