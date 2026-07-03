#pragma once
#include "active_scan_command_planner.hpp"
#include "active_scan_manager.hpp"
#include "common.hpp"
#include "config.hpp"
#include "grid.hpp"
#include "localizer.hpp"
#include "map_quality.hpp"
#include "mapping_supervisor.hpp"
#include "metrics.hpp"
#include "sensors.hpp"
#include "sparse_scan_collector.hpp"
#include "sparse_scan_yaw_matcher.hpp"
#include "spin_scan_localization.hpp"
#include "tof.hpp"
#include "tof_pose_correction.hpp"
#include "yaw_correction_gate.hpp"
#include "yaw_correction_apply.hpp"

namespace robot_slamd {

int real_main(int argc, char **argv) {
    std::string config_path, output_override;
    double duration_s = -1.0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--duration-s" && i + 1 < argc) duration_s = std::strtod(argv[++i], nullptr);
        else if (a == "--output-dir" && i + 1 < argc) output_override = argv[++i];
        else {
            std::cerr << "usage: robot_slamd --config config.yaml [--duration-s N] [--output-dir PATH]\n";
            return 2;
        }
    }
    if (config_path.empty()) { std::cerr << "--config is required\n"; return 2; }
    Config cfg = load_config(config_path, output_override);
    cfg.output_dir = absolute_path(cfg.output_dir);
    validate_config(cfg);
    std::string err;
    std::string run_root = cfg.output_dir + "/runs";
    if (!mkdir_p(run_root)) { std::cerr << "failed to create output_dir: " << run_root << "\n"; return 1; }
    std::string run_dir;
    if (!create_unique_run_dir(run_root, run_dir, err)) {
        std::cerr << "failed to create run_dir under " << run_root << ": " << err << "\n";
        return 1;
    }
    unlink((cfg.output_dir + "/latest").c_str());
    if (symlink(run_dir.c_str(), (cfg.output_dir + "/latest").c_str()) != 0) {
        std::ofstream(cfg.output_dir + "/latest.txt") << run_dir << "\n";
    }
    write_resolved_config(cfg, run_dir + "/config.resolved.yaml");
    std::ifstream src(config_path, std::ios::binary);
    std::ofstream(run_dir + "/config.input.yaml", std::ios::binary) << src.rdbuf();

    std::ofstream traj(run_dir + "/trajectory.csv");
    std::ofstream loclog(run_dir + "/localization_log.csv");
    std::ofstream toflog(run_dir + "/tof_log.csv");
    std::ofstream map_quality_log;
    std::ofstream supervisor_log;
    std::ofstream active_scan_log;
    std::ofstream active_scan_command_log;
    std::ofstream sparse_scan_samples_log;
    std::ofstream sparse_scan_bins_log;
    std::ofstream sparse_scan_sessions_log;
    std::ofstream yaw_match_curve_log;
    std::ofstream yaw_match_summary_log;
    std::ofstream yaw_correction_gate_log;
    std::ofstream yaw_correction_apply_log;
    std::ofstream encoderlog;
    std::ofstream corrlog;
    std::ofstream yawlog;
    std::ofstream yaw_curve_log;
    std::ofstream yaw_summary_log;
    std::ofstream spin_scan_log;
    std::ofstream spin_bins_log;
    std::ofstream spin_curve_log;
    std::ofstream spin_summary_log;
    bool yaw_curve_debug = cfg.tof_pose_correction_enabled && cfg.tof_pose_correction_mode == "yaw_candidate" && cfg.tof_pose_correction_yaw_residual_curve_debug;
    if (cfg.map_quality_enabled) map_quality_log.open(run_dir + "/map_quality.csv");
    if (cfg.mapping_supervisor_enabled) supervisor_log.open(run_dir + "/supervisor_log.csv");
    if (cfg.active_scan_enabled) active_scan_log.open(run_dir + "/active_scan_log.csv");
    if (cfg.active_scan_execution_enabled && cfg.active_scan_execution_mode != "disabled") active_scan_command_log.open(run_dir + "/active_scan_command_log.csv");
    if (cfg.sparse_scan_enabled && cfg.sparse_scan_write_sample_log) sparse_scan_samples_log.open(run_dir + "/sparse_scan_samples.csv");
    if (cfg.sparse_scan_enabled && cfg.sparse_scan_write_bin_log) sparse_scan_bins_log.open(run_dir + "/sparse_scan_bins.csv");
    if (cfg.sparse_scan_enabled && cfg.sparse_scan_write_session_log) sparse_scan_sessions_log.open(run_dir + "/sparse_scan_sessions.csv");
    if (cfg.sparse_scan_yaw_match_enabled && cfg.sparse_scan_yaw_match_write_curve_log) yaw_match_curve_log.open(run_dir + "/yaw_match_curve.csv");
    if (cfg.sparse_scan_yaw_match_enabled && cfg.sparse_scan_yaw_match_write_summary_log) yaw_match_summary_log.open(run_dir + "/yaw_match_summary.csv");
    if (cfg.yaw_correction_enabled && cfg.yaw_correction_mode != "disabled") yaw_correction_gate_log.open(run_dir + "/yaw_correction_gate.csv");
    if (cfg.yaw_correction_enabled && cfg.yaw_correction_mode == "writeback" && cfg.yaw_correction_writeback_enabled && cfg.yaw_correction_apply_log_enabled) yaw_correction_apply_log.open(run_dir + "/yaw_correction_apply.csv");
    if (cfg.encoder_source == "cjc_bl4820_uart") encoderlog.open(run_dir + "/encoder_log.csv");
    if (cfg.tof_pose_correction_enabled) corrlog.open(run_dir + "/tof_correction_log.csv");
    if (cfg.tof_pose_correction_enabled && cfg.tof_pose_correction_mode == "yaw_candidate") yawlog.open(run_dir + "/candidate_yaw.csv");
    if (yaw_curve_debug) {
        yaw_curve_log.open(run_dir + "/yaw_residual_curve.csv");
        yaw_summary_log.open(run_dir + "/yaw_residual_summary.csv");
    }
    if (cfg.spin_scan_enabled) {
        spin_scan_log.open(run_dir + "/spin_scan.csv");
        spin_bins_log.open(run_dir + "/spin_scan_bins.csv");
        spin_curve_log.open(run_dir + "/spin_match_curve.csv");
        spin_summary_log.open(run_dir + "/spin_match_summary.csv");
    }
    traj << "timestamp_us,x_m,y_m,yaw_rad,v_mps,w_radps\n";
    loclog << "timestamp_us,left_ticks,right_ticks,left_delta,right_delta,dl_m,dr_m,ds_m,dyaw_enc_rad,imu_raw_gyro_z_rad_s,imu_used_gyro_z_rad_s,gyro_z_filtered_rad_s,gyro_bias_rad_s,yaw_enc_rad,yaw_imu_rad,yaw_fused_rad,pose_quality,localization_confidence,warn_flags\n";
    toflog << "timestamp_us,id,raw_range_mm,range_status,confidence,range_m,valid,filtered_range_m,filtered_confidence,free_delta,occ_delta,decision\n";
    if (map_quality_log) write_map_quality_header(map_quality_log);
    if (supervisor_log) write_supervisor_log_header(supervisor_log);
    if (active_scan_log) write_active_scan_log_header(active_scan_log);
    if (active_scan_command_log) write_active_scan_command_log_header(active_scan_command_log);
    if (sparse_scan_samples_log) write_sparse_scan_samples_header(sparse_scan_samples_log);
    if (sparse_scan_bins_log) write_sparse_scan_bins_header(sparse_scan_bins_log);
    if (sparse_scan_sessions_log) write_sparse_scan_sessions_header(sparse_scan_sessions_log);
    if (yaw_match_curve_log) write_yaw_match_curve_header(yaw_match_curve_log);
    if (yaw_match_summary_log) write_yaw_match_summary_header(yaw_match_summary_log);
    if (yaw_correction_gate_log) write_yaw_correction_gate_header(yaw_correction_gate_log);
    if (yaw_correction_apply_log) write_yaw_correction_apply_header(yaw_correction_apply_log);
    if (encoderlog) encoderlog << "timestamp_us,left_pos_raw,right_pos_raw,left_delta_ticks,right_delta_ticks,left_total_ticks,right_total_ticks,left_rpm,right_rpm,left_current_raw,right_current_raw,left_status,right_status,decision,reason\n";
    if (cfg.tof_pose_correction_enabled) {
        corrlog << "timestamp_us,mode,accepted,reason,localizer_x,localizer_y,localizer_yaw,odom_x,odom_y,odom_yaw,debug_offset_x_m,debug_offset_y_m,debug_offset_yaw_rad,odom_residual,improvement,best_x,best_y,best_yaw,best_dx,best_dy,best_dyaw,best_residual,second_best_residual,used_tof,candidate_count,odom_front_expected_m,odom_left_expected_m,odom_right_expected_m,best_front_expected_m,best_left_expected_m,best_right_expected_m\n";
    }
    if (cfg.tof_pose_correction_enabled && cfg.tof_pose_correction_mode == "yaw_candidate") {
        yawlog << "timestamp_us,odom_yaw,raw_candidate_yaw,filtered_candidate_yaw,raw_delta_yaw,filtered_delta_yaw,consistent_count,stable,best_residual,odom_residual,improvement,used_tof,reason,localizer_yaw,candidate_source_mode,best_yaw,second_yaw,third_yaw,second_best_residual,third_best_residual,best_second_margin,second_third_margin,best_second_yaw_gap_deg,best_third_yaw_gap_deg,candidate_reliability,candidate_usable,reject_for_flat_curve,reject_for_multimodal,reject_for_edge_best,reject_for_low_improvement,reject_for_low_margin,reject_for_insufficient_tof\n";
    }
    if (yaw_curve_debug) {
        yaw_curve_log << "timestamp_us,attempt_index,odom_x,odom_y,odom_yaw,candidate_yaw,yaw_offset_deg,total_residual,front_obs_m,front_expected_m,front_residual_m,left_obs_m,left_expected_m,left_residual_m,right_obs_m,right_expected_m,right_residual_m,used_tof,valid_candidate,reason\n";
        yaw_summary_log << "timestamp_us,attempt_index,best_yaw,second_yaw,third_yaw,best_residual,second_residual,third_residual,best_second_margin,best_second_yaw_gap_deg,curve_min_residual,curve_mean_residual,curve_p25_residual,curve_p50_residual,curve_p75_residual,curve_sharpness,best_at_search_edge,num_local_minima,multimodal,flat_curve,candidate_reliability,candidate_usable,reject_for_flat_curve,reject_for_multimodal,reject_for_edge_best,reject_for_low_improvement,reject_for_low_margin,reject_for_insufficient_tof\n";
    }
    if (cfg.spin_scan_enabled) {
        spin_scan_log << "timestamp_us,scan_id,state,base_x,base_y,base_yaw,linear_speed_mps,angular_speed_radps,accumulated_rotation_deg,sensor_id,global_bearing_deg,range_m,confidence,range_status,valid,reason\n";
        spin_bins_log << "scan_id,bin_index,bin_center_deg,selected_range_m,selected_confidence,sample_count,valid_count,sensor_mask,valid\n";
        spin_curve_log << "timestamp_us,scan_id,candidate_yaw,yaw_offset_deg,total_residual,valid_bins,front_like_bins,left_like_bins,right_like_bins,reason\n";
        spin_summary_log << "timestamp_us,scan_id,complete,matched,candidate_yaw,delta_yaw,best_residual,second_residual,third_residual,best_second_margin,best_second_yaw_gap_deg,curve_min_residual,curve_mean_residual,curve_p25_residual,curve_p50_residual,curve_p75_residual,curve_sharpness,flat_curve,multimodal,best_at_search_edge,num_local_minima,valid_bins,total_bins,valid_bin_ratio,reliability,usable,reason\n";
    }

    SensorManager sensors(cfg);
    if (!sensors.init(err)) { std::cerr << err << "\n"; return 1; }
    Localizer loc(cfg);
    TofFilter filter(cfg);
    ChunkedGrid grid(cfg);
    TofPoseCorrector tof_corrector(cfg);
    SpinScanLocalizer spin_scan(cfg);
    MapQuality map_quality(cfg);
    MappingSupervisor supervisor(cfg);
    ActiveScanManager active_scan(cfg);
    ActiveScanCommandPlanner active_scan_command(cfg);
    SparseScanCollector sparse_scan(cfg);
    SparseScanYawMatcher sparse_scan_yaw_match(cfg);
    YawCorrectionGate yaw_correction_gate(cfg);
    YawCorrectionApply yaw_correction_apply(cfg);
    MapQualitySnapshot latest_map_quality_snapshot;
    MappingSupervisorSnapshot latest_supervisor_snapshot;
    ActiveScanSnapshot latest_active_scan_snapshot;
    ActiveScanCommandSnapshot latest_active_scan_command_snapshot;
    SparseScanYawMatchSummary latest_yaw_match_summary;
    bool map_alloc_fail_pending = false;
    RunMetrics metrics;
    TofHealth tof_health;
    double odom_quality = 1.0;
    double latest_pose_quality = 1.0;
    double latest_localization_confidence = 1.0;
    bool yaw_candidate_have_filter = false;
    double yaw_candidate_filtered_delta = 0.0;
    int yaw_candidate_consistent_count = 0;
    bool yaw_candidate_stable = false;
    double latest_correction_improvement = 0.0;
    uint64_t correction_attempt_index = 0;
    uint64_t supervisor_imu_gap_count = 0;
    uint64_t supervisor_imu_read_fail_count = 0;

    auto compute_pose_quality = [&](const std::vector<std::string> &flags) {
        double q = odom_quality;
        for (const auto &id : {"front", "left", "right"}) {
            if (tof_health.unhealthy(id, cfg)) q -= 0.1;
        }
        if (latest_correction_improvement < -0.01) q -= 0.05;
        if (cfg.tof_pose_correction_mode == "yaw_candidate" && cfg.tof_pose_correction_enabled && !yaw_candidate_stable) q -= 0.02;
        for (const auto &f : flags) {
            if (f.find("tof_") == 0 && f.find("unhealthy") != std::string::npos) q -= 0.1;
        }
        return clamp01(q);
    };

    uint64_t start = now_us_steady();
    double effective_localization_hz = cfg.encoder_source == "cjc_bl4820_uart" ? std::max(cfg.localization_hz, cfg.encoder_read_hz) : cfg.localization_hz;
    uint64_t last_loc = start, last_tof = start, last_map = start, last_log = start, last_correction = start;
    bool have_last_encoder_sample_time = false;
    uint64_t last_encoder_sample_time = 0;
    std::vector<TofSample> latest_tof;
    double gyro_bias = 0.0;
    int gyro_count = 0;
    double gyro_max_abs = 0.0;
    bool calibrated = cfg.gyro_bias_static_calib_s <= 0.0;
    std::cout << "robot_slamd run_dir=" << run_dir << "\n";
    std::cout << "WARN calibration_required: wheel_base/radius/ticks_per_rev are defaults unless calibrated\n";

    auto make_supervisor_sensor_input = [&]() {
        SupervisorSensorInput in;
        const auto &es = sensors.encoder_stats();
        in.localization_updates = metrics.localization_updates;
        in.encoder_left_errors = es.left_errors;
        in.encoder_right_errors = es.right_errors;
        in.encoder_jump_rejects = es.jump_rejects;
        in.encoder_timeout_errors = es.uart_timeout_errors;
        in.encoder_rejected_updates = loc.rejected_encoder_updates();
        in.imu_gap_count = supervisor_imu_gap_count;
        in.imu_read_fail_count = supervisor_imu_read_fail_count;
        in.gyro_spike_count = loc.gyro_spikes();
        in.map_alloc_failures = metrics.map_alloc_failures;
        int unhealthy = 0;
        for (const auto &id : {"front", "left", "right"}) {
            if (tof_health.unhealthy(id, cfg)) unhealthy++;
        }
        in.unhealthy_tof_routes = unhealthy;
        return in;
    };

    auto update_supervisor_from_quality = [&](double now_s, const MapQualitySnapshot &qs, bool force_log) {
        if (!cfg.mapping_supervisor_enabled && !cfg.active_scan_enabled && !cfg.active_scan_execution_enabled && !cfg.sparse_scan_enabled && !cfg.sparse_scan_yaw_match_enabled && !cfg.yaw_correction_enabled) return;
        SupervisorSensorInput sin = make_supervisor_sensor_input();
        bool changed = supervisor.update(now_s, qs, sin);
        latest_supervisor_snapshot = supervisor.snapshot();
        if (cfg.mapping_supervisor_enabled && supervisor_log && (force_log || changed || supervisor.should_log(now_s))) {
            write_supervisor_log_row(supervisor_log, latest_supervisor_snapshot);
            supervisor.mark_logged(now_s, latest_supervisor_snapshot);
        }
    };

    auto update_active_scan_from_snapshots = [&](double now_s, const MapQualitySnapshot &qs, const MappingSupervisorSnapshot &ss, bool force_log) {
        if (!cfg.active_scan_enabled && !cfg.active_scan_execution_enabled && !cfg.sparse_scan_enabled && !cfg.sparse_scan_yaw_match_enabled && !cfg.yaw_correction_enabled) return;
        const auto &p = loc.pose();
        ActiveScanInput in;
        in.timestamp_s = now_s;
        in.map_quality = qs;
        in.supervisor = ss;
        in.pose_x_m = p.x;
        in.pose_y_m = p.y;
        in.yaw_rad = p.yaw;
        in.linear_speed_mps = loc.v();
        in.yaw_rate_radps = loc.w();
        in.localization_valid = true;
        bool changed = active_scan.update(in);
        latest_active_scan_snapshot = active_scan.snapshot();
        if (cfg.active_scan_enabled && active_scan_log && (force_log || changed || active_scan.should_log(now_s))) {
            write_active_scan_log_row(active_scan_log, latest_active_scan_snapshot);
            active_scan.mark_logged(now_s, latest_active_scan_snapshot);
        }
    };

    auto update_active_scan_command_from_snapshots = [&](double now_s, const MapQualitySnapshot &qs, const MappingSupervisorSnapshot &ss, const ActiveScanSnapshot &as, bool force_log) {
        if (!cfg.active_scan_execution_enabled || cfg.active_scan_execution_mode == "disabled") return;
        const auto &p = loc.pose();
        ActiveScanCommandInput in;
        in.timestamp_s = now_s;
        in.active_scan = as;
        in.supervisor = ss;
        in.map_quality = qs;
        in.pose_x_m = p.x;
        in.pose_y_m = p.y;
        in.yaw_rad = p.yaw;
        in.linear_speed_mps = loc.v();
        in.yaw_rate_radps = loc.w();
        in.localization_valid = true;
        bool changed = active_scan_command.update(in);
        ActiveScanCommandSnapshot cs = active_scan_command.snapshot();
        latest_active_scan_command_snapshot = cs;
        if (active_scan_command_log && (force_log || changed || active_scan_command.should_log(now_s))) {
            write_active_scan_command_log_row(active_scan_command_log, cs);
            active_scan_command.mark_logged(now_s, cs);
        }
    };

    auto drain_sparse_scan_logs = [&]() {
        std::string state = sparse_scan_state_name(sparse_scan.state());
        if (sparse_scan_samples_log) {
            for (const auto &sample : sparse_scan.drain_samples()) write_sparse_scan_sample_row(sparse_scan_samples_log, sample, state);
        } else {
            sparse_scan.drain_samples();
        }
        if (sparse_scan_bins_log) {
            for (const auto &bin : sparse_scan.drain_bins()) write_sparse_scan_bin_row(sparse_scan_bins_log, bin);
        } else {
            sparse_scan.drain_bins();
        }
        if (sparse_scan_sessions_log) {
            for (const auto &summary : sparse_scan.drain_summaries()) write_sparse_scan_session_row(sparse_scan_sessions_log, summary);
        } else {
            sparse_scan.drain_summaries();
        }
    };

    auto update_yaw_correction_apply_from_gate = [&](double now_s, const YawCorrectionGateSnapshot &gate_snap) {
        if (!cfg.yaw_correction_enabled || cfg.yaw_correction_mode != "writeback" || !cfg.yaw_correction_writeback_enabled) return;
        const auto &p = loc.pose();
        YawCorrectionApplySnapshot apply_snap = yaw_correction_apply.update(now_s, gate_snap, loc, p.yaw, loc.v(), loc.w(), latest_supervisor_snapshot, true);
        if (yaw_correction_apply_log && apply_snap.attempted) write_yaw_correction_apply_row(yaw_correction_apply_log, apply_snap);
    };

    auto update_yaw_correction_gate_from_summary = [&](double now_s, const SparseScanYawMatchSummary &ys, bool force_log) {
        if (!cfg.yaw_correction_enabled || cfg.yaw_correction_mode == "disabled") return;
        const auto &p = loc.pose();
        YawCorrectionGateInput in;
        in.timestamp_s = now_s;
        in.yaw_match = ys;
        in.map_quality = latest_map_quality_snapshot;
        in.supervisor = latest_supervisor_snapshot;
        in.active_scan = latest_active_scan_snapshot;
        in.active_scan_command = latest_active_scan_command_snapshot;
        in.current_yaw_rad = p.yaw;
        in.linear_speed_mps = loc.v();
        in.yaw_rate_radps = loc.w();
        in.localization_valid = true;
        bool changed = yaw_correction_gate.update(in);
        YawCorrectionGateSnapshot snap = yaw_correction_gate.snapshot();
        if (yaw_correction_gate_log && (force_log || changed || yaw_correction_gate.should_log(now_s))) {
            write_yaw_correction_gate_row(yaw_correction_gate_log, snap);
            yaw_correction_gate.mark_logged(now_s, snap);
        }
        update_yaw_correction_apply_from_gate(now_s, snap);
    };

    auto drain_yaw_match_logs = [&](double now_s, bool force_gate_log) {
        if (yaw_match_curve_log) {
            for (const auto &row : sparse_scan_yaw_match.drain_curve_rows()) write_yaw_match_curve_row(yaw_match_curve_log, row);
        } else {
            sparse_scan_yaw_match.drain_curve_rows();
        }
        std::vector<SparseScanYawMatchSummary> summaries = sparse_scan_yaw_match.drain_summaries();
        for (const auto &summary : summaries) {
            if (yaw_match_summary_log) write_yaw_match_summary_row(yaw_match_summary_log, summary);
            latest_yaw_match_summary = summary;
            update_yaw_correction_gate_from_summary(now_s, summary, force_gate_log);
        }
    };

    auto update_yaw_match_from_completed_scans = [&](double now_s, const std::vector<SparseScanForMatching> &completed_scans) {
        if (!cfg.sparse_scan_yaw_match_enabled) return;
        if (!completed_scans.empty()) {
            sparse_scan_yaw_match.update(now_s, completed_scans, grid, latest_map_quality_snapshot, latest_supervisor_snapshot);
            if (sparse_scan_yaw_match.should_log(now_s)) sparse_scan_yaw_match.mark_logged(now_s);
        }
        drain_yaw_match_logs(now_s, false);
    };

    auto update_sparse_scan_from_snapshots = [&](double now_s, uint64_t now_us, const std::vector<FilteredTofSample> &spin_tof) {
        if (!cfg.sparse_scan_enabled) return;
        SparseScanInput in;
        in.timestamp_s = now_s;
        in.timestamp_us = now_us;
        in.pose = loc.pose();
        in.linear_speed_mps = loc.v();
        in.yaw_rate_radps = loc.w();
        in.localization_valid = true;
        in.map_quality = latest_map_quality_snapshot;
        in.supervisor = latest_supervisor_snapshot;
        in.active_scan = latest_active_scan_snapshot;
        in.active_scan_command = latest_active_scan_command_snapshot;
        in.tof_samples = spin_tof;
        sparse_scan.update(in);
        drain_sparse_scan_logs();
        update_yaw_match_from_completed_scans(now_s, sparse_scan.drain_completed_sessions_for_matching());
        sparse_scan.mark_logged(now_s);
    };

    while (!g_stop) {
        uint64_t now = now_us_steady();
        double sim_t = (now - start) / 1000000.0;
        if (duration_s > 0.0 && sim_t >= duration_s) break;

        if (!calibrated) {
            ImuSample imu = sensors.read_imu(now, sim_t);
            gyro_bias += imu.gyro_z_rad_s;
            gyro_max_abs = std::max(gyro_max_abs, std::fabs(imu.gyro_z_rad_s));
            gyro_count++;
            if (sim_t >= cfg.gyro_bias_static_calib_s) {
                gyro_bias = gyro_count ? gyro_bias / gyro_count : 0.0;
                if (gyro_max_abs > cfg.static_gyro_max_abs_rad_s) {
                    std::cerr << "WARN imu_calib_failed, using bias=0\n";
                    gyro_bias = 0.0;
                }
                loc.set_gyro_bias(gyro_bias);
                calibrated = true;
            }
            usleep(1000);
            continue;
        }

        if ((now - last_loc) >= static_cast<uint64_t>(1000000.0 / effective_localization_hz)) {
            double dt = (now - last_loc) / 1000000.0;
            last_loc = now;
            EncoderSample enc = sensors.read_encoder(now, sim_t);
            if (encoderlog) {
                const auto &elog = sensors.last_encoder_log();
                encoderlog << elog.t_us << "," << elog.left_pos_raw << "," << elog.right_pos_raw << ","
                           << elog.left_delta_ticks << "," << elog.right_delta_ticks << ","
                           << elog.left_total_ticks << "," << elog.right_total_ticks << ","
                           << elog.left_rpm << "," << elog.right_rpm << ","
                           << elog.left_current_raw << "," << elog.right_current_raw << ","
                           << static_cast<int>(elog.left_status) << "," << static_cast<int>(elog.right_status) << ","
                           << elog.decision << "," << elog.reason << "\n";
            }
            if (cfg.encoder_source == "csv") {
                if (have_last_encoder_sample_time && enc.t_us > last_encoder_sample_time) {
                    dt = static_cast<double>(enc.t_us - last_encoder_sample_time) / 1000000.0;
                }
                last_encoder_sample_time = enc.t_us;
                have_last_encoder_sample_time = true;
            }
            ImuSample imu = sensors.read_imu(now, sim_t);
            loc.update(enc, imu, dt);
            metrics.localization_updates++;
            std::vector<std::string> sensor_warnings = sensors.consume_warnings();
            for (const auto &sw : sensor_warnings) {
                if (sw == "imu_gap" || sw == "icm43600_no_data") supervisor_imu_gap_count++;
                else if (sw == "imu_read_fail" || sw == "icm43600_error") supervisor_imu_read_fail_count++;
            }
            std::vector<std::string> extra_warnings = merge_warnings(sensor_warnings, loc.consume_warnings());
            std::vector<std::string> flags = warning_list(cfg, loc.v(), loc.w(), extra_warnings, map_alloc_fail_pending);
            odom_quality = odom_quality_from_flags(flags);
            latest_pose_quality = compute_pose_quality(flags);
            latest_localization_confidence = latest_pose_quality;
            metrics.pose_quality_sum += latest_pose_quality;
            metrics.pose_quality_min = std::min(metrics.pose_quality_min, latest_pose_quality);
            metrics.localization_confidence_sum += latest_localization_confidence;
            metrics.localization_confidence_min = std::min(metrics.localization_confidence_min, latest_localization_confidence);
            std::string wf = warn_flags(flags);
            map_alloc_fail_pending = false;
            loclog << now << "," << enc.left_ticks << "," << enc.right_ticks << ","
                   << loc.left_delta_ticks() << "," << loc.right_delta_ticks() << ","
                   << loc.dl() << "," << loc.dr() << "," << loc.ds() << "," << loc.dyaw_enc() << ","
                   << imu.raw_gyro_z_rad_s << "," << imu.gyro_z_rad_s << ","
                   << loc.gyro_filtered() << "," << loc.gyro_bias() << "," << loc.yaw_enc() << ","
                   << loc.yaw_imu() << "," << loc.pose().yaw << "," << latest_pose_quality << ","
                   << latest_localization_confidence << "," << wf << "\n";
        }
        if ((now - last_tof) >= static_cast<uint64_t>(1000000.0 / cfg.tof_read_hz)) {
            last_tof = now;
            latest_tof = sensors.read_tof(now, loc.pose());
            for (const auto &ev : sensors.consume_tof_events()) {
                toflog << ev.t_us << "," << ev.id << ",-1,-1,0,0,0,0,0,0,0," << ev.decision << "\n";
            }
        }
        if ((now - last_map) >= static_cast<uint64_t>(1000000.0 / cfg.mapping_hz)) {
            last_map = now;
            bool pause = std::fabs(loc.v()) > cfg.pause_linear_speed_mps || std::fabs(loc.w()) > cfg.pause_angular_speed_radps;
            std::map<std::string, bool> seen_tof{{"front", false}, {"left", false}, {"right", false}};
            std::vector<TofObsForCorrection> correction_obs;
            std::vector<FilteredTofSample> spin_tof;
            for (const auto &t : latest_tof) {
                seen_tof[t.id] = true;
                TofFilterResult res = filter.process(t);
                double map_quality_now_s = (now - start) / 1000000.0;
                if (cfg.map_quality_enabled || cfg.mapping_supervisor_enabled || cfg.active_scan_enabled || cfg.active_scan_execution_enabled || cfg.sparse_scan_enabled || cfg.sparse_scan_yaw_match_enabled || cfg.yaw_correction_enabled) map_quality.record_tof_result(map_quality_now_s, t, res);
                spin_tof.push_back({t.t_us, t.id, res.range_m, res.filtered_confidence, t.range_status, res.map_update, res.hit, res.decision});
                tof_health.record_sample(t.id, res);
                metrics.tof_samples++;
                double raw_m = t.raw_range_mm / 1000.0;
                bool valid = res.map_update && res.hit;
                bool free_only = res.map_update && !res.hit;
                if (valid) metrics.tof_hit_updates++;
                else if (free_only) metrics.tof_free_only++;
                else metrics.tof_rejected++;

                bool low_odom_quality = odom_quality < 0.5;
                bool route_unhealthy = tof_health.unhealthy(t.id, cfg);
                if (cfg.tof_pose_correction_enabled && res.map_update && !route_unhealthy) {
                    correction_obs.push_back({t.id, res.range_m, res.hit, !res.hit, res.filtered_confidence});
                }
                bool static_boost = loc.stationary() && res.stability_count >= cfg.static_scan_stable_required;
                bool should_update_map = !pause && res.map_update && !low_odom_quality && !route_unhealthy;
                std::string log_decision = static_boost ? res.decision + "_static_boost" : res.decision;
                if (res.map_update && low_odom_quality) {
                    log_decision = "mapping_skipped_low_odom_quality";
                    metrics.low_odom_quality_pauses++;
                } else if (res.map_update && route_unhealthy) {
                    log_decision = std::string("tof_") + t.id + "_unhealthy";
                    metrics.tof_unhealthy_pauses++;
                }

                double free_delta = 0.0;
                double occ_delta = 0.0;
                if (should_update_map) {
                    double weight = res.hit ? confidence_weight(res.filtered_confidence) : 1.0;
                    double boost = static_boost ? cfg.static_scan_boost : 1.0;
                    free_delta = cfg.log_odds_free * weight * boost;
                    occ_delta = res.hit ? cfg.log_odds_occ * weight * boost : 0.0;
                }
                toflog << t.t_us << "," << t.id << "," << t.raw_range_mm << "," << t.range_status << ","
                       << t.confidence << "," << raw_m << "," << (valid ? 1 : 0) << "," << res.range_m << ","
                       << res.filtered_confidence << "," << free_delta << "," << occ_delta << "," << log_decision << "\n";
                bool map_updated = false;
                if (should_update_map) {
                    try {
                        grid.update_tof_ray(loc.pose(), cfg.tof_extrinsics.at(t.id), res.range_m, res.hit, free_delta, occ_delta, cfg);
                        map_updated = true;
                        metrics.map_updates++;
                        if (static_boost) metrics.static_scan_boost_updates++;
                    } catch (const std::bad_alloc &) {
                        map_alloc_fail_pending = true;
                        metrics.map_alloc_failures++;
                    }
                }
                if (cfg.map_quality_enabled || cfg.mapping_supervisor_enabled || cfg.active_scan_enabled || cfg.active_scan_execution_enabled || cfg.sparse_scan_enabled || cfg.sparse_scan_yaw_match_enabled || cfg.yaw_correction_enabled) {
                    map_quality.record_map_update(map_quality_now_s, res.map_update, map_updated, res.hit, free_only);
                }
            }
            for (const auto &id : {"front", "left", "right"}) {
                if (!seen_tof[id]) tof_health.record_gap(id);
            }
            if (cfg.spin_scan_enabled) {
                spin_scan.update(now, loc.pose(), spin_tof, grid, cfg);
                for (const auto &s : spin_scan.drain_samples()) {
                    spin_scan_log << s.timestamp_us << "," << s.scan_id << "," << s.state << ","
                                  << s.base_x << "," << s.base_y << "," << s.base_yaw << ","
                                  << s.linear_speed_mps << "," << s.angular_speed_radps << "," << s.accumulated_rotation_deg << ","
                                  << s.sensor_id << "," << (s.global_bearing * 180.0 / kPi) << "," << s.range_m << ","
                                  << s.confidence << "," << s.range_status << "," << (s.valid ? 1 : 0) << "," << s.reason << "\n";
                }
                for (const auto &b : spin_scan.drain_bins()) {
                    spin_bins_log << b.scan_id << "," << b.bin_index << "," << b.bin_center_deg << ","
                                  << b.selected_range_m << "," << b.selected_confidence << "," << b.sample_count << ","
                                  << b.valid_count << "," << b.sensor_mask << "," << (b.valid ? 1 : 0) << "\n";
                }
                for (const auto &c : spin_scan.drain_curve()) {
                    spin_curve_log << c.timestamp_us << "," << c.scan_id << "," << c.candidate_yaw << ","
                                   << c.yaw_offset_deg << "," << c.total_residual << "," << c.valid_bins << ","
                                   << c.front_like_bins << "," << c.left_like_bins << "," << c.right_like_bins << "," << c.reason << "\n";
                }
                for (const auto &r : spin_scan.drain_summary()) {
                    spin_summary_log << r.timestamp_us << "," << r.scan_id << "," << (r.complete ? 1 : 0) << "," << (r.matched ? 1 : 0) << ","
                                     << r.candidate_yaw << "," << r.delta_yaw << "," << r.best_residual << ","
                                     << r.second_residual << "," << r.third_residual << "," << r.best_second_margin << ","
                                     << r.best_second_yaw_gap_deg << "," << r.curve_min_residual << "," << r.curve_mean_residual << ","
                                     << r.curve_p25_residual << "," << r.curve_p50_residual << "," << r.curve_p75_residual << ","
                                     << r.curve_sharpness << "," << (r.flat_curve ? 1 : 0) << "," << (r.multimodal ? 1 : 0) << ","
                                     << (r.best_at_search_edge ? 1 : 0) << "," << r.num_local_minima << "," << r.valid_bins << ","
                                     << r.total_bins << "," << r.valid_bin_ratio << "," << r.reliability << "," << (r.usable ? 1 : 0) << "," << r.reason << "\n";
                    metrics.spin_scan_attempts++;
                    if (r.complete) metrics.spin_scan_completed++; else metrics.spin_scan_failed++;
                    if (r.matched) metrics.spin_scan_matched++;
                    if (r.usable) metrics.spin_scan_usable++;
                    metrics.spin_scan_valid_bins_sum += r.valid_bins;
                    metrics.spin_scan_valid_bin_ratio_sum += r.valid_bin_ratio;
                    metrics.spin_scan_reliability_sum += r.reliability;
                    metrics.spin_scan_reliability_min = std::min(metrics.spin_scan_reliability_min, r.reliability);
                    if (r.multimodal) metrics.spin_scan_multimodal_count++;
                    if (r.flat_curve) metrics.spin_scan_flat_curve_count++;
                    if (r.best_at_search_edge) metrics.spin_scan_edge_best_count++;
                }
            }
            update_sparse_scan_from_snapshots((now - start) / 1000000.0, now, spin_tof);
            if (cfg.tof_pose_correction_enabled && (now - last_correction) >= static_cast<uint64_t>(1000000.0 / cfg.tof_pose_correction_update_hz)) {
                last_correction = now;
                Pose correction_pose = loc.pose();
                correction_pose.x += cfg.tof_pose_correction_debug_offset_x_m;
                correction_pose.y += cfg.tof_pose_correction_debug_offset_y_m;
                correction_pose.yaw = wrap_pi(correction_pose.yaw + deg2rad(cfg.tof_pose_correction_debug_offset_yaw_deg));
                PoseCorrectionResult cres;
                if (odom_quality < 0.5) {
                    cres.corrected_pose = correction_pose;
                    cres.reason = "bad_odom";
                } else {
                    cres = tof_corrector.correct(correction_pose, correction_obs, grid, cfg);
                }
                cres.accepted = false;
                double improvement = cres.odom_residual - cres.best_residual;
                latest_correction_improvement = improvement;
                double best_shift = std::hypot(cres.dx, cres.dy);
                metrics.tof_correction_attempts++;
                correction_attempt_index++;
                if (cres.accepted) metrics.tof_correction_accepts++;
                else metrics.tof_correction_rejects++;
                metrics.tof_correction_residual_sum += cres.best_residual;
                metrics.tof_correction_odom_residual_sum += cres.odom_residual;
                metrics.tof_correction_improvement_sum += improvement;
                metrics.tof_correction_best_shift_sum += best_shift;
                metrics.tof_correction_max_dx = std::max(metrics.tof_correction_max_dx, best_shift);
                metrics.tof_correction_max_dyaw = std::max(metrics.tof_correction_max_dyaw, std::fabs(cres.dyaw));
                const auto &p = loc.pose();
                double candidate_reliability = 0.0;
                bool candidate_usable = false;
                bool reject_for_flat_curve = false;
                bool reject_for_multimodal = false;
                bool reject_for_edge_best = false;
                bool reject_for_low_improvement = false;
                bool reject_for_low_margin = false;
                bool reject_for_insufficient_tof = false;
                if (cfg.tof_pose_correction_mode == "yaw_candidate") {
                    double raw_candidate_yaw = cres.corrected_pose.yaw;
                    double raw_delta_yaw = wrap_pi(raw_candidate_yaw - correction_pose.yaw);
                    if (!yaw_candidate_have_filter) {
                        yaw_candidate_filtered_delta = raw_delta_yaw;
                        yaw_candidate_have_filter = true;
                    } else {
                        yaw_candidate_filtered_delta = wrap_pi(yaw_candidate_filtered_delta + cfg.tof_pose_correction_candidate_lpf_alpha * wrap_pi(raw_delta_yaw - yaw_candidate_filtered_delta));
                    }
                    double filtered_candidate_yaw = wrap_pi(correction_pose.yaw + yaw_candidate_filtered_delta);
                    bool valid_candidate = cres.used_tof >= cfg.tof_pose_correction_min_valid_tof &&
                                           improvement > 0.0 &&
                                           cres.best_residual < cres.odom_residual &&
                                           cres.reason != "bad_odom" &&
                                           cres.reason != "insufficient_tof" &&
                                           cres.reason != "no_map";
                    bool consistent = valid_candidate && std::fabs(wrap_pi(raw_delta_yaw - yaw_candidate_filtered_delta)) <= deg2rad(cfg.tof_pose_correction_consistency_yaw_deg);
                    if (consistent) yaw_candidate_consistent_count++;
                    else yaw_candidate_consistent_count = 0;
                    yaw_candidate_stable = yaw_candidate_consistent_count >= cfg.tof_pose_correction_required_consistency;
                    reject_for_insufficient_tof = cres.used_tof < cfg.tof_pose_correction_min_valid_tof;
                    reject_for_low_improvement = improvement <= 0.0;
                    reject_for_flat_curve = cres.flat_curve;
                    reject_for_multimodal = cres.multimodal;
                    reject_for_edge_best = cres.best_at_search_edge;
                    reject_for_low_margin = cres.best_second_margin < cfg.tof_pose_correction_min_best_second_margin;
                    if (reject_for_insufficient_tof || reject_for_low_improvement) {
                        candidate_reliability = 0.0;
                    } else {
                        candidate_reliability = 1.0;
                        if (reject_for_flat_curve) candidate_reliability -= 0.4;
                        if (reject_for_multimodal) candidate_reliability -= 0.4;
                        if (reject_for_edge_best) candidate_reliability -= 0.3;
                        if (reject_for_low_margin) candidate_reliability -= 0.3;
                        if (!yaw_candidate_stable) candidate_reliability -= 0.2;
                        candidate_reliability = std::max(0.0, std::min(1.0, candidate_reliability));
                    }
                    candidate_usable = !reject_for_insufficient_tof &&
                                       !reject_for_low_improvement &&
                                       yaw_candidate_stable &&
                                       !reject_for_flat_curve &&
                                       !reject_for_multimodal &&
                                       !reject_for_edge_best &&
                                       !reject_for_low_margin &&
                                       candidate_reliability >= cfg.tof_pose_correction_min_candidate_reliability;
                    yawlog << now << "," << correction_pose.yaw << "," << raw_candidate_yaw << "," << filtered_candidate_yaw << ","
                           << raw_delta_yaw << "," << yaw_candidate_filtered_delta << "," << yaw_candidate_consistent_count << ","
                           << (yaw_candidate_stable ? 1 : 0) << "," << cres.best_residual << "," << cres.odom_residual << ","
                           << improvement << "," << cres.used_tof << "," << cres.reason << "," << p.yaw << "," << cfg.tof_pose_correction_mode << ","
                           << cres.best_yaw << "," << cres.second_yaw << "," << cres.third_yaw << ","
                           << cres.second_residual << "," << cres.third_residual << ","
                           << cres.best_second_margin << "," << cres.second_third_margin << ","
                           << cres.best_second_yaw_gap_deg << "," << cres.best_third_yaw_gap_deg << ","
                           << candidate_reliability << "," << (candidate_usable ? 1 : 0) << ","
                           << (reject_for_flat_curve ? 1 : 0) << "," << (reject_for_multimodal ? 1 : 0) << ","
                           << (reject_for_edge_best ? 1 : 0) << "," << (reject_for_low_improvement ? 1 : 0) << ","
                           << (reject_for_low_margin ? 1 : 0) << "," << (reject_for_insufficient_tof ? 1 : 0) << "\n";
                }
                if (yaw_curve_debug) {
                    for (const auto &point : cres.yaw_curve) {
                        yaw_curve_log << now << "," << correction_attempt_index << ","
                                      << correction_pose.x << "," << correction_pose.y << "," << correction_pose.yaw << ","
                                      << point.candidate_yaw << "," << point.yaw_offset_deg << "," << point.total_residual << ","
                                      << point.front_obs_m << "," << point.front_expected_m << "," << point.front_residual_m << ","
                                      << point.left_obs_m << "," << point.left_expected_m << "," << point.left_residual_m << ","
                                      << point.right_obs_m << "," << point.right_expected_m << "," << point.right_residual_m << ","
                                      << point.used_tof << "," << (point.valid_candidate ? 1 : 0) << "," << point.reason << "\n";
                    }
                    yaw_summary_log << now << "," << correction_attempt_index << ","
                                    << cres.best_yaw << "," << cres.second_yaw << "," << cres.third_yaw << ","
                                    << cres.best_residual << "," << cres.second_residual << "," << cres.third_residual << ","
                                    << cres.best_second_margin << "," << cres.best_second_yaw_gap_deg << ","
                                    << cres.curve_min_residual << "," << cres.curve_mean_residual << ","
                                    << cres.curve_p25_residual << "," << cres.curve_p50_residual << "," << cres.curve_p75_residual << ","
                                    << cres.curve_sharpness << "," << (cres.best_at_search_edge ? 1 : 0) << ","
                                    << cres.num_local_minima << "," << (cres.multimodal ? 1 : 0) << "," << (cres.flat_curve ? 1 : 0) << ","
                                    << candidate_reliability << "," << (candidate_usable ? 1 : 0) << ","
                                    << (reject_for_flat_curve ? 1 : 0) << "," << (reject_for_multimodal ? 1 : 0) << ","
                                    << (reject_for_edge_best ? 1 : 0) << "," << (reject_for_low_improvement ? 1 : 0) << ","
                                    << (reject_for_low_margin ? 1 : 0) << "," << (reject_for_insufficient_tof ? 1 : 0) << "\n";
                }
                corrlog << now << "," << cfg.tof_pose_correction_mode << "," << (cres.accepted ? 1 : 0) << "," << cres.reason << ","
                        << p.x << "," << p.y << "," << p.yaw << ","
                        << correction_pose.x << "," << correction_pose.y << "," << correction_pose.yaw << ","
                        << cfg.tof_pose_correction_debug_offset_x_m << "," << cfg.tof_pose_correction_debug_offset_y_m << "," << deg2rad(cfg.tof_pose_correction_debug_offset_yaw_deg) << ","
                        << cres.odom_residual << "," << improvement << ","
                        << cres.corrected_pose.x << "," << cres.corrected_pose.y << "," << cres.corrected_pose.yaw << ","
                        << cres.dx << "," << cres.dy << "," << cres.dyaw << ","
                        << cres.best_residual << "," << cres.second_residual << ","
                        << cres.used_tof << "," << cres.candidate_count << ","
                        << cres.odom_front_expected_m << "," << cres.odom_left_expected_m << "," << cres.odom_right_expected_m << ","
                        << cres.best_front_expected_m << "," << cres.best_left_expected_m << "," << cres.best_right_expected_m << "\n";
            }
        }
        if (cfg.map_quality_enabled || cfg.mapping_supervisor_enabled || cfg.active_scan_enabled || cfg.active_scan_execution_enabled || cfg.sparse_scan_enabled || cfg.sparse_scan_yaw_match_enabled || cfg.yaw_correction_enabled) {
            double quality_now_s = (now - start) / 1000000.0;
            bool log_quality = cfg.map_quality_enabled && map_quality.should_log(quality_now_s);
            bool supervise_due = cfg.mapping_supervisor_enabled && supervisor.should_log(quality_now_s);
            bool active_scan_due = (cfg.active_scan_enabled || cfg.active_scan_execution_enabled || cfg.sparse_scan_enabled || cfg.sparse_scan_yaw_match_enabled || cfg.yaw_correction_enabled) && active_scan.should_log(quality_now_s);
            bool command_due = cfg.active_scan_execution_enabled && active_scan_command.should_log(quality_now_s);
            bool sparse_due = cfg.sparse_scan_enabled && sparse_scan.should_log(quality_now_s);
            bool yaw_correction_due = cfg.yaw_correction_enabled && yaw_correction_gate.should_log(quality_now_s);
            if (log_quality || supervise_due || active_scan_due || command_due || sparse_due || yaw_correction_due) {
                GridStats gst = grid.stats(cfg);
                gst.map_alloc_failures = metrics.map_alloc_failures;
                map_quality.update_grid_stats(gst);
                MapQualitySnapshot qs = map_quality.snapshot(quality_now_s);
                latest_map_quality_snapshot = qs;
                if (log_quality) {
                    if (map_quality_log) write_map_quality_row(map_quality_log, qs);
                    map_quality.mark_logged(quality_now_s, qs);
                }
                update_supervisor_from_quality(quality_now_s, qs, false);
                update_active_scan_from_snapshots(quality_now_s, qs, latest_supervisor_snapshot, false);
                update_active_scan_command_from_snapshots(quality_now_s, qs, latest_supervisor_snapshot, latest_active_scan_snapshot, false);
                if (yaw_correction_due) update_yaw_correction_gate_from_summary(quality_now_s, latest_yaw_match_summary, false);
            }
        }
        if ((now - last_log) >= static_cast<uint64_t>(1000000.0 / cfg.log_hz)) {
            last_log = now;
            const auto &p = loc.pose();
            traj << now << "," << p.x << "," << p.y << "," << p.yaw << "," << loc.v() << "," << loc.w() << "\n";
        }
        if (file_exists(run_dir + "/save_map.trigger")) {
            grid.save(run_dir + "/map.pgm", run_dir + "/map.yaml", cfg);
            unlink((run_dir + "/save_map.trigger").c_str());
        }
        usleep(1000);
    }
    if (cfg.save_on_exit) grid.save(run_dir + "/map.pgm", run_dir + "/map.yaml", cfg);
    if (cfg.map_quality_enabled || cfg.mapping_supervisor_enabled || cfg.active_scan_enabled || cfg.active_scan_execution_enabled || cfg.sparse_scan_enabled || cfg.sparse_scan_yaw_match_enabled || cfg.yaw_correction_enabled) {
        double quality_now_s = (now_us_steady() - start) / 1000000.0;
        GridStats gst = grid.stats(cfg);
        gst.map_alloc_failures = metrics.map_alloc_failures;
        map_quality.update_grid_stats(gst);
        MapQualitySnapshot qs = map_quality.snapshot(quality_now_s);
        latest_map_quality_snapshot = qs;
        if (cfg.map_quality_enabled) {
            if (map_quality_log) write_map_quality_row(map_quality_log, qs);
            map_quality.mark_logged(quality_now_s, qs);
            metrics.map_quality = map_quality.run_stats();
        }
        if (cfg.mapping_supervisor_enabled || cfg.active_scan_enabled || cfg.active_scan_execution_enabled || cfg.sparse_scan_enabled || cfg.sparse_scan_yaw_match_enabled || cfg.yaw_correction_enabled) {
            update_supervisor_from_quality(quality_now_s, qs, true);
            if (cfg.mapping_supervisor_enabled) metrics.mapping_supervisor = supervisor.run_stats(quality_now_s);
        }
        if (cfg.active_scan_enabled || cfg.active_scan_execution_enabled || cfg.sparse_scan_enabled || cfg.sparse_scan_yaw_match_enabled || cfg.yaw_correction_enabled) {
            update_active_scan_from_snapshots(quality_now_s, qs, latest_supervisor_snapshot, true);
            if (cfg.active_scan_enabled) metrics.active_scan = active_scan.run_stats(quality_now_s);
        }
        if (cfg.active_scan_execution_enabled) {
            update_active_scan_command_from_snapshots(quality_now_s, qs, latest_supervisor_snapshot, latest_active_scan_snapshot, true);
            metrics.active_scan_command = active_scan_command.run_stats(quality_now_s);
        }
        if (cfg.sparse_scan_enabled) {
            SparseScanInput finish_in;
            finish_in.timestamp_s = quality_now_s;
            finish_in.timestamp_us = now_us_steady();
            finish_in.pose = loc.pose();
            finish_in.linear_speed_mps = loc.v();
            finish_in.yaw_rate_radps = 0.0;
            finish_in.localization_valid = true;
            finish_in.map_quality = latest_map_quality_snapshot;
            finish_in.supervisor = latest_supervisor_snapshot;
            finish_in.active_scan = latest_active_scan_snapshot;
            finish_in.active_scan_command = latest_active_scan_command_snapshot;
            sparse_scan.update(finish_in);
            drain_sparse_scan_logs();
            update_yaw_match_from_completed_scans(quality_now_s, sparse_scan.drain_completed_sessions_for_matching());
            metrics.sparse_scan = sparse_scan.run_stats(quality_now_s);
        }
        if (cfg.sparse_scan_yaw_match_enabled) {
            drain_yaw_match_logs(quality_now_s, false);
            metrics.sparse_scan_yaw_match = sparse_scan_yaw_match.run_stats(quality_now_s);
        }
        if (cfg.yaw_correction_enabled) {
            update_yaw_correction_gate_from_summary(quality_now_s, latest_yaw_match_summary, true);
            metrics.yaw_correction = yaw_correction_gate.run_stats(quality_now_s);
            metrics.yaw_correction_apply = yaw_correction_apply.run_stats();
        }
    }
    write_run_metrics(run_dir + "/metrics.csv", metrics, loc, grid, tof_health, cfg, sensors.encoder_stats());
    std::cout << "robot_slamd saved run_dir=" << run_dir << "\n";
    return 0;
}


} // namespace robot_slamd
