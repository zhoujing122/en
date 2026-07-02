#!/usr/bin/env python3
import argparse
import csv
import math
import shutil
import subprocess
from pathlib import Path


def write_rows(path, header, rows):
    with Path(path).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def generate_stationary_inputs(work_dir, duration_s):
    data_dir = Path(work_dir) / "csv_inputs"
    data_dir.mkdir(parents=True, exist_ok=True)
    encoder_rows = []
    imu_rows = []
    tof_rows = []
    for i in range(int(duration_s * 50)):
        t_us = i * 20000
        encoder_rows.append([t_us, 0, 0])
        imu_rows.append([t_us, 0.0])
    for i in range(int(duration_s * 30)):
        t_us = i * 33333
        tof_rows.extend([
            [t_us, "front", 1900, 0, 90],
            [t_us, "left", 1920, 0, 90],
            [t_us, "right", 1920, 0, 90],
        ])
    write_rows(data_dir / "encoder.csv", ["timestamp_us", "left_ticks", "right_ticks"], encoder_rows)
    write_rows(data_dir / "imu.csv", ["timestamp_us", "gyro_z_rad_s"], imu_rows)
    write_rows(data_dir / "tof.csv", ["timestamp_us", "id", "raw_range_mm", "range_status", "confidence"], tof_rows)
    return data_dir


def write_config(path, data_dir, output_dir, debug_x_m, debug_yaw_deg):
    Path(path).write_text(f"""robot:
  wheel_base_m: 0.180
  wheel_radius_left_m: 0.032
  wheel_radius_right_m: 0.032
  encoder_ticks_per_rev: 1024

encoder:
  source: csv
  path: {data_dir / "encoder.csv"}

imu:
  source: csv
  path: {data_dir / "imu.csv"}
  mode: localization
  gyro_axis: z
  gyro_sign: 1
  gyro_bias_static_calib_s: 0
  static_gyro_max_abs_rad_s: 0.03
  yaw_fusion_alpha: 0.95

localization:
  encoder_max_wheel_speed_mps: 1.0
  gyro_lpf_alpha: 0.30
  gyro_spike_radps: 4.0
  stationary_linear_speed_mps: 0.01
  stationary_angular_speed_radps: 0.02
  gyro_bias_adapt_alpha: 0.001

tof:
  source: csv
  csv_path: {data_dir / "tof.csv"}
  frequency_hz: 30
  min_range_m: 0.05
  max_range_m: 3.0
  confidence_min: 70
  median_window: 5
  jump_reject_m: 0.6
  mad_reject_m: 0.20
  stable_hits_required: 2

mapping:
  resolution_m: 0.05
  chunk_cells: 64
  initial_width_m: 10.0
  initial_height_m: 10.0
  ray_step_deg: 2.0
  hit_thickness_m: 0.08
  log_odds_occ: 0.85
  log_odds_free: -0.35
  log_odds_min: -3.5
  log_odds_max: 3.5
  occupied_thresh: 0.65
  free_thresh: 0.25
  max_cells_per_tof_update: 4096
  static_scan_stable_required: 3
  static_scan_boost: 1.5

runtime:
  localization_hz: 50
  tof_read_hz: 30
  mapping_hz: 10
  log_hz: 10

runtime_limits:
  max_linear_speed_mps: 0.25
  pause_linear_speed_mps: 0.40
  max_angular_speed_radps: 0.785
  pause_angular_speed_radps: 1.571

logging:
  output_dir: {output_dir}

map_saver:
  save_on_exit: true
  autosave_period_s: 0

tof_pose_correction:
  enabled: true
  mode: score_only
  update_hz: 5.0
  search_xy_m: 0.10
  search_xy_step_m: 0.01
  search_yaw_deg: 10.0
  search_yaw_step_deg: 1.0
  min_valid_tof: 2
  max_residual_m: 0.25
  min_margin: 0.0
  max_xy_m: 0.03
  max_yaw_deg: 2.0
  gain: 0.3
  mapping_mode_apply: false
  debug_pose_offset_x_m: {debug_x_m}
  debug_pose_offset_y_m: 0.0
  debug_pose_offset_yaw_deg: {debug_yaw_deg}
""")


def latest_run(output_dir):
    latest = Path(output_dir) / "latest"
    if latest.exists():
        return latest.resolve()
    runs = sorted((Path(output_dir) / "runs").glob("*"))
    return runs[-1] if runs else None


def read_rows(path):
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def valid_correction_rows(run_dir):
    rows = []
    for row in read_rows(Path(run_dir) / "tof_correction_log.csv"):
        if int(row.get("used_tof", "0")) >= 2 and int(row.get("candidate_count", "0")) > 0:
            rows.append(row)
    return rows


def mean(values):
    return sum(values) / len(values) if values else 0.0


def run_case(binary, work_dir, case_name, debug_x_m, debug_yaw_deg, duration_s):
    case_dir = Path(work_dir) / case_name
    data_dir = generate_stationary_inputs(case_dir, duration_s)
    output_dir = case_dir / "output"
    config_path = case_dir / "config.yaml"
    write_config(config_path, data_dir, output_dir, debug_x_m, debug_yaw_deg)
    cmd = [binary, "--config", str(config_path), "--duration-s", str(duration_s), "--output-dir", str(output_dir)]
    print("running: " + " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print(proc.stdout, end="", flush=True)
    print(proc.stderr, end="", flush=True)
    if proc.returncode != 0:
        raise SystemExit(proc.returncode)
    run_dir = latest_run(output_dir)
    if not run_dir:
        raise SystemExit(f"{case_name}: no run directory")
    rows = valid_correction_rows(run_dir)
    if not rows:
        raise SystemExit(f"{case_name}: no valid correction rows")
    tail = rows[len(rows) // 2:]
    best_dx_mean = mean([float(r["best_dx"]) for r in tail])
    best_dyaw_mean = mean([float(r["best_dyaw"]) for r in tail])
    accepted = sum(int(r["accepted"]) for r in rows)
    improvement_mean = mean([float(r["improvement"]) for r in tail])
    print(f"{case_name}: run_dir={run_dir}")
    print(f"{case_name}: best_dx_mean={best_dx_mean:.6f}, best_dyaw_mean={best_dyaw_mean:.6f}, improvement_mean={improvement_mean:.6f}, accepted={accepted}")
    if accepted != 0:
        raise SystemExit(f"{case_name}: score_only accepted must remain 0")
    return run_dir, best_dx_mean, best_dyaw_mean


def main():
    parser = argparse.ArgumentParser(description="CSV smoke for ToF correction score_only debug pose offsets.")
    parser.add_argument("--binary", required=True)
    parser.add_argument("--work-dir", default="/tmp/robot_slamd_tof_correction_smoke")
    parser.add_argument("--duration-s", type=float, default=4.0)
    parser.add_argument("--keep", action="store_true")
    args = parser.parse_args()

    work_dir = Path(args.work_dir)
    if work_dir.exists() and not args.keep:
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)

    yaw_run, _, yaw_dyaw = run_case(args.binary, work_dir, "yaw_plus_5deg", 0.0, 5.0, args.duration_s)
    x_run, x_dx, _ = run_case(args.binary, work_dir, "x_plus_5cm", 0.05, 0.0, args.duration_s)

    if yaw_dyaw >= -math.radians(1.0):
        raise SystemExit(f"yaw_plus_5deg: expected negative best_dyaw, got {yaw_dyaw}")
    if x_dx >= -0.015:
        raise SystemExit(f"x_plus_5cm: expected negative best_dx, got {x_dx}")
    print("tof_correction_csv_smoke: ok")
    print(f"yaw_run: {yaw_run}")
    print(f"x_run: {x_run}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
