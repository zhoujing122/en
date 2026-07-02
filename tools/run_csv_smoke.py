#!/usr/bin/env python3
import argparse
import csv
import shutil
import subprocess
from pathlib import Path


def write_rows(path, header, rows):
    with Path(path).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def generate_inputs(work_dir, duration_s):
    data_dir = Path(work_dir) / "csv_inputs"
    data_dir.mkdir(parents=True, exist_ok=True)
    encoder_rows = []
    imu_rows = []
    tof_rows = []
    left = 0
    right = 0
    for i in range(int(duration_s * 50)):
        t_us = i * 20000
        left += 12
        right += 13
        if i == 35:
            left += 5000
            right += 5000
        encoder_rows.append([t_us, left, right])
        gyro = 0.04
        if i == 45:
            gyro = 8.0
        imu_rows.append([t_us, gyro])
    for i in range(int(duration_s * 30)):
        t_us = i * 33333
        if i < 10:
            samples = [
                ["front", 1200, 0, 90],
                ["left", 700, 0, 85],
                ["right", 750, 0, 85],
            ]
        elif i < 18:
            samples = [
                ["front", 1210, 0, 20],
                ["left", 710, 0, 85],
                ["right", 750, 0, 85],
            ]
        elif i < 24:
            samples = [
                ["front", 4500, 255, 0],
                ["left", 720, 0, 85],
                ["right", 740, 0, 85],
            ]
        else:
            samples = [
                ["front", 1210, 0, 90],
                ["left", 1800, 6, 10],
                ["right", 735, 0, 85],
            ]
        for sensor_id, raw_mm, status, confidence in samples:
            tof_rows.append([t_us, sensor_id, raw_mm, status, confidence])
    write_rows(data_dir / "encoder.csv", ["timestamp_us", "left_ticks", "right_ticks"], encoder_rows)
    write_rows(data_dir / "imu.csv", ["timestamp_us", "gyro_z_rad_s"], imu_rows)
    write_rows(data_dir / "tof.csv", ["timestamp_us", "id", "raw_range_mm", "range_status", "confidence"], tof_rows)
    return data_dir


def write_config(path, data_dir, output_dir):
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
  enabled: false
""")


def latest_run(output_dir):
    latest = Path(output_dir) / "latest"
    if latest.exists():
        return latest.resolve()
    runs = sorted((Path(output_dir) / "runs").glob("*"))
    return runs[-1] if runs else None


def main():
    parser = argparse.ArgumentParser(description="Run robot_slamd with generated CSV inputs that include common sensor faults.")
    parser.add_argument("--binary", required=True, help="path to robot_slamd binary")
    parser.add_argument("--work-dir", default="/tmp/robot_slamd_csv_smoke")
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--keep", action="store_true")
    args = parser.parse_args()

    work_dir = Path(args.work_dir)
    if work_dir.exists() and not args.keep:
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    output_dir = work_dir / "output"
    data_dir = generate_inputs(work_dir, args.duration_s)
    config_path = work_dir / "config.csv_smoke.yaml"
    write_config(config_path, data_dir, output_dir)

    cmd = [args.binary, "--config", str(config_path), "--duration-s", str(args.duration_s), "--output-dir", str(output_dir)]
    print("running: " + " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print(proc.stdout, end="", flush=True)
    print(proc.stderr, end="", flush=True)
    if proc.returncode != 0:
        return proc.returncode

    run_dir = latest_run(output_dir)
    if not run_dir:
        print("no run directory produced")
        return 1
    print(f"run_dir: {run_dir}")
    summarizer = Path(__file__).with_name("summarize_run.py")
    summary = subprocess.run([str(summarizer), str(run_dir), "--strict"], check=False)
    if summary.returncode != 0:
        return summary.returncode

    tof_text = (run_dir / "tof_log.csv").read_text()
    loc_text = (run_dir / "localization_log.csv").read_text()
    expected = [
        ("rejected_confidence", tof_text),
        ("free_only", tof_text),
        ("log_only", tof_text),
        ("encoder_jump", loc_text),
        ("gyro_spike", loc_text),
    ]
    missing = [name for name, text in expected if name not in text]
    if missing:
        print("missing_expected_observability: " + ",".join(missing))
        return 1
    print("csv_smoke: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
