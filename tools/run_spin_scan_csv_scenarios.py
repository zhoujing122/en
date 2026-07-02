#!/usr/bin/env python3
import argparse
import csv
import math
import shutil
import subprocess
from pathlib import Path

TOF_EXTRINSICS = {
    "front": (0.10, 0.00, 0.0),
    "left": (0.00, 0.08, math.radians(90.0)),
    "right": (0.00, -0.08, math.radians(-90.0)),
}


def write_rows(path, header, rows):
    with Path(path).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def ray_segment(px, py, yaw, x1, y1, x2, y2):
    dx, dy = math.cos(yaw), math.sin(yaw)
    sx, sy = x2 - x1, y2 - y1
    det = -dx * sy + dy * sx
    if abs(det) < 1e-9:
        return None
    bx, by = x1 - px, y1 - py
    t = (-sy * bx + sx * by) / det
    u = (-dy * bx + dx * by) / det
    if t > 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def ray_world(px, py, yaw, segments, max_range=3.0):
    best = 1e9
    for seg in segments:
        t = ray_segment(px, py, yaw, *seg)
        if t is not None and t < best:
            best = t
    return best if best < 1e8 else max_range + 1.0


def tof_ranges(pose, segments, max_range=3.0):
    x, y, yaw = pose
    cy, sy = math.cos(yaw), math.sin(yaw)
    out = {}
    for sensor_id, (ex, ey, eyaw) in TOF_EXTRINSICS.items():
        sx = x + cy * ex - sy * ey
        syw = y + sy * ex + cy * ey
        out[sensor_id] = ray_world(sx, syw, yaw + eyaw, segments, max_range)
    return out


def make_spin_path(duration_s, angular_speed_radps, stop_after_s=None):
    n = int(duration_s * 50)
    path = []
    yaw = 0.0
    prev_t = 0.0
    for i in range(n):
        t = i / 50.0
        dt = 0.0 if i == 0 else t - prev_t
        prev_t = t
        w = angular_speed_radps if stop_after_s is None or t <= stop_after_s else 0.0
        yaw = wrap_pi(yaw + w * dt)
        path.append((0.0, 0.0, yaw))
    return path


def cumulative_ticks(path, wheel_base=0.180, wheel_radius=0.032, ticks_per_rev=1024):
    left = 0
    right = 0
    enc_rows = []
    imu_rows = []
    prev = path[0]
    for i, pose in enumerate(path):
        t_us = i * 20000
        if i > 0:
            dyaw = wrap_pi(pose[2] - prev[2])
            dl = -dyaw * wheel_base * 0.5
            dr = dyaw * wheel_base * 0.5
            left += int(round(dl / (2.0 * math.pi * wheel_radius) * ticks_per_rev))
            right += int(round(dr / (2.0 * math.pi * wheel_radius) * ticks_per_rev))
            gyro = dyaw / 0.02
        else:
            gyro = 0.0
        enc_rows.append([t_us, left, right])
        imu_rows.append([t_us, gyro])
        prev = pose
    return enc_rows, imu_rows


def sample_pose(path, t_us):
    idx = min(len(path) - 1, max(0, round(t_us / 20000)))
    return path[idx]


def scenario_spec(name):
    asymmetric_room = [
        (1.15, -1.10, 1.15, 0.95),
        (-0.95, 0.95, 1.15, 0.95),
        (-0.95, -1.10, -0.95, 0.35),
        (0.25, -0.75, 1.15, -0.75),
        (-0.45, 0.10, -0.10, 0.10),
    ]
    if name == "case_spin_corner_good":
        return {"segments": asymmetric_room, "duration": 26.0, "w": 0.25, "confidence": 92, "max_duration": 25.0, "min_margin": 0.0, "multi_window": 0.001, "min_reliability": 0.3}
    if name == "case_spin_flat_wall_degenerate":
        return {"segments": [(1.20, -2.0, 1.20, 2.0)], "duration": 26.0, "w": 0.25, "confidence": 92, "max_duration": 25.0}
    if name == "case_spin_corridor_ambiguous":
        return {"segments": [(-2.0, 0.65, 2.0, 0.65), (-2.0, -0.65, 2.0, -0.65)], "duration": 26.0, "w": 0.25, "confidence": 92, "max_duration": 25.0}
    if name == "case_spin_bad_tof_confidence":
        return {"segments": asymmetric_room, "duration": 26.0, "w": 0.25, "confidence": 8, "max_duration": 25.0}
    if name == "case_spin_insufficient_rotation":
        return {"segments": asymmetric_room, "duration": 10.0, "w": 0.15, "confidence": 92, "max_duration": 8.0}
    if name == "case_spin_too_fast_rejected":
        return {"segments": asymmetric_room, "duration": 8.0, "w": 0.80, "confidence": 92, "max_duration": 8.0}
    raise ValueError(name)


def generate_case(case_dir, name):
    spec = scenario_spec(name)
    data_dir = Path(case_dir) / "csv_inputs"
    data_dir.mkdir(parents=True, exist_ok=True)
    path = make_spin_path(spec["duration"], spec["w"])
    enc_rows, imu_rows = cumulative_ticks(path)
    tof_rows = []
    for i in range(int(spec["duration"] * 30)):
        t_us = i * 33333
        pose = sample_pose(path, t_us)
        ranges = tof_ranges(pose, spec["segments"])
        for sensor_id in ["front", "left", "right"]:
            r = ranges[sensor_id]
            status = 0 if r <= 3.0 else 255
            raw_mm = int(round(min(r, 4.5) * 1000.0))
            confidence = spec["confidence"] if status == 0 else min(spec["confidence"], 10)
            tof_rows.append([t_us, sensor_id, raw_mm, status, confidence])
    write_rows(data_dir / "encoder.csv", ["timestamp_us", "left_ticks", "right_ticks"], enc_rows)
    write_rows(data_dir / "imu.csv", ["timestamp_us", "gyro_z_rad_s"], imu_rows)
    write_rows(data_dir / "tof.csv", ["timestamp_us", "id", "raw_range_mm", "range_status", "confidence"], tof_rows)
    return data_dir, spec


def write_config(path, data_dir, output_dir, spec):
    Path(path).write_text(f"""robot:
  wheel_base_m: 0.180
  wheel_radius_left_m: 0.032
  wheel_radius_right_m: 0.032
  encoder_ticks_per_rev: 1024

encoder:
  source: csv
  path: {data_dir / 'encoder.csv'}

imu:
  source: csv
  path: {data_dir / 'imu.csv'}
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
  csv_path: {data_dir / 'tof.csv'}
  frequency_hz: 30
  min_range_m: 0.05
  max_range_m: 3.0
  confidence_min: 70
  median_window: 3
  jump_reject_m: 1.5
  mad_reject_m: 1.0
  stable_hits_required: 1

tof_extrinsics:
  front.x_m: 0.10
  front.y_m: 0.00
  front.yaw_deg: 0
  front.fov_deg: 10
  left.x_m: 0.00
  left.y_m: 0.08
  left.yaw_deg: 90
  left.fov_deg: 10
  right.x_m: 0.00
  right.y_m: -0.08
  right.yaw_deg: -90
  right.fov_deg: 10

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
  static_scan_stable_required: 1
  static_scan_boost: 1.2

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
  mode: off

spin_scan_localization:
  enabled: true
  mode: observe_only
  min_rotation_deg: 300
  target_rotation_deg: 360
  max_duration_s: {spec['max_duration']}
  max_linear_speed_mps: 0.03
  angular_speed_min_radps: 0.08
  angular_speed_max_radps: 0.45
  angle_bin_deg: 3.0
  min_valid_bins: 20
  min_valid_bin_ratio: 0.15
  match_search_yaw_deg: 15
  match_search_yaw_step_deg: 1
  min_best_second_margin: {spec.get("min_margin", 0.02)}
  flat_curve_sharpness_thresh: 0.02
  multimodal_yaw_gap_deg: 3.0
  multimodal_residual_window: {spec.get("multi_window", 0.02)}
  min_reliability: {spec.get("min_reliability", 0.4)}
  debug_curve: true
""")


def latest_run(output_dir):
    latest = Path(output_dir) / "latest"
    if latest.exists():
        return latest.resolve()
    runs = sorted((Path(output_dir) / "runs").glob("*"))
    return runs[-1] if runs else None


def parse_analysis(text):
    out = {}
    for line in text.splitlines():
        if ": " not in line or line.startswith("  "):
            continue
        key, value = line.split(": ", 1)
        try:
            out[key] = float(value)
        except ValueError:
            out[key] = value
    return out


SUMMARY_FIELDS = [
    "scenario",
    "completed_scans",
    "matched_scans",
    "usable_ratio",
    "valid_bins_mean",
    "valid_bin_ratio_mean",
    "reliability_mean",
    "flat_curve_ratio",
    "multimodal_ratio",
    "expected_behavior",
    "pass",
]

EXPECTED_BEHAVIOR = {
    "case_spin_corner_good": "spin_scan_usable",
    "case_spin_flat_wall_degenerate": "degenerate_flat_or_multimodal",
    "case_spin_corridor_ambiguous": "degenerate_multimodal",
    "case_spin_bad_tof_confidence": "reject_bad_tof",
    "case_spin_insufficient_rotation": "reject_insufficient_rotation",
    "case_spin_too_fast_rejected": "reject_too_fast_spin",
}


def metric(row, key):
    try:
        return float(row.get(key, 0.0))
    except (TypeError, ValueError):
        return 0.0


def scenario_pass(name, row):
    if name == "case_spin_corner_good":
        return metric(row, "completed_scans") > 0 and metric(row, "matched_scans") > 0 and metric(row, "usable_ratio") > 0 and metric(row, "reliability_mean") > 0
    if name == "case_spin_flat_wall_degenerate":
        return metric(row, "flat_curve_ratio") >= 0.5 or metric(row, "multimodal_ratio") >= 0.5 or metric(row, "usable_ratio") == 0
    if name == "case_spin_corridor_ambiguous":
        return metric(row, "multimodal_ratio") >= 0.5 or metric(row, "usable_ratio") == 0
    if name == "case_spin_bad_tof_confidence":
        return metric(row, "usable_ratio") == 0 and metric(row, "reliability_mean") < 0.3
    if name == "case_spin_insufficient_rotation":
        return metric(row, "completed_scans") == 0 or metric(row, "usable_ratio") == 0
    if name == "case_spin_too_fast_rejected":
        return metric(row, "completed_scans") == 0 or metric(row, "usable_ratio") == 0
    return False


def summary_row(name, analysis):
    row = {field: "" for field in SUMMARY_FIELDS}
    row["scenario"] = name
    for key in SUMMARY_FIELDS:
        if key in analysis:
            row[key] = analysis[key]
    row["expected_behavior"] = EXPECTED_BEHAVIOR[name]
    row["pass"] = "true" if scenario_pass(name, row) else "false"
    return row


def run_case(binary, work_dir, name):
    case_dir = Path(work_dir) / name
    data_dir, spec = generate_case(case_dir, name)
    output_dir = case_dir / "output"
    config_path = case_dir / "config.yaml"
    write_config(config_path, data_dir, output_dir, spec)
    cmd = [binary, "--config", str(config_path), "--duration-s", str(spec["duration"]), "--output-dir", str(output_dir)]
    print("running: " + " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print(proc.stdout, end="")
    print(proc.stderr, end="")
    if proc.returncode != 0:
        return proc.returncode, None, {}
    run_dir = latest_run(output_dir)
    if not run_dir:
        print(f"{name}: no run directory")
        return 1, None, {}
    analyzer = Path(__file__).with_name("analyze_spin_scan.py")
    analysis = subprocess.run(["python3", str(analyzer), str(run_dir)], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (case_dir / "analysis.txt").write_text(analysis.stdout + analysis.stderr)
    print(f"{name}: run_dir={run_dir}")
    print(analysis.stdout, end="")
    print(analysis.stderr, end="")
    if analysis.returncode != 0:
        return analysis.returncode, run_dir, parse_analysis(analysis.stdout + analysis.stderr)
    return 0, run_dir, parse_analysis(analysis.stdout)


def main():
    parser = argparse.ArgumentParser(description="Run spin_scan_localization CSV scenario matrix.")
    parser.add_argument("--binary", required=True)
    parser.add_argument("--work-dir", default="/tmp/robot_slamd_spin_scan_scenarios")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--keep", action="store_true")
    args = parser.parse_args()

    work_dir = Path(args.work_dir)
    if work_dir.exists() and not args.keep:
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)

    scenarios = list(EXPECTED_BEHAVIOR.keys())
    rows = []
    failed = []
    for name in scenarios:
        rc, _, analysis = run_case(args.binary, work_dir, name)
        row = summary_row(name, analysis)
        if rc != 0:
            row["pass"] = "false"
        rows.append(row)
        if row["pass"] != "true":
            failed.append(name)

    summary_path = work_dir / "spin_scenario_summary.csv"
    with summary_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"spin_scenario_summary_csv: {summary_path}")
    if failed:
        print("failed_scenarios: " + ",".join(failed))
    else:
        print("failed_scenarios: none")
    if failed and args.strict:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
