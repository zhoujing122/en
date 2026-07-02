#!/usr/bin/env python3
import argparse
import csv
from collections import Counter, defaultdict
from pathlib import Path


REQUIRED_FILES = [
    "config.resolved.yaml",
    "trajectory.csv",
    "localization_log.csv",
    "tof_log.csv",
    "metrics.csv",
    "map.pgm",
    "map.yaml",
]


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def read_metrics(path):
    out = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            out[row["key"]] = row["value"]
    return out


def split_flags(value):
    if not value or value == "ok":
        return []
    return [x for x in value.split("|") if x]


def as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def summarize(run_dir, strict):
    run_dir = Path(run_dir)
    missing = [name for name in REQUIRED_FILES if not (run_dir / name).exists()]
    print(f"run_dir: {run_dir}")
    if missing:
        print("missing_files: " + ",".join(missing))
        if strict:
            return 1
    else:
        print("missing_files: none")

    metrics = read_metrics(run_dir / "metrics.csv") if (run_dir / "metrics.csv").exists() else {}
    if metrics:
        interesting = [
            "localization_updates",
            "tof_samples",
            "tof_hit_updates",
            "tof_free_only",
            "tof_rejected",
            "map_updates",
            "low_odom_quality_pauses",
            "tof_unhealthy_pauses",
            "encoder_rejected_updates",
            "gyro_spikes",
            "map_occupied_cells",
            "map_free_cells",
            "map_unknown_cells",
        ]
        print("metrics:")
        for key in interesting:
            if key in metrics:
                print(f"  {key}: {metrics[key]}")
        for prefix in ("front", "left", "right"):
            keys = [
                f"{prefix}_hit_rate",
                f"{prefix}_reject_rate",
                f"{prefix}_gap_count",
                f"{prefix}_confidence_mean",
                f"{prefix}_unhealthy",
            ]
            print(f"  {prefix}: " + ", ".join(f"{k[len(prefix)+1:]}={metrics.get(k, 'na')}" for k in keys))

    if (run_dir / "trajectory.csv").exists():
        rows = read_csv(run_dir / "trajectory.csv")
        if rows:
            t0 = int(rows[0]["timestamp_us"])
            t1 = int(rows[-1]["timestamp_us"])
            print(f"trajectory_points: {len(rows)}")
            print(f"trajectory_duration_s: {(t1 - t0) / 1000000.0:.3f}")
            print(f"final_pose: x={as_float(rows[-1].get('x_m')):.3f}, y={as_float(rows[-1].get('y_m')):.3f}, yaw={as_float(rows[-1].get('yaw_rad')):.3f}")
        else:
            print("trajectory_points: 0")

    if (run_dir / "localization_log.csv").exists():
        flag_counts = Counter()
        for row in read_csv(run_dir / "localization_log.csv"):
            flag_counts.update(split_flags(row.get("warn_flags", "")))
        print("localization_warnings:")
        if flag_counts:
            for key, value in flag_counts.most_common():
                print(f"  {key}: {value}")
        else:
            print("  none")

    if (run_dir / "tof_log.csv").exists():
        decision_counts = Counter()
        by_id = defaultdict(lambda: {"n": 0, "conf_sum": 0.0, "valid": 0})
        for row in read_csv(run_dir / "tof_log.csv"):
            decision_counts[row.get("decision", "")] += 1
            sensor_id = row.get("id", "")
            by_id[sensor_id]["n"] += 1
            by_id[sensor_id]["conf_sum"] += as_float(row.get("confidence"))
            by_id[sensor_id]["valid"] += int(row.get("valid", "0") == "1")
        print("tof_decisions:")
        if decision_counts:
            for key, value in decision_counts.most_common():
                print(f"  {key}: {value}")
        else:
            print("  none")
        print("tof_by_id:")
        for sensor_id in sorted(by_id):
            item = by_id[sensor_id]
            mean = item["conf_sum"] / item["n"] if item["n"] else 0.0
            print(f"  {sensor_id}: samples={item['n']}, valid={item['valid']}, confidence_mean={mean:.1f}")

    if strict:
        required_positive = ["localization_updates", "tof_samples"]
        bad = [key for key in required_positive if int(float(metrics.get(key, "0"))) <= 0]
        if bad:
            print("strict_fail_zero_metrics: " + ",".join(bad))
            return 1
    return 0


def main():
    parser = argparse.ArgumentParser(description="Summarize a robot_slamd run directory.")
    parser.add_argument("run_dir")
    parser.add_argument("--strict", action="store_true", help="return non-zero for missing files or zero core metrics")
    args = parser.parse_args()
    raise SystemExit(summarize(args.run_dir, args.strict))


if __name__ == "__main__":
    main()
