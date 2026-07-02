#!/usr/bin/env python3
import argparse
import csv
from collections import Counter
from pathlib import Path


def as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value, default=0):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def read_csv(path):
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def read_metrics(path):
    out = {}
    if not path.exists():
        return out
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if "key" in row and "value" in row:
                out[row["key"]] = row["value"]
    return out


def mean(values):
    return sum(values) / len(values) if values else 0.0


def ratio_true(rows, key):
    return sum(1 for r in rows if as_int(r.get(key)) != 0) / len(rows) if rows else 0.0


def analyze(run_dir, strict=False):
    run_dir = Path(run_dir)
    scan_path = run_dir / "spin_scan.csv"
    bins_path = run_dir / "spin_scan_bins.csv"
    curve_path = run_dir / "spin_match_curve.csv"
    summary_path = run_dir / "spin_match_summary.csv"
    metrics_path = run_dir / "metrics.csv"

    scans = read_csv(scan_path)
    bins = read_csv(bins_path)
    curve = read_csv(curve_path)
    summary = read_csv(summary_path)
    metrics = read_metrics(metrics_path)

    total_scans = len({r.get("scan_id") for r in summary if r.get("scan_id") not in (None, "")})
    completed_scans = sum(1 for r in summary if as_int(r.get("complete")) != 0)
    matched_scans = sum(1 for r in summary if as_int(r.get("matched")) != 0)
    usable_scans = sum(1 for r in summary if as_int(r.get("usable")) != 0)
    usable_ratio = usable_scans / total_scans if total_scans else 0.0
    valid_bins_mean = mean([as_float(r.get("valid_bins")) for r in summary])
    valid_bin_ratio_mean = mean([as_float(r.get("valid_bin_ratio")) for r in summary])
    reliabilities = [as_float(r.get("reliability")) for r in summary]
    reliability_mean = mean(reliabilities)
    reliability_min = min(reliabilities) if reliabilities else 0.0
    flat_curve_ratio = ratio_true(summary, "flat_curve")
    multimodal_ratio = ratio_true(summary, "multimodal")
    best_at_search_edge_ratio = ratio_true(summary, "best_at_search_edge")
    best_second_margin_mean = mean([as_float(r.get("best_second_margin")) for r in summary])
    best_second_yaw_gap_mean_deg = mean([as_float(r.get("best_second_yaw_gap_deg")) for r in summary])
    curve_sharpness_mean = mean([as_float(r.get("curve_sharpness")) for r in summary])
    reason_counts = Counter(r.get("reason", "") for r in summary)

    print(f"run_dir: {run_dir}")
    print(f"spin_scan_csv: {scan_path if scan_path.exists() else 'missing'}")
    print(f"spin_scan_bins_csv: {bins_path if bins_path.exists() else 'missing'}")
    print(f"spin_match_curve_csv: {curve_path if curve_path.exists() else 'missing'}")
    print(f"spin_match_summary_csv: {summary_path if summary_path.exists() else 'missing'}")
    print(f"metrics_csv: {metrics_path if metrics_path.exists() else 'missing'}")
    print(f"spin_scan_rows: {len(scans)}")
    print(f"spin_scan_bins_rows: {len(bins)}")
    print(f"spin_match_curve_rows: {len(curve)}")
    print(f"spin_match_summary_rows: {len(summary)}")
    print(f"total_scans: {total_scans}")
    print(f"completed_scans: {completed_scans}")
    print(f"matched_scans: {matched_scans}")
    print(f"usable_scans: {usable_scans}")
    print(f"usable_ratio: {usable_ratio:.6f}")
    print(f"valid_bins_mean: {valid_bins_mean:.6f}")
    print(f"valid_bin_ratio_mean: {valid_bin_ratio_mean:.6f}")
    print(f"reliability_mean: {reliability_mean:.9f}")
    print(f"reliability_min: {reliability_min:.9f}")
    print(f"flat_curve_ratio: {flat_curve_ratio:.6f}")
    print(f"multimodal_ratio: {multimodal_ratio:.6f}")
    print(f"best_at_search_edge_ratio: {best_at_search_edge_ratio:.6f}")
    print(f"best_second_margin_mean: {best_second_margin_mean:.9f}")
    print(f"best_second_yaw_gap_mean_deg: {best_second_yaw_gap_mean_deg:.6f}")
    print(f"curve_sharpness_mean: {curve_sharpness_mean:.9f}")
    print("reason_counts:")
    if reason_counts:
        for reason, count in reason_counts.most_common():
            print(f"  {reason}: {count}")
    else:
        print("  none")
    for key in [
        "spin_scan_attempts",
        "spin_scan_completed",
        "spin_scan_failed",
        "spin_scan_matched",
        "spin_scan_usable",
        "spin_scan_valid_bins_mean",
        "spin_scan_valid_bin_ratio_mean",
        "spin_scan_reliability_mean",
        "spin_scan_reliability_min",
    ]:
        if key in metrics:
            print(f"metric_{key}: {metrics[key]}")

    if strict:
        failures = []
        if total_scans <= 0:
            failures.append("total_scans <= 0")
        if completed_scans <= 0:
            failures.append("completed_scans <= 0")
        if matched_scans <= 0:
            failures.append("matched_scans <= 0")
        min_valid_bins = 0
        if summary:
            min_valid_bins = min(as_int(r.get("valid_bins")) for r in summary)
        if valid_bins_mean < min_valid_bins:
            failures.append("valid_bins_mean below observed minimum")
        if failures:
            print("strict: fail")
            for item in failures:
                print(f"  {item}")
            return 1
        print("strict: pass")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Analyze spin_scan_localization observe_only logs.")
    parser.add_argument("run_dir")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    raise SystemExit(analyze(args.run_dir, args.strict))


if __name__ == "__main__":
    main()
