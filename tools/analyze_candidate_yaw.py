#!/usr/bin/env python3
import argparse
import csv
import math
from collections import Counter
from pathlib import Path

BAD_REASONS = {"bad_odom", "tof_unhealthy", "insufficient_tof", "no_map"}


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
    with Path(path).open(newline="") as f:
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


def resolve_input(path):
    path = Path(path)
    if path.is_dir():
        return (
            path,
            path / "candidate_yaw.csv",
            path / "tof_correction_log.csv",
            path / "metrics.csv",
            path / "yaw_residual_summary.csv",
            path / "yaw_residual_curve.csv",
        )
    run_dir = path.parent
    return (
        run_dir,
        path,
        run_dir / "tof_correction_log.csv",
        run_dir / "metrics.csv",
        run_dir / "yaw_residual_summary.csv",
        run_dir / "yaw_residual_curve.csv",
    )


def mean(values):
    return sum(values) / len(values) if values else 0.0


def stddev(values):
    if not values:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((v - m) * (v - m) for v in values) / len(values))


def max_step(values):
    if len(values) < 2:
        return 0.0
    return max(abs(values[i] - values[i - 1]) for i in range(1, len(values)))


def reason_majority_bad(reason_counts, total_rows):
    if total_rows <= 0:
        return False
    bad = sum(reason_counts.get(reason, 0) for reason in BAD_REASONS)
    return bad > total_rows / 2.0


def ratio_true(rows, key):
    return sum(1 for r in rows if as_int(r.get(key)) != 0) / len(rows) if rows else 0.0


def residual_stats(rows, prefix):
    valid = [r for r in rows if as_float(r.get(f"{prefix}_obs_m")) > 0.0]
    residuals = [as_float(r.get(f"{prefix}_residual_m")) for r in valid]
    return mean(residuals), len(valid) / len(rows) if rows else 0.0


def analyze(input_path, strict=False):
    run_dir, candidate_path, correction_path, metrics_path, summary_path, curve_path = resolve_input(input_path)
    if not candidate_path.exists():
        print(f"candidate_yaw_csv: {candidate_path}")
        print("error: candidate_yaw.csv not found")
        return 1

    rows = read_csv(candidate_path)
    total_rows = len(rows)
    stable_rows = sum(1 for r in rows if as_int(r.get("stable")) != 0)
    stable_ratio = stable_rows / total_rows if total_rows else 0.0
    improvements = [as_float(r.get("improvement")) for r in rows]
    used_tof = [as_float(r.get("used_tof")) for r in rows]
    filtered_delta = [as_float(r.get("filtered_delta_yaw")) for r in rows]
    raw_delta = [as_float(r.get("raw_delta_yaw")) for r in rows]
    reason_counts = Counter(r.get("reason", "") for r in rows)
    best_second_margins = [as_float(r.get("best_second_margin")) for r in rows]
    best_second_yaw_gaps_deg = [as_float(r.get("best_second_yaw_gap_deg")) for r in rows]
    candidate_reliability = [as_float(r.get("candidate_reliability")) for r in rows]

    improvement_mean = mean(improvements)
    improvement_positive_ratio = sum(1 for v in improvements if v > 0.0) / total_rows if total_rows else 0.0
    used_tof_mean = mean(used_tof)
    filtered_delta_yaw_max_step_rad = max_step(filtered_delta)
    filtered_delta_yaw_std_rad = stddev(filtered_delta)
    raw_delta_yaw_std_rad = stddev(raw_delta)
    first_filtered_delta_yaw = filtered_delta[0] if filtered_delta else 0.0
    last_filtered_delta_yaw = filtered_delta[-1] if filtered_delta else 0.0
    max_abs_filtered_delta_yaw = max((abs(v) for v in filtered_delta), default=0.0)
    best_second_margin_mean = mean(best_second_margins)
    best_second_yaw_gap_mean_deg = mean(best_second_yaw_gaps_deg)
    low_margin_nearby_count = reason_counts.get("yaw_candidate_low_margin_nearby", 0)
    low_margin_ambiguous_count = reason_counts.get("yaw_candidate_low_margin_ambiguous", 0)
    low_margin_nearby_ratio = low_margin_nearby_count / total_rows if total_rows else 0.0
    low_margin_ambiguous_ratio = low_margin_ambiguous_count / total_rows if total_rows else 0.0
    candidate_reliability_mean = mean(candidate_reliability)
    candidate_reliability_min = min(candidate_reliability) if candidate_reliability else 0.0
    candidate_usable_rows = sum(1 for r in rows if as_int(r.get("candidate_usable")) != 0)
    candidate_usable_ratio = candidate_usable_rows / total_rows if total_rows else 0.0
    reject_for_flat_curve_count = sum(1 for r in rows if as_int(r.get("reject_for_flat_curve")) != 0)
    reject_for_multimodal_count = sum(1 for r in rows if as_int(r.get("reject_for_multimodal")) != 0)
    reject_for_edge_best_count = sum(1 for r in rows if as_int(r.get("reject_for_edge_best")) != 0)
    reject_for_low_improvement_count = sum(1 for r in rows if as_int(r.get("reject_for_low_improvement")) != 0)
    reject_for_low_margin_count = sum(1 for r in rows if as_int(r.get("reject_for_low_margin")) != 0)
    reject_for_insufficient_tof_count = sum(1 for r in rows if as_int(r.get("reject_for_insufficient_tof")) != 0)

    correction_rows = read_csv(correction_path) if correction_path.exists() else []
    metrics = read_metrics(metrics_path)
    summary_rows = read_csv(summary_path) if summary_path.exists() else []
    curve_rows = read_csv(curve_path) if curve_path.exists() else []

    curve_sharpness_mean = mean([as_float(r.get("curve_sharpness")) for r in summary_rows])
    flat_curve_ratio = ratio_true(summary_rows, "flat_curve")
    multimodal_ratio = ratio_true(summary_rows, "multimodal")
    best_at_search_edge_ratio = ratio_true(summary_rows, "best_at_search_edge")
    num_local_minima_mean = mean([as_float(r.get("num_local_minima")) for r in summary_rows])
    summary_best_second_yaw_gap_mean_deg = mean([as_float(r.get("best_second_yaw_gap_deg")) for r in summary_rows])
    curve_p50_minus_min_mean = mean([as_float(r.get("curve_p50_residual")) - as_float(r.get("curve_min_residual")) for r in summary_rows])
    front_residual_mean, front_valid_ratio = residual_stats(curve_rows, "front")
    left_residual_mean, left_valid_ratio = residual_stats(curve_rows, "left")
    right_residual_mean, right_valid_ratio = residual_stats(curve_rows, "right")

    print(f"run_dir: {run_dir}")
    print(f"candidate_yaw_csv: {candidate_path}")
    print(f"tof_correction_log_csv: {correction_path if correction_path.exists() else 'missing'}")
    print(f"metrics_csv: {metrics_path if metrics_path.exists() else 'missing'}")
    print(f"yaw_residual_summary_csv: {summary_path if summary_path.exists() else 'missing'}")
    print(f"yaw_residual_curve_csv: {curve_path if curve_path.exists() else 'missing'}")
    print(f"total_rows: {total_rows}")
    print(f"stable_rows: {stable_rows}")
    print(f"stable_ratio: {stable_ratio:.6f}")
    print(f"improvement_mean: {improvement_mean:.9f}")
    print(f"improvement_positive_ratio: {improvement_positive_ratio:.6f}")
    print(f"used_tof_mean: {used_tof_mean:.6f}")
    print(f"filtered_delta_yaw_max_step_rad: {filtered_delta_yaw_max_step_rad:.9f}")
    print(f"filtered_delta_yaw_std_rad: {filtered_delta_yaw_std_rad:.9f}")
    print(f"raw_delta_yaw_std_rad: {raw_delta_yaw_std_rad:.9f}")
    print(f"first_filtered_delta_yaw: {first_filtered_delta_yaw:.9f}")
    print(f"last_filtered_delta_yaw: {last_filtered_delta_yaw:.9f}")
    print(f"max_abs_filtered_delta_yaw: {max_abs_filtered_delta_yaw:.9f}")
    print(f"best_second_margin_mean: {best_second_margin_mean:.9f}")
    print(f"best_second_yaw_gap_mean_deg: {best_second_yaw_gap_mean_deg:.6f}")
    print(f"low_margin_nearby_count: {low_margin_nearby_count}")
    print(f"low_margin_ambiguous_count: {low_margin_ambiguous_count}")
    print(f"low_margin_nearby_ratio: {low_margin_nearby_ratio:.6f}")
    print(f"low_margin_ambiguous_ratio: {low_margin_ambiguous_ratio:.6f}")
    print(f"candidate_reliability_mean: {candidate_reliability_mean:.9f}")
    print(f"candidate_reliability_min: {candidate_reliability_min:.9f}")
    print(f"candidate_usable_rows: {candidate_usable_rows}")
    print(f"candidate_usable_ratio: {candidate_usable_ratio:.6f}")
    print(f"reject_for_flat_curve_count: {reject_for_flat_curve_count}")
    print(f"reject_for_multimodal_count: {reject_for_multimodal_count}")
    print(f"reject_for_edge_best_count: {reject_for_edge_best_count}")
    print(f"reject_for_low_improvement_count: {reject_for_low_improvement_count}")
    print(f"reject_for_low_margin_count: {reject_for_low_margin_count}")
    print(f"reject_for_insufficient_tof_count: {reject_for_insufficient_tof_count}")
    print("reason_counts:")
    if reason_counts:
        for reason, count in reason_counts.most_common():
            print(f"  {reason}: {count}")
    else:
        print("  none")
    print(f"tof_correction_rows: {len(correction_rows)}")
    print(f"yaw_residual_summary_rows: {len(summary_rows)}")
    if summary_rows:
        print(f"curve_sharpness_mean: {curve_sharpness_mean:.9f}")
        print(f"flat_curve_ratio: {flat_curve_ratio:.6f}")
        print(f"multimodal_ratio: {multimodal_ratio:.6f}")
        print(f"best_at_search_edge_ratio: {best_at_search_edge_ratio:.6f}")
        print(f"num_local_minima_mean: {num_local_minima_mean:.6f}")
        print(f"summary_best_second_yaw_gap_mean_deg: {summary_best_second_yaw_gap_mean_deg:.6f}")
        print(f"curve_p50_minus_min_mean: {curve_p50_minus_min_mean:.9f}")
    print(f"yaw_residual_curve_rows: {len(curve_rows)}")
    if curve_rows:
        print(f"front_residual_mean: {front_residual_mean:.9f}")
        print(f"left_residual_mean: {left_residual_mean:.9f}")
        print(f"right_residual_mean: {right_residual_mean:.9f}")
        print(f"front_valid_ratio: {front_valid_ratio:.6f}")
        print(f"left_valid_ratio: {left_valid_ratio:.6f}")
        print(f"right_valid_ratio: {right_valid_ratio:.6f}")
    if metrics:
        for key in [
            "tof_correction_attempts",
            "tof_correction_accepts",
            "tof_correction_rejects",
            "tof_correction_improvement_mean",
            "pose_quality_mean",
            "pose_quality_min",
            "localization_confidence_mean",
            "localization_confidence_min",
        ]:
            if key in metrics:
                print(f"metric_{key}: {metrics[key]}")

    failures = []
    if strict:
        if total_rows <= 0:
            failures.append("total_rows <= 0")
        if stable_ratio < 0.5:
            failures.append("stable_ratio < 0.5")
        if improvement_mean <= 0.0:
            failures.append("improvement_mean <= 0")
        if improvement_positive_ratio < 0.6:
            failures.append("improvement_positive_ratio < 0.6")
        if used_tof_mean < 2.0:
            failures.append("used_tof_mean < 2.0")
        if filtered_delta_yaw_max_step_rad > 0.05:
            failures.append("filtered_delta_yaw_max_step_rad > 0.05")
        if reason_majority_bad(reason_counts, total_rows):
            failures.append("bad reason majority")
        if failures:
            print("strict: fail")
            for item in failures:
                print(f"  {item}")
            return 1
        print("strict: pass")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Analyze candidate_yaw.csv stability from yaw_candidate runs.")
    parser.add_argument("path", help="run_dir or candidate_yaw.csv path")
    parser.add_argument("--strict", action="store_true", help="return non-zero unless yaw candidate quality passes thresholds")
    args = parser.parse_args()
    raise SystemExit(analyze(args.path, args.strict))


if __name__ == "__main__":
    main()
