#!/usr/bin/env python3
import argparse
import csv
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


def resolve_path(path):
    path = Path(path)
    if path.is_dir():
        return path / "runtime_profile.csv"
    return path


def read_rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def summarize(path, strict=False):
    profile_path = resolve_path(path)
    if not profile_path.exists():
        print(f"runtime_profile_csv: {profile_path}")
        print("error: runtime_profile.csv not found")
        return 1

    rows = read_rows(profile_path)
    elapsed = [as_float(r.get("elapsed_s")) for r in rows]
    cpu = [as_float(r.get("cpu_percent")) for r in rows]
    rss_mb = [as_float(r.get("rss_kb")) / 1024.0 for r in rows]
    threads = [as_int(r.get("thread_count")) for r in rows]

    duration_s = (max(elapsed) - min(elapsed)) if len(elapsed) >= 2 else (elapsed[0] if elapsed else 0.0)
    cpu_percent_mean = sum(cpu) / len(cpu) if cpu else 0.0
    cpu_percent_max = max(cpu) if cpu else 0.0
    rss_mb_mean = sum(rss_mb) / len(rss_mb) if rss_mb else 0.0
    rss_mb_max = max(rss_mb) if rss_mb else 0.0
    thread_count_max = max(threads) if threads else 0

    print(f"runtime_profile_csv: {profile_path}")
    print(f"rows: {len(rows)}")
    print(f"duration_s: {duration_s:.3f}")
    print(f"cpu_percent_mean: {cpu_percent_mean:.3f}")
    print(f"cpu_percent_max: {cpu_percent_max:.3f}")
    print(f"rss_mb_mean: {rss_mb_mean:.3f}")
    print(f"rss_mb_max: {rss_mb_max:.3f}")
    print(f"thread_count_max: {thread_count_max}")

    if strict:
        failures = []
        if duration_s <= 0.0:
            failures.append("duration_s <= 0")
        if rss_mb_max >= 200.0:
            failures.append("rss_mb_max >= 200")
        if cpu_percent_max >= 200.0:
            failures.append("cpu_percent_max >= 200")
        if failures:
            print("strict: fail")
            for item in failures:
                print(f"  {item}")
            return 1
        print("strict: pass")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Summarize robot_slamd runtime_profile.csv.")
    parser.add_argument("path", help="run_dir or runtime_profile.csv path")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    raise SystemExit(summarize(args.path, args.strict))


if __name__ == "__main__":
    main()
